//===-- T8xxStackifyInf.cpp - Arranges the operands on the operand stack ------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the pass which converts floating point instructions from
// pseudo registers into register stack instructions.  This pass uses live
// variable information to indicate where the FPn registers are used and their
// lifetimes.
//
// The x87 hardware tracks liveness of the stack registers, so it is necessary
// to implement exact liveness tracking between basic blocks. The CFG edges are
// partitioned into bundles where the same FP registers must be live in
// identical stack positions. Instructions are inserted at the end of each basic
// block to rearrange the live registers to match the outgoing bundle.
//
// This approach avoids splitting critical edges at the potential cost of more
// live register shuffling instructions when critical edges are present.
//
//===----------------------------------------------------------------------===//

#include "T8xx.h"
#include "T8xxInstrInfo.h"
#include "T8xxSubtarget.h"
#include "T8xxMachineFunctionInfo.h"
#include "T8xxDebugValueManager.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/EdgeBundles.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/Config/llvm-config.h"
#include "llvm/IR/InlineAsm.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include <algorithm>
#include <bitset>

using namespace llvm;

#define DEBUG_TYPE "t8xx-codegen"

STATISTIC(NumFXCH, "Number of fxch instructions inserted");
STATISTIC(NumFP  , "Number of floating point instructions");

namespace llvm {

  struct T8xxStackPass : public MachineFunctionPass {

  protected:
    // Attempt to implement the algorithm to determine the depth of
    // an expression as outlined in the transputer compiler writers
    // guide.
    unsigned int getDepth (MachineInstr *MI,
			   const MachineRegisterInfo &MRI,
			   const LiveIntervals &LIS);

    MachineInstr *reorderRecursive (MachineFunction &MF,
				   MachineInstr *MI,
				   MachineRegisterInfo &MRI,
				    LiveIntervals &LIS,
				    std::vector<MachineInstr *> &output);

  public:
    static char ID;

    T8xxStackPass() : MachineFunctionPass(ID) {
    }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.setPreservesCFG();
      AU.addRequired<MachineDominatorTree>();
      AU.addRequired<EdgeBundles>();
      AU.addRequired<LiveIntervals>();
      AU.addPreservedID(MachineLoopInfoID);
      AU.addPreservedID(LiveVariablesID);
      AU.addPreservedID(MachineDominatorsID);
      AU.addPreserved<MachineDominatorTree>();

      AU.addRequired<VirtRegMap>();
      AU.addPreserved<VirtRegMap>();

      MachineFunctionPass::getAnalysisUsage(AU);
    }

    bool runOnMachineFunction(MachineFunction &MF) override;

    MachineFunctionProperties getRequiredProperties() const override {
      return MachineFunctionProperties();
      /*
      return MachineFunctionProperties().set(
          MachineFunctionProperties::Property::NoVRegs);
      */
    }

    MachineFunctionProperties getSetProperties() const override {
      return MachineFunctionProperties().set(
          MachineFunctionProperties::Property::NoVRegs);
    }

    StringRef getPassName() const override { return "T8xx INT Stackifier"; }

  private:

};

} // end anonymous namespace


using namespace llvm;

char T8xxStackPass::ID = 0;

INITIALIZE_PASS_BEGIN(T8xxStackPass, "t8xxstackifier", "T8xx INT Stackifier",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(EdgeBundles)
INITIALIZE_PASS_END(T8xxStackPass, "t8xxstackifier", "T8xx INT Stackifier",
                    false, false)
/*
INITIALIZE_PASS(T8xxStackPass, "t8xxstackifier", "T8xx INT Stackifier",
                      false, false)
*/

FunctionPass *llvm::createT8xxStackPass() {
  return new T8xxStackPass();
}


namespace {
/// A stack for walking the tree of instructions being built, visiting the
/// MachineOperands in DFS order.
class TreeWalkerState {
  using mop_iterator = MachineInstr::mop_iterator;
  using mop_reverse_iterator = std::reverse_iterator<mop_iterator>;
  using RangeTy = iterator_range<mop_reverse_iterator>;
  SmallVector<RangeTy, 4> Worklist;

public:
  // This puts an iterator to the "explicit_used" operands on the stack
  //  iterator_range<mop_iterator> explicit_uses() {
  //  return make_range(operands_begin() + getNumExplicitDefs(),
  //                    operands_begin() + getNumExplicitOperands());
  //}
  explicit TreeWalkerState(MachineInstr *Insert) {
    const iterator_range<mop_iterator> &Range = Insert->explicit_uses();
    if (!Range.empty())
      Worklist.push_back(reverse(Range));
  }

  bool done() const { return Worklist.empty(); }

  // Get next operand from top element from back of list
  MachineOperand &pop() {
    RangeTy &Range = Worklist.back();
    MachineOperand &Op = *Range.begin();
    Range = drop_begin(Range);
    if (Range.empty())
      Worklist.pop_back();
    assert((Worklist.empty() || !Worklist.back().empty()) &&
           "Empty ranges shouldn't remain in the worklist");
    return Op;
  }

  /// OKH: Dump content of treewalker
  void dump () {
    SmallVector<RangeTy, 4>::iterator wl_iter;
    for (wl_iter = Worklist.begin (); wl_iter != Worklist.end (); ++wl_iter)
      {
	for (auto I = wl_iter->begin (); I != wl_iter->end (); ++I)
	  I->dump ();
      }
  }


  /// Push Instr's operands onto the stack to be visited.
  void pushOperands(MachineInstr *Instr) {
    const iterator_range<mop_iterator> &Range(Instr->explicit_uses());
    if (!Range.empty())
      Worklist.push_back(reverse(Range));
  }

  /// Some of Instr's operands are on the top of the stack; remove them and
  /// re-insert them starting from the beginning (because we've commuted them).
  void resetTopOperands(MachineInstr *Instr) {
    assert(hasRemainingOperands(Instr) &&
           "Reseting operands should only be done when the instruction has "
           "an operand still on the stack");
    Worklist.back() = reverse(Instr->explicit_uses());
  }

  /// Test whether Instr has operands remaining to be visited at the top of
  /// the stack.
  bool hasRemainingOperands(const MachineInstr *Instr) const {
    if (Worklist.empty())
      return false;
    const RangeTy &Range = Worklist.back();
    return !Range.empty() && Range.begin()->getParent() == Instr;
  }

  /// Test whether the given register is present on the stack, indicating an
  /// operand in the tree that we haven't visited yet. Moving a definition of
  /// Reg to a point in the tree after that would change its value.
  ///
  /// This is needed as a consequence of using implicit local.gets for
  /// uses and implicit local.sets for defs.
  bool isOnStack(unsigned Reg) const {
    for (const RangeTy &Range : Worklist)
      for (const MachineOperand &MO : Range)
        if (MO.isReg() && MO.getReg() == Reg)
          return true;
    return false;
  }
};

} // end anonymous namespace


// Identify the definition for this register at this point. This is a
// generalization of MachineRegisterInfo::getUniqueVRegDef that uses
// LiveIntervals to handle complex cases.
static MachineInstr *getVRegDef(unsigned Reg, const MachineInstr *Insert,
                                const MachineRegisterInfo &MRI,
                                const LiveIntervals &LIS) {
  // Most registers are in SSA form here so we try a quick MRI query first.
  if (MachineInstr *Def = MRI.getUniqueVRegDef(Reg))
    return Def;

  /*

  // MRI doesn't know what the Def is. Try asking LIS.
  if (const VNInfo *ValNo = LIS.getInterval(Reg).getVNInfoBefore(
          LIS.getInstructionIndex(*Insert)))
    {
      const LiveInterval &li = LIS.getInterval(Reg);
      li.dump ();
      printf ("SlotIndex %i\n", ValNo->def);
      MachineInstr *temp = LIS.getInstructionFromIndex(ValNo->def);
      if (temp)
	temp->dump ();

    return LIS.getInstructionFromIndex(ValNo->def);
    }
  */


  return nullptr;
}



// Determine whether MI reads memory, writes memory, has side effects,
// and/or uses the stack pointer value.
static void query(const MachineInstr &MI, bool &Read, bool &Write,
                  bool &Effects, bool &StackPointer) {
  assert(!MI.isTerminator());

  if (MI.isDebugInstr() || MI.isPosition())
    return;

  // Check for loads.
  if (MI.mayLoad() && !MI.isDereferenceableInvariantLoad())
    Read = true;

  // Check for stores.
  if (MI.mayStore()) {
    Write = true;
  } else if (MI.hasOrderedMemoryRef()) {
    /*
    switch (MI.getOpcode()) {
    case WebAssembly::DIV_S_I32:
    case WebAssembly::I64_TRUNC_U_F64:
      // These instruction have hasUnmodeledSideEffects() returning true
      // because they trap on overflow and invalid so they can't be arbitrarily
      // moved, however hasOrderedMemoryRef() interprets this plus their lack
      // of memoperands as having a potential unknown memory reference.
      break;
    default:
      // Record volatile accesses, unless it's a call, as calls are handled
      // specially below.
      */
      if (!MI.isCall()) {
        Write = true;
        Effects = true;
      }
      /*
             break;
    }
      */
  }

  // Check for side effects.
  if (MI.hasUnmodeledSideEffects()) {
    /*
    switch (MI.getOpcode()) {
    case WebAssembly::I64_TRUNC_U_F64:
      // These instructions have hasUnmodeledSideEffects() returning true
      // because they trap on overflow and invalid so they can't be arbitrarily
      // moved, however in the specific case of register stackifying, it is safe
      // to move them because overflow and invalid are Undefined Behavior.
      break;
    default:
    */
      Effects = true;
      /*
      break;
    }
      */
  }

  // Check for writes to __stack_pointer global.
/*
  if ((MI.getOpcode() == WebAssembly::GLOBAL_SET_I32 ||
       MI.getOpcode() == WebAssembly::GLOBAL_SET_I64) &&
      strcmp(MI.getOperand(0).getSymbolName(), "__stack_pointer") == 0)
    StackPointer = true;
*/

  // Analyze calls.
  if (MI.isCall()) {
    /*
    queryCallee(MI, Read, Write, Effects, StackPointer);
    */
    // Assume the worst.
    // TODO: Understand function and implement
    Write = true;
    Read = true;
    Effects = true;
  }
}



static bool isSafeToMove(const MachineOperand *Def, const MachineOperand *Use,
                         const MachineInstr *Insert,
                         const T8xxMachineFunctionInfo &MFI,
                         const MachineRegisterInfo &MRI) {

  // TODO: Implement the functionality here
  // For initial test, just assume that the block can be moved
  return true;
}


// Test whether Def is safe and profitable to rematerialize.
static bool shouldRematerialize(const MachineInstr &Def,
                                const T8xxInstrInfo *TII) {
  //  return Def.isAsCheapAsAMove() && TII->isTriviallyReMaterializable(Def);
  return TII->isTriviallyReMaterializable(Def);
}


/// Test whether OneUse, a use of Reg, dominates all of Reg's other uses.
static bool oneUseDominatesOtherUses(unsigned Reg, const MachineOperand &OneUse,
                                     const MachineBasicBlock &MBB,
                                     const MachineRegisterInfo &MRI,
                                     const MachineDominatorTree &MDT,
                                     LiveIntervals &LIS,
                                     T8xxMachineFunctionInfo &MFI) {
  const LiveInterval &LI = LIS.getInterval(Reg);

  const MachineInstr *OneUseInst = OneUse.getParent();
  VNInfo *OneUseVNI = LI.getVNInfoBefore(LIS.getInstructionIndex(*OneUseInst));

  for (const MachineOperand &Use : MRI.use_nodbg_operands(Reg)) {
    if (&Use == &OneUse)
      continue;

    const MachineInstr *UseInst = Use.getParent();
    VNInfo *UseVNI = LI.getVNInfoBefore(LIS.getInstructionIndex(*UseInst));

    if (UseVNI != OneUseVNI)
      continue;

    if (UseInst == OneUseInst) {
      // Another use in the same instruction. We need to ensure that the one
      // selected use happens "before" it.
      if (&OneUse > &Use)
        return false;
    } else {
      // Test that the use is dominated by the one selected use.
      while (!MDT.dominates(OneUseInst, UseInst)) {
        // Actually, dominating is over-conservative. Test that the use would
        // happen after the one selected use in the stack evaluation order.
        //
        // This is needed as a consequence of using implicit local.gets for
        // uses and implicit local.sets for defs.
        if (UseInst->getDesc().getNumDefs() == 0)
          return false;
        const MachineOperand &MO = UseInst->getOperand(0);
        if (!MO.isReg())
          return false;
        Register DefReg = MO.getReg();
        if (!DefReg.isVirtual() || !MFI.isVRegStackified(DefReg))
          return false;
        assert(MRI.hasOneNonDBGUse(DefReg));
        const MachineOperand &NewUse = *MRI.use_nodbg_begin(DefReg);
        const MachineInstr *NewUseInst = NewUse.getParent();
        if (NewUseInst == OneUseInst) {
          if (&OneUse > &NewUse)
            return false;
          break;
        }
        UseInst = NewUseInst;
      }
    }
  }
  return true;
}


// Test whether Reg, as defined at Def, has exactly one use. This is a
// generalization of MachineRegisterInfo::hasOneNonDBGUse that uses
// LiveIntervals to handle complex cases.
static bool hasOneNonDBGUse(unsigned Reg, MachineInstr *Def,
                            MachineRegisterInfo &MRI, MachineDominatorTree &MDT,
                            LiveIntervals &LIS) {
  // Most registers are in SSA form here so we try a quick MRI query first.
  if (MRI.hasOneNonDBGUse(Reg))
    return true;

  bool HasOne = false;
  const LiveInterval &LI = LIS.getInterval(Reg);
  const VNInfo *DefVNI =
      LI.getVNInfoAt(LIS.getInstructionIndex(*Def).getRegSlot());
  assert(DefVNI);
  for (auto &I : MRI.use_nodbg_operands(Reg)) {
    const auto &Result = LI.Query(LIS.getInstructionIndex(*I.getParent()));
    if (Result.valueIn() == DefVNI) {
      if (!Result.isKill())
        return false;
      if (HasOne)
        return false;
      HasOne = true;
    }
  }
  return HasOne;
}


/// A single-use def in the same block with no intervening memory or register
/// dependencies; move the def down and nest it with the current instruction.
static MachineInstr *moveForSingleUse(unsigned Reg, MachineOperand &Op,
                                      MachineInstr *Def, MachineBasicBlock &MBB,
                                      MachineInstr *Insert, LiveIntervals &LIS,
                                      T8xxMachineFunctionInfo &MFI,
                                      MachineRegisterInfo &MRI)

{
  LLVM_DEBUG(dbgs() << "Move for single use: "; Def->dump());

  // OKH: Note, the T8xxDebugValueManager is derived from WebAssemblyDVM and
  // does the actual work of shifting instructions!
  // I.e. the "sink" method already moves the Def instruction to the place
  // before Insert.

  T8xxDebugValueManager DefDIs(Def);
  DefDIs.sink(Insert);
  //  LIS.handleMove(*Def);

  if (MRI.hasOneDef(Reg) && MRI.hasOneNonDBGUse(Reg)) {
    // No one else is using this register for anything so we can just stackify
    // it in place.
    //    MFI.stackifyVReg(MRI, Reg);
    //    printf ("Reg Stackified %u\n", Reg);
  } else {
    // The register may have unrelated uses or defs; create a new register for
    // just our one def and use so that we can stackify it.
    Register NewReg = MRI.createVirtualRegister(MRI.getRegClass(Reg));
    Op.setReg(NewReg);
    //    DefDIs.updateReg(NewReg);

    // Tell LiveIntervals about the new register.
    LIS.createAndComputeVirtRegInterval(NewReg);

    // Tell LiveIntervals about the changes to the old register.
    LiveInterval &LI = LIS.getInterval(Reg);
    LI.removeSegment(LIS.getInstructionIndex(*Def).getRegSlot(),
		     LIS.getInstructionIndex(*Op.getParent()).getRegSlot(),
                         /*RemoveDeadValNo=*/  true);

    // TODO:
    //MFI.stackifyVReg(MRI, NewReg);

    LLVM_DEBUG(dbgs() << " - Replaced register: "; Def->dump());
  }

  //  imposeStackOrdering(Def);
  return Def;
}


static MachineInstr *getPrevNonDebugInst(MachineInstr *MI) {
  for (auto *I = MI->getPrevNode(); I; I = I->getPrevNode())
    if (!I->isDebugInstr())
      return I;
  return nullptr;
}

// Shrink LI to its uses, cleaning up LI.
static void shrinkToUses(LiveInterval &LI, LiveIntervals &LIS) {
  if (LIS.shrinkToUses(&LI)) {
    SmallVector<LiveInterval *, 4> SplitLIs;
    LIS.splitSeparateComponents(LI, SplitLIs);
  }
}



/// A trivially cloneable instruction; clone it and nest the new copy with the
/// current instruction.
static MachineInstr *rematerializeCheapDef(
    unsigned Reg, MachineOperand &Op, MachineInstr &Def, MachineBasicBlock &MBB,
    MachineBasicBlock::instr_iterator Insert, LiveIntervals &LIS,
    T8xxMachineFunctionInfo &MFI, MachineRegisterInfo &MRI,
    const T8xxInstrInfo *TII, const T8xxRegisterInfo *TRI) {
  LLVM_DEBUG(dbgs() << "Rematerializing cheap def: "; Def.dump());
  LLVM_DEBUG(dbgs() << " - for use in "; Op.getParent()->dump());

  //  WebAssemblyDebugValueManager DefDIs(&Def);

  Register NewReg = MRI.createVirtualRegister(MRI.getRegClass(Reg));
  //  DefDIs.cloneSink(&*Insert, NewReg);
  Op.setReg(NewReg);
  MachineInstr *Clone = getPrevNonDebugInst(&*Insert);
  assert(Clone);
  LIS.InsertMachineInstrInMaps(*Clone);
  LIS.createAndComputeVirtRegInterval(NewReg);
  MFI.stackifyVReg(MRI, NewReg);
  //  imposeStackOrdering(Clone);

  LLVM_DEBUG(dbgs() << " - Cloned to "; Clone->dump());

  // Shrink the interval.
  bool IsDead = MRI.use_empty(Reg);
  if (!IsDead) {
    LiveInterval &LI = LIS.getInterval(Reg);
    shrinkToUses(LI, LIS);
    IsDead = !LI.liveAt(LIS.getInstructionIndex(Def).getDeadSlot());
  }

  // If that was the last use of the original, delete the original.
  if (IsDead) {
    LLVM_DEBUG(dbgs() << " - Deleting original\n");
    //SlotIndex Idx = LIS.getInstructionIndex(Def).getRegSlot();
    // ??
    //    LIS.removePhysRegDefAt(MCRegister::from(WebAssembly::ARGUMENTS), Idx);
    LIS.removeInterval(Reg);
    LIS.RemoveMachineInstrFromMaps(Def);
    // DefDIs.removeDef();
  }

  return Clone;
}



unsigned int T8xxStackPass::getDepth (MachineInstr *MI,
				      const MachineRegisterInfo &MRI,
				      const LiveIntervals &LIS)
{
      // Debugging Write out all definitions and operators
      const iterator_range<MachineInstr::mop_iterator> &Range_defs = MI->defs();
      const iterator_range<MachineInstr::mop_iterator> &Range_uses = MI->explicit_uses();
      int RegDefCount = 0,
	RegUseCount = 0;
      unsigned int DepthE = 0,
	DepthSubE = 0;

      // Find out how many registers are defined and how many are needed as input
      for (auto I = Range_defs.begin (); I != Range_defs.end (); ++I)
	{
	  if (I->isReg())
	    ++RegDefCount;
	}
      for (auto I = Range_uses.begin (); I != Range_uses.end (); ++I)
	{
	  if (I->isReg())
	    ++RegUseCount;
	}

      if (RegUseCount == 0)
	DepthE = RegDefCount;
      else
	{
	  SmallVector<int, 4> OpDepth;

	  for (auto I = Range_uses.begin (); I != Range_uses.end (); ++I)
	    {
	      if (I->isReg () && !I->getReg().isPhysical())
		{
		  Register Reg = I->getReg();
		  MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
		  if (DefI)
		    {
		      unsigned int SubE = getDepth (DefI, MRI, LIS);
		      OpDepth.push_back(SubE);
		    }
		  else
		    {
		      // Try to find all definitions of the register.
		      MachineRegisterInfo::def_instr_iterator def_reg = MRI.def_instr_begin(Reg);
		      while (def_reg != MRI.def_instr_end())
			{
			  def_reg->dump();
			  ++def_reg;
			}

		      // TODO: Not entirely sure, if this is really the correct answer
		      // If the definition is in the same block, it might still be possible
		      // to shift the instruction
		      OpDepth.push_back (10000);
		    }
		}
	    }

	  if (MI->getOpcode() == T8xx::REV)
	    // REV is special in that it does not change the depth of the instruction.
	    DepthE = DepthSubE;
	  else
	    {
	      switch (OpDepth.size ())
		{
		case 0: DepthE = 1;
		  break;
		case 1: DepthE = OpDepth[0];
		  break;
		case 2:
		  {
		    if (OpDepth[0] > OpDepth[1])
		      DepthE = OpDepth[0];
		    else
		      {
			if (OpDepth[0] < OpDepth[1])
			  DepthE = OpDepth[1];
			else
			  DepthE = OpDepth[0] + 1;
		      }
		  }
		  break;
		default:
		  DepthE = 1;
		}
	    }
	}

      // Determine the depth on an instruction
      // Lower limit is the maximum of produced and used stack registers
      // LowLimit = max(RegDefCount,RegUseCount)
      //
      // RegUseCount needs to be adjusted to functions that need more registers to fill one used Reg
      // Largest register use + RegUse Count - 1

      /*
      printf ("---> Insert structure\n");
      MI->dump ();
      printf ("RegUseCount %i  RegDefCount %i  DepthE %i  DepthSubE %i\n", RegUseCount, RegDefCount, DepthE, DepthSubE);
      */
      //      printf ("<--- Insert structure  Defs = %i   Uses = %i\n", RegDefCount, RegUseCount);
      return (DepthE);
}


MachineInstr *T8xxStackPass::reorderRecursive (MachineFunction &MF,
					      MachineInstr *MI,
					      MachineRegisterInfo &MRI,
					       LiveIntervals &LIS,
					       std::vector<MachineInstr *> &output)
{
  T8xxMachineFunctionInfo &MFI = *MF.getInfo<T8xxMachineFunctionInfo>();
  MachineBasicBlock *MBB = MI->getParent ();
  const auto *TII = MF.getSubtarget<T8xxSubtarget>().getInstrInfo();
  const auto *TRI = MF.getSubtarget<T8xxSubtarget>().getRegisterInfo();
  auto &MDT = getAnalysis<MachineDominatorTree>();

  // Debugging Write out all definitions and operators
  const iterator_range<MachineInstr::mop_iterator> &Range_defs = MI->defs();
  const iterator_range<MachineInstr::mop_iterator> &Range_uses = MI->explicit_uses();
  int RegDefCount = 0,
    RegUseCount = 0;
  unsigned int DepthE = 0,
    DepthSubE = 0;

  MachineInstr *Insert = MI;

  // Vector to hold the depth / operand register usage of the
  // preceding operations
  SmallVector<std::pair<int, MachineOperand *>, 4> OpDepth;

  // Find out how many registers are defined and how many are needed as input
  // When a variable has multiple definitions, put "-1" on the register stack
  MI->dump ();
  for (auto I = Range_uses.begin (); I != Range_uses.end (); ++I)
    {
      //      if (I->isReg () && !I->getReg().isPhysical())
      if (I->isReg ())
	{
	  Register Reg = I->getReg();
	  MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
	  if (DefI)
	    {
	      int SubE = getDepth (DefI, MRI, LIS);
	      OpDepth.push_back (std::make_pair(SubE, I));
	    }
	  else
	    // Register with multiple definitions or those, where it is
	    // not possible to move the respective instruction get
	    // depth "10000" (arbitrary value)
	    OpDepth.push_back (std::make_pair(10000, I));

	  //	  printf ("Op Depth %i\n", OpDepth.back ());
	}
    }

  if (OpDepth.size () == 1)
    {
      printf ("Reorder Depth 1  %i\n", OpDepth[0].first);
      // Move instruction ahead of current instruction and then move on
      // to definition
      MachineOperand *Use = OpDepth[0].second;
      Register Reg = Use->getReg ();
      MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
      /*
      Insert = moveForSingleUse(Reg, *Use, DefI, *MBB,
				MI, LIS, MFI, MRI);
      */
      reorderRecursive (MF, DefI, MRI, LIS, output);
    }

  if (OpDepth.size () == 2)
    {
      printf ("Reorder Depth 2,  %i %i\n", OpDepth[0].first, OpDepth[1].first);
      // Move instruction ahead of current instruction and then move on
      // to definition

      if (OpDepth[1].first > OpDepth[0].first)
	{
	  if (OpDepth[0].first > 2)
	    {
	      printf ("Br A\n");
	      MachineOperand *Use = OpDepth[1].second;
	      Register Reg = Use->getReg ();
	      MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
	      /*
	      Insert = moveForSingleUse(Reg, *Use, DefI, *MBB,
						      MI, LIS, MFI, MRI);
	      */
	      reorderRecursive (MF, DefI, MRI, LIS, output);

	      // Introduce temporary variable
	      // Simply introduce a workspace register
	      /*
	      if (VRM.isAssignedReg (Reg))
		VRM.assignVirt2StackSlot (Reg);

	      DebugLoc DL = Instr->getDebugLoc();
	      MachineBasicBlock::iterator MBBI = *DefI;
	      BuildMI(*MBB, ++MBBI, DL, TII->get(T8xx::STL)).addReg(Reg).addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);
	      */

	      // Now the second operand
	      Use = OpDepth[0].second;
	      Register Reg2 = Use->getReg ();
	      DefI = getVRegDef(Reg2, MI, MRI, LIS);
	      /*
	      Insert = moveForSingleUse(Reg2, *Use, DefI, *MBB,
					MI, LIS, MFI, MRI);
	      */
	      reorderRecursive (MF, DefI, MRI, LIS, output);

	      // Insert "ldl" for temporary variable
	      /*
	      DebugLoc DL = DefI->getDebugLoc();
	      MachineBasicBlock::iterator MBBI = *MI;
	      // Note: The AREG is used later to identify this as a special code segment
	      BuildMI(MBB, MBBI, DL, TII->get(T8xx::LDL),T8xx::AREG).addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);
	      */
	    }
	  else
	    // TODO: Check for commuting operators
	    {
	      printf ("Br B\n");
	      MachineOperand *Use = OpDepth[1].second;
	      Register Reg = Use->getReg ();
	      MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
	      /*
	      Insert = moveForSingleUse(Reg, *Use, DefI, *MBB,
						      MI, LIS, MFI, MRI);
	      */
	      reorderRecursive (MF, DefI, MRI, LIS, output);

	      Use = OpDepth[0].second;
	      Reg = Use->getReg ();
	      DefI = getVRegDef(Reg, MI, MRI, LIS);
	      /*
	      Insert = moveForSingleUse(Reg, *Use, DefI, *MBB,
					MI, LIS, MFI, MRI);
	      */
	      reorderRecursive (MF, DefI, MRI, LIS, output);
	    }
	}
      else
	{
	  if (OpDepth[1].first < 3)
	    {
	      printf ("Br C\n");
	      MachineOperand *Use = OpDepth[0].second;
	      Register Reg = Use->getReg ();
	      MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
	      /*
	      Insert = moveForSingleUse(Reg, *Use, DefI, *MBB,
						      MI, LIS, MFI, MRI);
	      */
	      reorderRecursive (MF, DefI, MRI, LIS, output);

	      Use = OpDepth[1].second;
	      Reg = Use->getReg ();
	      DefI = getVRegDef(Reg, MI, MRI, LIS);
	      /*
	      Insert = moveForSingleUse(Reg, *Use, DefI, *MBB,
					MI, LIS, MFI, MRI);
	      */
	      reorderRecursive (MF, DefI, MRI, LIS, output);
	    }
	  else
	    {
	      printf ("Br D\n");
	      MachineOperand *Use = OpDepth[1].second;
	      Register Reg = Use->getReg ();
	      MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
	      /*
	      Insert = moveForSingleUse(Reg, *Use, DefI, *MBB,
						      MI, LIS, MFI, MRI);
	      */
	      reorderRecursive (MF, DefI, MRI, LIS, output);
	      // Store in temporary variable

	      Use = OpDepth[0].second;
	      Reg = Use->getReg ();
	      DefI = getVRegDef(Reg, MI, MRI, LIS);
	      /*
	      Insert = moveForSingleUse(Reg, *Use, DefI, *MBB,
					MI, LIS, MFI, MRI);
	      */
	      reorderRecursive (MF, DefI, MRI, LIS, output);
	      // Load temporary variable
	    }
	}
    }

  //  MI->dump ();
  output.push_back (MI);

  return (Insert);
}


/*

      //  Insert iterator to the used operands (may be registers)
      TreeWalkerState TreeWalker(Insert);
      while (!TreeWalker.done()) {
	printf ("---------------------\nTreewalker dump\n");
	TreeWalker.dump ();
	printf ("Treewalker dump end\n");

        MachineOperand &Use = TreeWalker.pop();

	// Debug
	printf ("MO dump\n");
	Insert->dump ();

	// We're only interested in explicit virtual register operands.
        if (!Use.isReg())
          continue;

        Register Reg = Use.getReg();
        assert(Use.isUse() && "explicit_uses() should only iterate over uses");
        assert(!Use.isImplicit() &&
               "explicit_uses() should only iterate over explicit operands");
        if (Reg.isPhysical())
          continue;

        // Identify the definition for this register at this point.
	if (MRI.hasOneDef(Reg) && MRI.hasOneNonDBGUse(Reg)) {
	  printf ("Has one def %i\n", Reg);
	}
	// Create temporary variable
	else
	  {
	    // Introduce a workspace register
	    if (VRM.isAssignedReg (Reg))
	      VRM.assignVirt2StackSlot (Reg);
	    VRM.dump ();

	    // Insert a "ldl" for the workspace register before the instruction
	    DebugLoc DL = Insert->getDebugLoc();
	    MachineBasicBlock::iterator MBBI = *Insert;
	    // Note: The AREG is used later to identify this as a special code segment
	    BuildMI(MBB, MBBI, DL, TII->get(T8xx::LDL),T8xx::AREG).addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);

	    // Move iterator to "LDL"
	    --MBBI;
	    Insert = &(*MBBI);

	    continue;
	  }

	MachineInstr *DefI = getVRegDef(Reg, Insert, MRI, LIS);
        if (!DefI)
	  continue;

	// Debug
	printf ("Defined by \n");
	DefI->dump ();

	// Don't nest an INLINE_ASM def into anything, because we don't have
        // constraints for $pop outputs.
        if (DefI->isInlineAsm())
          continue;

        MachineOperand *Def = DefI->findRegisterDefOperand(Reg);
        assert(Def != nullptr);

	printf ("Operand\n");
	Def->dump ();
	// OKH: DefI is the instruction that defines the register, Def is the defined operand


	// Decide which strategy to take. Prefer to move a single-use value
        // over cloning it, and prefer cloning over introducing a tee.
        // For moving, we require the def to be in the same block as the use;
        // this makes things simpler (LiveIntervals' handleMove function only
        // supports intra-block moves) and it's MachineSink's job to catch all
        // the sinking opportunities anyway.
        bool SameBlock = DefI->getParent() == &MBB;
        bool CanMove = SameBlock && isSafeToMove(Def, &Use, Insert, MFI, MRI) &&
                       !TreeWalker.isOnStack(Reg);

	// Loading a constant does not depend on anything and can always be moved,
	// even between blocks
	if (DefI->getOpcode() == T8xx::LDC)
	  CanMove = true;

	printf ("Move for single use SameBlock %i   CanMove %i\n", SameBlock, CanMove);

	// Note: Problems occur with instructions that don't consume
	// any registers. In that case, the instruction should be skipped.

	if (CanMove && hasOneNonDBGUse(Reg, DefI, MRI, MDT, LIS)) {
          Insert = moveForSingleUse(Reg, Use, DefI, MBB, Insert, LIS, MFI, MRI);
	}
	else {
	  // Simply introduce a workspace register
	  if (VRM.isAssignedReg (Reg))
	    VRM.assignVirt2StackSlot (Reg);

	  VRM.dump ();

	  // Insert a "ldl" for the workspace register before the instruction
	  DebugLoc DL = Insert->getDebugLoc();
	  MachineBasicBlock::iterator MBBI = *Insert;
	  // Note: The AREG is used later to identify this as a special code segment
	  BuildMI(MBB, MBBI, DL, TII->get(T8xx::LDL),T8xx::AREG).addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);

	  // Move iterator to "LDL"
	  --MBBI;
	  Insert = &(*MBBI);

	  continue;
	}

#if 0
	else if (shouldRematerialize(*DefI, TII)) {
	  printf ("Rematerialize\n");
          Insert =
              rematerializeCheapDef(Reg, Use, *DefI, MBB, Insert->getIterator(),
                                    LIS, MFI, MRI, TII, TRI);
        } else if (CanMove && oneUseDominatesOtherUses(Reg, Use, MBB, MRI, MDT,
                                                       LIS, MFI)) {
	  printf ("MoveAndTeeForMultiUse\n");
	  //          Insert = moveAndTeeForMultiUse(Reg, Use, DefI, MBB, Insert, LIS, MFI,
	//	      MRI, TII);
	    Insert = DefI;
        } else {
          // We failed to stackify the operand. If the problem was ordering
          // constraints, Commuting may be able to help.
          // Proceed to the next operand.
          continue;
        }
#endif

        // We stackified an operand. Add the defining instruction's operands to
        // the worklist stack now to continue to build an ever deeper tree.
	//        Commuting.reset();
        TreeWalker.pushOperands(Insert);
      }
*/



/// runOnMachineFunction - Loop over all of the basic blocks, transforming FP
/// register references into FP stack references.
///
bool T8xxStackPass::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "********** Register Stackifying **********\n"
                       "********** Function: "
                    << MF.getName() << '\n');

  MachineRegisterInfo &MRI = MF.getRegInfo();
  T8xxMachineFunctionInfo &MFI = *MF.getInfo<T8xxMachineFunctionInfo>();
  const auto *TII = MF.getSubtarget<T8xxSubtarget>().getInstrInfo();
  const auto *TRI = MF.getSubtarget<T8xxSubtarget>().getRegisterInfo();
  auto &MDT = getAnalysis<MachineDominatorTree>();
  auto &LIS = getAnalysis<LiveIntervals>();

  // OKH: Try to use the virtual register map
  auto &VRM = getAnalysis<VirtRegMap>();
  printf ("############ Register Map\n");
  VRM.dump ();

  // LiveInterval dump
  printf ("############ LiveInterval Map\n");
  LIS.dump ();

  // Some map to keep track of registers that have already been created as
  // workspace registers
  std::map<unsigned, unsigned> wp_reg_map;

  // Walk the instructions from the top down. Currently we don't look past
  // block boundaries, and the blocks aren't ordered so the block visitation
  // order isn't significant, but we may want to change this in the future.
  for (MachineBasicBlock &MBB : MF) {

    // For test purposes let's leave with going through the first instruction
    //MachineBasicBlock &MBB = *MF.begin();

    int count = 0;

    // Don't use a range-based for loop, because we modify the list as we're
    // iterating over it and the end iterator may change.
    std::vector<MachineInstr *> outvec;
    for (auto MII = MBB.begin(); MII != MBB.end(); ++MII)
      {
	printf ("######## NewInstruction");

	MachineInstr *Insert = &*MII;

	// Don't nest anything inside an inline asm, because we don't have
	// constraints for $push inputs.
	if (Insert->isInlineAsm())
	  continue;

	// Ignore debugging intrinsics.
	if (Insert->isDebugValue())
	  continue;

	// When the instruction does not define anything, it is a store
	// instruction and should be recursed
	const iterator_range<MachineInstr::mop_iterator> &Range_defs = Insert->defs();
	if (Range_defs.begin () == Range_defs.end ())
	  {
            // Calculate depth (according to Transputer compiler writing guide
	    unsigned int MIDepth = getDepth (Insert, MRI, LIS);
	    //      printf ("--> Instruction Depth = %i\n", MIDepth);

	    // Reorder instructions (according to Transputer compiler writing guide)
	    Insert = reorderRecursive (MF, Insert, MRI, LIS, outvec);
	  }

      }  // MachineInstruction

    printf ("Print sequence\n");
    std::map<Register, int> vreg_map;

    for (auto O = outvec.begin (); O != outvec.end (); ++O)
      {
	(*O)->dump ();

	// Try to analyse how often each virtual register is used
	const iterator_range<MachineInstr::mop_iterator> &Range_defs = (*O)->defs();

	// Find out how many registers are defined and how many are needed as input
	for (auto I = Range_defs.begin (); I != Range_defs.end (); ++I)
	  {
	    if (I->isReg())
	      {
		Register reg = I->getReg ();
		if (vreg_map.find (reg) != vreg_map.end ())
		  vreg_map[reg]++;
		else
		  vreg_map[reg] = 1;
	      }
	  }

      }
    printf ("End Print sequence\n\n");

    printf ("Def usage\n");
    for (auto I = vreg_map.begin (); I != vreg_map.end (); ++I)
      printf ("ID %i  Count %i\n", I->first.id(), I->second);
    printf ("End Def usage\n");


    // Now the processing depends on how the definitions are
    // used.
    // Intermittend writes -> copy to temp
    // Expensive operation -> copy to temp
    // Cheap instruction -> duplicate
    // Single use -> move

    printf ("Rearranging instructions\n");
    MachineInstr *cur = NULL, *prev = NULL;

    for (auto O = outvec.begin (); O != outvec.end (); ++O)
      {
	(*O)->dump ();
	// Try to analyse how often each virtual register is used
	const iterator_range<MachineInstr::mop_iterator> &Range_defs = (*O)->defs();

	cur = *O;

	if (cur && prev)
	  {
	    T8xxDebugValueManager DefDIs(prev);
	    DefDIs.sink(cur);
	  }

	prev = cur;
      }

  } // MachineBasicBlock

  // Insert some code to save the virtual register in a stack slot
  // after the definition
  printf ("############ Convert to stack registers\n");

  for (MachineBasicBlock &MBB : MF) {

  for (auto MII = MBB.begin(); MII != MBB.end(); ++MII) {
      MachineInstr *Instr = &*MII;

      // Don't nest anything inside an inline asm, because we don't have
      // constraints for $push inputs.
      if (Instr->isInlineAsm())
        continue;

      // Ignore debugging intrinsics.
      if (Instr->isDebugValue())
        continue;

      if (Instr->getNumExplicitDefs () > 0)
	{
	  const iterator_range<MachineInstr::mop_iterator> &Range(Instr->defs());
	  if (Range.begin()->isReg())
	    {
	      Register Reg = Range.begin()->getReg();

	      if (Reg.isVirtual() && !VRM.isAssignedReg(Reg))
		{
		  dbgs() << printReg(Reg, TRI) << "\n";
		  Instr->dump ();

		  DebugLoc DL = Instr->getDebugLoc();
		  MachineBasicBlock::iterator MBBI = *Instr;
		  BuildMI(MBB, ++MBBI, DL, TII->get(T8xx::STL)).addReg(Reg).addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);
		}
	    }
	}
  } // MachineInstr

  } // MachineBasicBlock


  // ######### Replace virtual registers with the stack registers

    for (MachineBasicBlock &MBB : MF) {

  for (auto MII = MBB.begin(); MII != MBB.end(); ++MII) {
      MachineInstr *Instr = &*MII;

      Instr->dump ();

      // Don't nest anything inside an inline asm, because we don't have
      // constraints for $push inputs.
      if (Instr->isInlineAsm())
        continue;

      // Ignore debugging intrinsics.
      if (Instr->isDebugValue())
        continue;

      // Definition is always AREG
      const iterator_range<MachineInstr::mop_iterator> &Range_defs(Instr->defs());
      for (auto OP = Range_defs.begin(); OP != Range_defs.end (); ++OP)
	{
	  if (OP->isReg())
	    {
	      Register Reg = OP->getReg();

	      if (Reg.isVirtual())
		{
		  OP->setReg(T8xx::AREG);
		}
	    }
	}

      // Input operands are numbered from AREG up to CREG ...
      const iterator_range<MachineInstr::mop_iterator> &Range(Instr->explicit_uses());
      int RegAdd = 0;
      for (auto OP = Range.begin(); OP != Range.end (); ++OP)
	{
	  if (OP->isReg())
	    {
	      Register Reg = OP->getReg();
	      //	      printf ("Num  %i  Reg %i\n", RegAdd, Reg);

	      if ((MRI.reg_begin (Reg) != MRI.reg_end ()) && Reg.isVirtual())
		{
		  switch (RegAdd)
		    {
		    case 0: OP->setReg(T8xx::AREG);
		      break;
		    case 1: OP->setReg(T8xx::BREG);
		      break;
		    case 2: OP->setReg(T8xx::CREG);
		      break;
		    }
		  ++RegAdd;
		}
	    }
	}

  } // MachineInstr

  } // MachineBasicBlock



#if 0
  // If we used VALUE_STACK anywhere, add it to the live-in sets everywhere so
  // that it never looks like a use-before-def.
  if (Changed) {
    MF.getRegInfo().addLiveIn(WebAssembly::VALUE_STACK);
    for (MachineBasicBlock &MBB : MF)
      MBB.addLiveIn(WebAssembly::VALUE_STACK);
  }

#ifndef NDEBUG
  // Verify that pushes and pops are performed in LIFO order.
  SmallVector<unsigned, 0> Stack;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      if (MI.isDebugInstr())
        continue;
      for (MachineOperand &MO : reverse(MI.explicit_uses())) {
        if (!MO.isReg())
          continue;
        Register Reg = MO.getReg();
        if (MFI.isVRegStackified(Reg))
          assert(Stack.pop_back_val() == Reg &&
                 "Register stack pop should be paired with a push");
      }
      for (MachineOperand &MO : MI.defs()) {
        if (!MO.isReg())
          continue;
        Register Reg = MO.getReg();
        if (MFI.isVRegStackified(Reg))
          Stack.push_back(MO.getReg());
      }
    }
    // TODO: Generalize this code to support keeping values on the stack across
    // basic block boundaries.
    assert(Stack.empty() &&
           "Register stack pushes and pops should be balanced");
  }
#endif

#endif



  /*
  for (unsigned i = 0, e = MRI.getNumVirtRegs(); i != e; ++i) {
    Register Reg = Register::index2VirtReg(i);
    if (!VRM.isAssignedReg(Reg))
      {
	dbgs() << printReg(Reg, TRI) << "\n";

	// This one does some extra checks!?
	//MachineInstr *DefI = getVRegDef(Reg, Insert, MRI, LIS);
	MachineInstr *DefI = MRI.getUniqueVRegDef(Reg);
	DefI->dump ();
      }
  }
  */

  /*
  */


  printf ("############ Register Map\n");
  VRM.dump ();

  //  return Changed;
  return false;
}
