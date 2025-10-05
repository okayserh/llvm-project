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
#include "llvm/CodeGen/VirtRegMap.h"
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

#define DEBUG_TYPE "t8xx-stackify"

STATISTIC(NumFXCH, "Number of fxch instructions inserted");
STATISTIC(NumFP  , "Number of floating point instructions");

namespace llvm {

  struct T8xxStackPass : public MachineFunctionPass {

  protected:
    // Aggregate register class to indicate whether a register
    // of that class takes space on the floating point register/operand
    // stack or on the integer register/operand stack
    enum T8xxRegStack
      {
	TRS_Int,
	TRS_Float
      };

    T8xxRegStack mapRegisterStack (unsigned RegClassID);
    
    // Attempt to implement the algorithm to determine the depth of
    // an expression as outlined in the transputer compiler writers
    // guide.
    unsigned int getDepth (MachineInstr *MI,
			   const MachineRegisterInfo &MRI,
			   const LiveIntervals &LIS,
			   T8xxRegStack RegStack);

    MachineInstr *reorderRecursive (MachineFunction &MF,
				   MachineInstr *MI,
				   MachineRegisterInfo &MRI,
				    LiveIntervals &LIS,
				    VirtRegMap &VRM,
				    std::vector<MachineInstr *> &output);

  public:
    static char ID;

    T8xxStackPass() : MachineFunctionPass(ID) {
    }

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.setPreservesCFG();
      AU.addRequired<MachineDominatorTreeWrapperPass>();
      AU.addRequired<EdgeBundlesWrapperLegacy>();
      AU.addRequired<LiveIntervalsWrapperPass>();
      AU.addPreservedID(MachineLoopInfoID);
      AU.addPreservedID(LiveVariablesID);
      AU.addPreservedID(MachineDominatorsID);
      AU.addPreserved<MachineDominatorTreeWrapperPass>();

      AU.addRequired<VirtRegMapWrapperLegacy>();
      AU.addPreserved<VirtRegMapWrapperLegacy>();

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
INITIALIZE_PASS_END(T8xxStackPass, "t8xxstackifier", "T8xx INT Stackifier",
                    false, false)

FunctionPass *llvm::createT8xxStackPass() {
  return new T8xxStackPass();
}


  /*
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
  */


// Identify the definition for this register at this point. This is a
// generalization of MachineRegisterInfo::getUniqueVRegDef that uses
// LiveIntervals to handle complex cases.
static MachineInstr *getVRegDef(unsigned Reg, const MachineInstr *Insert,
                                const MachineRegisterInfo &MRI,
                                const LiveIntervals &LIS) {
  // Most registers are in SSA form here so we try a quick MRI query first.
  if (MachineInstr *Def = MRI.getUniqueVRegDef(Reg))
    return Def;

  // MRI doesn't know what the Def is. Try asking LIS.
  if (const VNInfo *ValNo = LIS.getInterval(Reg).getVNInfoBefore(
          LIS.getInstructionIndex(*Insert)))
    {
      LLVM_DEBUG({
	  MachineInstr *temp = LIS.getInstructionFromIndex(ValNo->def);
	  if (temp)
	    temp->dump ();
	});

    return LIS.getInstructionFromIndex(ValNo->def);
    }

  return nullptr;
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


T8xxStackPass::T8xxRegStack T8xxStackPass::mapRegisterStack (unsigned RegClassID)
{
  switch (RegClassID)
    {
    case T8xx::LRegRegClassID:
    case T8xx::ORegRegClassID:
      return TRS_Int;
      break;
    case T8xx::FPRegRegClassID:
    case T8xx::DFPRegRegClassID:
      return TRS_Float;
      break;
    default:
      llvm_unreachable("Unknow register class!");
    }
}

unsigned int T8xxStackPass::getDepth (MachineInstr *MI,
				      const MachineRegisterInfo &MRI,
				      const LiveIntervals &LIS,
				      T8xxRegStack RegStack)
				      //				      unsigned RegClassID)
{
      // Debugging Write out all definitions and operators
      const iterator_range<MachineInstr::mop_iterator> &Range_defs = MI->defs();
      const iterator_range<MachineInstr::mop_iterator> &Range_uses = MI->explicit_uses();
      unsigned int RegDefCount = 0,
	RegUseCount = 0;
      unsigned int DepthE = 0,
	DepthSubE = 0;

      // Find out how many registers are defined and how many are needed as input
      for (auto I = Range_defs.begin (); I != Range_defs.end (); ++I)
	{
	  // Count only definitions and uses that use space on the relevant register stack
	  if (I->isReg() &&
	      !I->getReg().isPhysical ())
	    {
	      unsigned RegClassID = MRI.getRegClassOrNull (I->getReg ())->getID ();
	      if (mapRegisterStack (RegClassID) == RegStack)
		{
		  ++RegDefCount;
		  // "Long registers" are a construct to get the long instructions
		  // properly into the SSA form. However, long registers take up
		  // two spaces on the register/operand stack
		  if (RegClassID == T8xx::LRegRegClassID)
		    ++RegDefCount;
		}
	    }
	}
      for (auto I = Range_uses.begin (); I != Range_uses.end (); ++I)
	{
	  if (I->isReg() &&
	      !I->getReg().isPhysical ())
	    {
	      unsigned RegClassID = MRI.getRegClassOrNull (I->getReg ())->getID ();
	      if (mapRegisterStack (RegClassID) == RegStack)
		{
		  ++RegUseCount;
		  // "Long registers" are a construct to get the long instructions
		  // properly into the SSA form. However, long registers use
		  // two spaces on the register/operand stack
		  if (RegClassID == T8xx::LRegRegClassID)
		    ++RegUseCount;
		}
	    }
	}

      if (RegUseCount == 0)
	DepthE = RegDefCount;
      else
	{
	  for (auto I = Range_uses.begin (); I != Range_uses.end (); ++I)
	    {
	      if (I->isReg () &&
		  !I->getReg().isPhysical() &&
		  (mapRegisterStack(MRI.getRegClassOrNull (I->getReg ())->getID ()) == RegStack))
		{
		  Register Reg = I->getReg();
		  MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
		  if (DefI)
		    {
		      unsigned int SubE = getDepth (DefI, MRI, LIS, RegStack);
		      if (SubE > DepthSubE)
			DepthSubE = SubE;
		    }
		  else
		    {
		      LLVM_DEBUG(dbgs() << "getDepth -> Multiple definitions\n");
		      // Try to find all definitions of the register.
		      MachineRegisterInfo::def_instr_iterator def_reg = MRI.def_instr_begin(Reg);
		      while (def_reg != MRI.def_instr_end())
			{
			  LLVM_DEBUG({
			      def_reg->dump();
			    });
			  ++def_reg;
			}
		    }
		}
	    }

	  if (MI->getOpcode() == T8xx::REV)
	    // REV is special in that it does not change the depth of the instruction.
	    DepthE = DepthSubE;
	  else
	    {
	      DepthE = DepthSubE + (RegUseCount - 1);
	      if (RegDefCount > DepthE)
		DepthE = RegDefCount;
	    }
	}

      // Determine the depth on an instruction
      // Lower limit is the maximum of produced and used stack registers
      // LowLimit = max(RegDefCount,RegUseCount)
      //
      // RegUseCount needs to be adjusted to functions that need more registers to fill one used Reg
      // Largest register use + RegUse Count - 1

      return (DepthE);
}


/*
 * Either inserts the instruction that defines "Use"
 * before the instruction "MI" or clones the instruction
 * and inserts the clone before instruction "MI".
 */

MachineInstr *SpliceOrCloneInstruction (MachineFunction &MF,
			  MachineBasicBlock *MBB,
			  MachineRegisterInfo &MRI,
			  LiveIntervals &LIS,
			  VirtRegMap &VRM,
			  MachineInstr *MI,
			  MachineOperand *Use)
{
  Register Reg = Use->getReg ();
  MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
  const auto *TII = MF.getSubtarget<T8xxSubtarget>().getInstrInfo();

  if (!Reg.isPhysical())
    {
      MachineBasicBlock::iterator ItDef = *DefI;
      MachineBasicBlock::iterator ItMi = *MI;

      // Debug output
      LLVM_DEBUG({
	  dbgs() << "SpOCl Def ";
	  ItDef->dump ();
	  dbgs() << "\n SpOCl Mi ";
	  ItMi->dump ();
	  dbgs() << "\n";
	});
      
      // If the instructions are already in the right sequence,
      // no splice is required
      if (std::next(ItDef) == ItMi)
	LLVM_DEBUG(dbgs() << "SpliceOrCloneInstruction: ### Instruction sequence already OK\n");
      else
	{
	  // Specifically only address the COPY $areg instruction!
	  if ((DefI->getOpcode () == T8xx::COPY) &&
	      (DefI->getOperand (1).isReg ()) &&
	      (DefI->getOperand (1).getReg () == T8xx::AREG))
	    {
	      // If the results of the copy is needed at some other place,
	      // the return value is stored in a temporary variable
	      LLVM_DEBUG(dbgs() << "SpliceOrCloneInstruction: ### Copy instruction\n");
	      DebugLoc DL = DefI->getDebugLoc();

	      Register RegClone = MRI.cloneVirtualRegister (Reg);
	      Use->setReg (RegClone);

	      // TODO: Just to see if this works. Might be rather inefficient to have this
	      // after each newly created virtual register
	      VRM.grow ();

	      // Store temporary variable after defining instruction
	      if (VRM.isAssignedReg (Reg))
		VRM.assignVirt2StackSlot (Reg);

	      MachineBasicBlock::iterator MBBI = *DefI;

	      BuildMI(*(MBBI->getParent()), ++MBBI, DL, TII->get(T8xx::STL)).addReg(Reg).
		addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);
		
	      // Create new virtual register for clone
	      DefI = BuildMI(*MBB, *MI, DL, TII->get(T8xx::LDL),RegClone).
		addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);
	    }

	  // Shift defining instruction in front of consuming instruction
	  MBB->splice (MI, DefI->getParent (), DefI);
	}
    }

  return (DefI);
}

const char *tcwg_tab5[] =
  {"CBA",    // C;B;A
   "CABr",   // C;A;B:rev
   "ACrBr",  // A;C;rev;B;rev
   "CBA",    // C;B;A
   "AsCBAl", // A;stl A; C; B; ldl a
   "AsCBAl", // A;stl A; C; B; ldl a
   "BCrA",
   "AsBCrAl",
   "AsBCrAl",
   "CBA",
   "CABr",
   "AsCBAl",
   "CBA",
   "AsCBAl",
   "AsCBAl",
   "BsCBlA",
   "BsCABlr",
   "AsBsCBlAl"};

MachineInstr *T8xxStackPass::reorderRecursive (MachineFunction &MF,
					       MachineInstr *MI,
					       MachineRegisterInfo &MRI,
					       LiveIntervals &LIS,
					       VirtRegMap &VRM,
					       std::vector<MachineInstr *> &output)
{
  MachineBasicBlock *MBB = MI->getParent ();
  const auto *TII = MF.getSubtarget<T8xxSubtarget>().getInstrInfo();

  // Debugging Write out all definitions and operators
  const iterator_range<MachineInstr::mop_iterator> &Range_uses = MI->explicit_uses();

  // Vector to hold the depth / operand register usage of the
  // preceding operations
  SmallVector<std::pair<int, MachineOperand *>, 4> OpDepth;
  SmallVector<std::pair<int, MachineOperand *>, 4> OpDepthFP;

  // Buffer to save registers that have been introduced during stackification
  Register reg_mem[5];

  // String to describe the required instruction sequence
  const char *str2code = NULL;

  // Find out how many registers are defined and how many are needed as input
  // When a variable has multiple definitions, put "-1" on the register stack
  for (auto I = Range_uses.begin (); I != Range_uses.end (); ++I)
    {
      const TargetRegisterClass *RC = NULL;
      if (I->isReg () && !I->getReg().isPhysical ())
	RC = MRI.getRegClassOrNull (I->getReg ());

      if (I->isReg () &&
	  !I->getReg().isPhysical() &&
	  ((RC->getID () == T8xx::ORegRegClassID) ||
	   (RC->getID () == T8xx::FPRegRegClassID) ||
	   (RC->getID () == T8xx::DFPRegRegClassID) ||
	   (RC->getID () == T8xx::LRegRegClassID)))
	{
	  Register Reg = I->getReg();
	  MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);
	  T8xxRegStack RegStack = mapRegisterStack (RC->getID());

	  if (DefI)
	    {
	      // When the register was already assigned to a temporary
	      // stack slot, the depth does not need to be analysed.
	      
	      // TODO: Verify for floating points regs
	      int SubE = 1;
	      if (VRM.isAssignedReg(Reg))
		SubE = getDepth (DefI, MRI, LIS, RegStack);

	      if (RegStack == TRS_Int)
		OpDepth.push_back (std::make_pair(SubE, I));
	      if (RegStack == TRS_Float)
		OpDepthFP.push_back (std::make_pair(SubE, I));
	    }
	  else
	    {
	      // Register with multiple definitions or those, where it is
	      // not possible to move the respective instruction get
	      // depth "10000" (arbitrary value)
	      if (RegStack == TRS_Int)
		// TODO: Verify this is the correct approach for LREG
		OpDepth.push_back (std::make_pair(10000, I));
	      if (RegStack == TRS_Float)
		OpDepthFP.push_back (std::make_pair(10000, I));
	    }
	}

      // TODO: Ignore phyiscal registers for the moment.
      // Those should only appear in COPY intstructions after
      // a function returns.
    }


  // Most common case, regular instruction without usage of FP registers
  if (OpDepthFP.size () == 0)
    {
      if (OpDepth.size () == 1)
	str2code = "A";

      if (OpDepth.size () == 2)
	{
	  LLVM_DEBUG(dbgs() << "Reorder Depth 2  " << OpDepth[0].first << "   " << OpDepth[1].first << "\n");
	  // Move instruction ahead of current instruction and then move on
	  // to definition

	  if (OpDepth[1].first > OpDepth[0].first)
	    {
	      if (OpDepth[0].first > 2)
		str2code = "BsABl";
	      else
		{
		  // TODO: Check for commuting operators
		  // For commuting operators this string can be used (i.e.
		  // result is same with AREG and BREG switched (like add,mul)
		  // str2code = "BA";

		  // For non commuting operators this string must be used
		  // It brings AREG and BREG into the required order
		  // (div, sub, stnl!)
		  str2code = "BAr";
		}
	    }
	  else
	    {
	      if (OpDepth[1].first < 3)
		str2code = "AB";
	      else
		str2code = "BsABl";
	    }
	}

      if (OpDepth.size () == 3)
	{
	  // Note: An Algorithm is written down in section 5.3.1 of the transputer compiler writer
	  // guide. However, it might be feasible to have an algorithm to determine the correct order.
	  // There may be one operand with >2 Depth, one operand with 2 Depth and one with 1 Depth.
	  // In this case, or when the Depths are lower, the usage of temporary variables
	  // is not needed!.
	  // Otherwise, up to two temporary variables are needed for the operands which
	  // have depth >2.
	  LLVM_DEBUG(dbgs() << "Reorder Depth 3  " << OpDepth[0].first << OpDepth[1].first << OpDepth[2].first << "\n");
	  int indx = 0,
	    indx_fac = 1;
	  for (int i = 0; i < 3; ++i)
	    {
	      // OpDepth[2-1] == 1 -> Add 0 to indx, i.e. do nothing
	      if (OpDepth[i].first == 2)
		indx += indx_fac;
	      if (OpDepth[i].first > 2)
		indx += indx_fac * 2;
	      indx_fac *= 3;
	    }
	  if (indx > 9) // No differentiation between case for C == 1 and C <= 2
	    indx -= 9;

	  if ((indx >= 0) && (indx < 18))
	    LLVM_DEBUG(dbgs() << "Operations " << tcwg_tab5[indx] << "\n");
	  else
	    LLVM_DEBUG(dbgs() << "3 Operand Index error!\n");

	  // Now transform the string into actual instructions
	  str2code = tcwg_tab5[indx];
	}
    }

  // One FP register is used. These cases could be mixed (i.e. one FP / one INT)
  if (OpDepthFP.size () == 1)
    {
      if (OpDepth.size () == 0)
	str2code = "D";
      else
	str2code = "DA";
    }

  // Two FP registers are used. These cases are "pure". I.e. only FP registers are used
  if (OpDepthFP.size () == 2)
    {
      if (OpDepthFP[1].first > OpDepthFP[0].first)
	{
	  if (OpDepthFP[0].first > 2)
	    str2code = "EsDEl";
	  else
	    // TODO: Check for commuting operators
	    str2code = "ED";
	}
      else
	{
	  if (OpDepthFP[1].first < 3)
	    str2code = "DE";
	  else
	    str2code = "EsDEl";
	}
    }

  if (str2code != NULL)
    {
      LLVM_DEBUG(dbgs() << "Str2Code " << str2code << "\n");

      while (*str2code != 0)
	{
	  // The "load instruction" must be preceded by the operand register
	  // Hence check for a load instruction at next position and fast forward
	  // to that position, if it is a load
	  if ((*(str2code+1) != 0) && (*(str2code+1) == 'l'))
	    ++str2code;

	  switch (*str2code)
	    {
	      // Integer cases
	    case 'A':
	    case 'B':
	    case 'C': {
	      // Note Character denotes operand position!
	      MachineOperand *Use = OpDepth[(*str2code) - 'A'].second;
	      Register Reg = Use->getReg ();
	      MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);

	      assert((DefI != nullptr) && "Integer, Instruction not found!!!");

	      DefI = SpliceOrCloneInstruction (MF, MBB, MRI, LIS, VRM, MI, Use);
	      reorderRecursive (MF, DefI, MRI, LIS, VRM, output);
	    }
	      break;

	      // Floating point cases
	    case 'D':
	    case 'E': {
	      // Note Character denotes operand position!
	      MachineOperand *Use = OpDepthFP[(*str2code) - 'D'].second;
	      Register Reg = Use->getReg ();
	      MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);

	      assert((DefI != nullptr) && "Floating Point, Instruction not found!!!");

	      DefI = SpliceOrCloneInstruction (MF, MBB, MRI, LIS, VRM, MI, Use);
	      reorderRecursive (MF, DefI, MRI, LIS, VRM, output);
	    }
	      break;

	      // Insert reversal of two top register stack positions
	      // Required for BA case with non commuting operator
	    case 'r': {
	      // Assert somehow that only two operands are available

	      // Note Character denotes operand position!
	      MachineOperand *Use1 = OpDepth[0].second;
	      Register Reg1 = Use1->getReg ();
	      MachineOperand *Use2 = OpDepth[1].second;
	      Register Reg2 = Use2->getReg ();

	      MachineBasicBlock::iterator MBBI = *MI;
	      DebugLoc DL = MI->getDebugLoc();

	      Register RegJoin;
	      RegJoin = MRI.createVirtualRegister (&T8xx::LRegRegClass);

	      VRM.grow ();

	      BuildMI(*MBB, MBBI, DL, TII->get(T8xx::JOIN),RegJoin)
		.addReg (Reg1)
		.addReg (Reg2);

	      BuildMI(*MBB, MBBI, DL, TII->get(T8xx::REV),Reg2)
		.addReg(RegJoin);
	    }
	      break;

	    case 's': {
	      bool bIntCase = (*(str2code-1) <= 'C');
	      int opno = bIntCase ? (*(str2code-1)) - 'A' :
		(*(str2code-1)) - 'D';

	      // Note Character denotes operand position!
	      MachineOperand *Use = OpDepth[opno].second;
	      Register Reg = Use->getReg ();
	      MachineInstr *DefI = getVRegDef(Reg, MI, MRI, LIS);

	      // Save register for later
	      reg_mem[(*(str2code-1)) - 'A'] = Reg;

	      Register RegClone = MRI.cloneVirtualRegister (Reg);
	      Use->setReg (RegClone);
	      VRM.grow ();

	      // Introduce temporary variable
	      // Simply introduce a workspace register
	      if (VRM.isAssignedReg (Reg))
		VRM.assignVirt2StackSlot (Reg);
	      DebugLoc DL = MI->getDebugLoc();

	      MachineBasicBlock::iterator MBBI = *DefI;

	      if (bIntCase)
		{
		  BuildMI(*MBB, ++MBBI, DL, TII->get(T8xx::STL)).addReg(Reg).
		    addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);
		}
	      else
		{
		  Register RegFPStack;
		  RegFPStack = MRI.createVirtualRegister (&T8xx::ORegRegClass);

		  VRM.grow ();

		  // TODO: Evaluate whether something needs to be done regarding the newly introduced
		  // integer variable?
		  BuildMI(*MBB, ++MBBI, DL, TII->get(T8xx::LDLP),RegFPStack).
		    addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);
		  DefI = BuildMI(*MBB, ++MBBI, DL, TII->get(T8xx::FPSTNLSN)).
		    addReg(Reg).addReg(RegFPStack);
		}
	    }
	      break;

	    case 'l': {
	      bool bIntCase = (*(str2code-1) <= 'C');
	      int opno = bIntCase ? (*(str2code-1)) - 'A' :
		(*(str2code-1)) - 'D';
	      MachineOperand *Use = OpDepth[opno].second;

	      // When the temporary register is introduced, the use
	      // is set to RegClone. Hence, we can retrieve the right clone from there
	      Register RegClone = Use->getReg ();

	      // Retrieve register that has been placed in temporary register
	      Register Reg = reg_mem[(*(str2code-1)) - 'A'];

	      // Load temporary variable before using instruction
	      DebugLoc DL = MI->getDebugLoc();
	      MachineBasicBlock::iterator MBBI = *MI;

	      if (bIntCase)
		{
		  BuildMI(*MBB, MBBI, DL, TII->get(T8xx::LDL),RegClone).
		    addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);
		}
	      else
		{
		  Register RegFPStack;
		  RegFPStack = MRI.createVirtualRegister (&T8xx::ORegRegClass);

		  // TODO: Evaluate whether something needs to be done regarding the newly introduced
		  // integer variable?
		  BuildMI(*MBB, ++MBBI, DL, TII->get(T8xx::LDLP),RegFPStack).
		    addFrameIndex(VRM.getStackSlot(Reg)).addImm(0);

		  // Load single when register is singe precision
		  if (MRI.getRegClassOrNull (Reg)->getID () == T8xx::FPRegRegClassID)
		    BuildMI(*MBB, ++MBBI, DL, TII->get(T8xx::FPLDNLSN),RegClone).
		      addReg(RegFPStack);

		  // Load double when register is double precision
		  if (MRI.getRegClassOrNull (Reg)->getID () == T8xx::DFPRegRegClassID)
		    BuildMI(*MBB, ++MBBI, DL, TII->get(T8xx::FPLDNLDB),RegClone).
		      addReg(RegFPStack);

		  VRM.grow ();
		}
	    }
	      break;

	    }

	  ++str2code;
	}
    }

  output.push_back (MI);
  return (MI);
}


enum StackAction
  {
    None,
    MoveInst,
    CloneInst,
    DefTemp
  };

typedef struct StackInfos_s
{
  int defs,
    nondbg_uses,
    oreg_depth,
    fpreg_depth;

  StackAction action;
  SmallVector<Register, 4> RegClones;
} StackInfos;



void insertTempStore (MachineBasicBlock::instr_iterator def,
		      MachineRegisterInfo &MRI,
		      VirtRegMap &VRM,
		      Register VirtOrig,
		      Register VirtNew)
{
  DebugLoc DL = def->getDebugLoc();
  MachineBasicBlock *MBB = def->getParent ();
  MachineFunction *MF = MBB->getParent ();
  MIMetadata MIMD = MIMetadata(*def);
  const auto *TII = MF->getSubtarget<T8xxSubtarget>().getInstrInfo();

  bool isFP = false;
  unsigned OpCode = 0;

  switch (MRI.getRegClassOrNull (VirtOrig)->getID ())
    {
    case T8xx::ORegRegClassID:
      OpCode = T8xx::STL;
      break;
    case T8xx::FPRegRegClassID:
      isFP = true;
      OpCode = T8xx::FPSTNLSN;
      break;
    case T8xx::DFPRegRegClassID:
      isFP = true;
      OpCode = T8xx::FPSTNLDB;
      break;
    default:
      llvm_unreachable("Unknow register class!");
    }

  Register RegFPStack;
  if (isFP)
    RegFPStack = MRI.createVirtualRegister (&T8xx::ORegRegClass);

  if (++def == MBB->end ())
    {
      if (isFP)
	{
	  BuildMI(MBB, MIMD, TII->get(T8xx::LDLP),RegFPStack).
	    addFrameIndex(VRM.getStackSlot(VirtOrig)).
	    addImm(0);

	  BuildMI(MBB, MIMD, TII->get(OpCode)).
	    addReg(VirtNew).
	    addReg(RegFPStack);
	}
      else
	{
	  BuildMI(MBB, MIMD, TII->get(OpCode)).
	    addReg(VirtNew).
	    addFrameIndex(VRM.getStackSlot(VirtOrig)).
	    addImm(0);
	}
    }
  else
    {
      if (isFP)
	{
	  BuildMI(*MBB, *def, DL, TII->get(T8xx::LDLP),RegFPStack).
	    addFrameIndex(VRM.getStackSlot(VirtOrig)).
	    addImm(0);

	  BuildMI(*MBB, *def, DL, TII->get(OpCode)).
	    addReg(VirtNew).
	    addReg(RegFPStack);
	}
      else
	{
	  BuildMI(*MBB, *def, DL, TII->get(T8xx::STL)).
	    addReg(VirtNew).
	    addFrameIndex(VRM.getStackSlot(VirtOrig)).
	    addImm(0);
	}
    }

}

void insertTempLoad (MachineBasicBlock::instr_iterator use,
		     MachineRegisterInfo &MRI,
		     VirtRegMap &VRM,
		     Register VirtOrig,
		     Register VirtNew)
{
  DebugLoc DL = use->getDebugLoc();
  MachineBasicBlock *MBB = use->getParent ();
  MachineFunction *MF = MBB->getParent ();
  const auto *TII = MF->getSubtarget<T8xxSubtarget>().getInstrInfo();

  switch (MRI.getRegClassOrNull (VirtOrig)->getID ())
    {
    case T8xx::ORegRegClassID:
      {
	BuildMI(*MBB, *use, DL, TII->get(T8xx::LDL), VirtNew).
	  addFrameIndex(VRM.getStackSlot(VirtOrig)).addImm(0);
      }
      break;

    case T8xx::FPRegRegClassID:
      {
	// For floating point numbers, an additional i32 register
	// is needed to address the stack
	Register RegFPStack;
	RegFPStack = MRI.createVirtualRegister (&T8xx::ORegRegClass);

	BuildMI(*MBB, *use, DL, TII->get(T8xx::LDLP),RegFPStack).
	  addFrameIndex(VRM.getStackSlot(VirtOrig)).addImm(0);

	BuildMI(*MBB, *use, DL, TII->get(T8xx::FPLDNLSN), VirtNew).
	  addReg(RegFPStack);
      }
      break;

    case T8xx::DFPRegRegClassID:
      {
	// For floating point numbers, an additional i32 register
	// is needed to address the stack
	Register RegFPStack;
	RegFPStack = MRI.createVirtualRegister (&T8xx::ORegRegClass);

	BuildMI(*MBB, *use, DL, TII->get(T8xx::LDLP),RegFPStack).
	  addFrameIndex(VRM.getStackSlot(VirtOrig)).addImm(0);

	BuildMI(*MBB, *use, DL, TII->get(T8xx::FPLDNLDB), VirtNew).
	  addReg(RegFPStack);
      }
      break;
    default:
      llvm_unreachable("Unknow register class\n");
    }
}


/// runOnMachineFunction - Loop over all of the basic blocks, transforming FP
/// register references into FP stack references.
///
bool T8xxStackPass::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "********** Register Stackifying **********\n"
                       "********** Function: "
                    << MF.getName() << '\n');

  MachineRegisterInfo &MRI = MF.getRegInfo();
  T8xxMachineFunctionInfo &MFI = *MF.getInfo<T8xxMachineFunctionInfo>();
  auto &LIS = getAnalysis<LiveIntervalsWrapperPass>().getLIS();

  // OKH: Try to use the virtual register map
  auto &VRM_Leg = getAnalysis<VirtRegMapWrapperLegacy>();
  VirtRegMap &VRM = VRM_Leg.getVRM ();

  LLVM_DEBUG({
      dbgs() << "############ Register Map\n";
      VRM.dump ();
      
      dbgs() << "############ LiveInterval Map\n";
      // LiveInterval dump
      LIS.dump ();
    });

  // Some map to keep track of registers that have already been created as
  // workspace registers
  std::map<unsigned, unsigned> wp_reg_map;

  // Walk the instructions from the bottom up. Currently we don't look past
  // block boundaries, and the blocks aren't ordered so the block visitation
  // order isn't significant, but we may want to change this in the future.
  std::map<Register, int> map_mult_def;

  // Test to see whether the algorithm can be structured differently.
  std::vector<StackInfos> stack_info (MRI.getNumVirtRegs ());

  for (unsigned int i = 0, e = MRI.getNumVirtRegs (); i != e; ++i)
    {
      unsigned VirtReg = Register::index2VirtReg (i);
      LLVM_DEBUG(dbgs() << "I " << i << "  Reg " << VirtReg << "\n");

      // Depth is initialized with -1 (for "not calculated yet")
      stack_info[i].oreg_depth = -1;
      stack_info[i].fpreg_depth = -1;
      stack_info[i].action = None;

      // Definitions of a register
      MachineRegisterInfo::def_instr_iterator def_iter = MRI.def_instr_begin(VirtReg);
      for (; def_iter != MRI.def_instr_end(); ++def_iter)
	{
	  LLVM_DEBUG({
	      def_iter->dump ();
	    });
	  stack_info[i].defs++;
	}

      // Uses of a register
      // Walk by operators, since a variable might be uses twice in the same instruction!
      MachineRegisterInfo::use_iterator use_iter = MRI.use_begin(VirtReg);
      for (; use_iter != MRI.use_end(); ++use_iter)
	{
	  LLVM_DEBUG({
	      use_iter->dump ();
	    });
	  stack_info[i].nondbg_uses++;
	}

      // More than one def -> Introduce temporary variable on stack
      // One def ->
      //    More than one use -> Simple instruction -> clone
      //                      -> Complex instruction -> temp variable on stack
      //    One use -> Move instruction
      if (stack_info[i].defs > 1)
	stack_info[i].action = DefTemp;
      else
	{
	  if (stack_info[i].defs > 0)
	    {
	      if (stack_info[i].nondbg_uses > 1)
		{
		  def_iter = MRI.def_instr_begin (VirtReg);
		  switch (def_iter->getOpcode ())
		    {
		    case T8xx::XDBLE :
		    case T8xx::LDC :
		    case T8xx::LDLP :
		      stack_info[i].action = CloneInst;
		      break;
		    default:
		      stack_info[i].action = DefTemp;
		    }
		}
	      else
		stack_info[i].action = MoveInst;
	    }
	}
    }

  // Do the floating point and integer operations simultaneously
  unsigned int origNumVirtRegs = MRI.getNumVirtRegs ();
  for (unsigned int i = 0, e = origNumVirtRegs; i != e; ++i)
    {
      unsigned VirtReg = Register::index2VirtReg (i);
      std::vector<std::pair<MachineOperand *, Register>> vreg_replace;

      // Introduce a temporary variable on the stack (the Transputer equivalent of a register)
      if (stack_info[i].action == DefTemp)
	{
	  // Allocate stack slot for virtual register
	  if (VRM.isAssignedReg (VirtReg))
	    VRM.assignVirt2StackSlot (VirtReg);

	  // Uses of a register are filled with appropriate load instructions first
	  MachineRegisterInfo::use_instr_nodbg_iterator use_iter = MRI.use_instr_nodbg_begin(VirtReg);

	  for (; use_iter != MRI.use_instr_nodbg_end(); ++use_iter)
	    {
	      // Replace register in using instruction with newly created virtual register
	      const iterator_range<MachineInstr::mop_iterator> &Range_uses = use_iter->uses();
	      MachineBasicBlock::instr_iterator MBBI(*use_iter);

	      // A using instruction may use a virtual reg multiple times.
	      // Create a load instruction and new virtual register for each use
	      for (MachineOperand *op_use = Range_uses.begin(); op_use != Range_uses.end (); ++op_use)
		{
		  if (op_use->isReg() && (op_use->getReg() == VirtReg))
		    {
		      // Create new virtual register and replace in the using instruction
		      Register VirtNew = MRI.createVirtualRegister (MRI.getRegClassOrNull (VirtReg));
		      vreg_replace.push_back (std::make_pair (op_use, VirtNew));
		      insertTempLoad (MBBI, MRI, VRM, VirtReg, VirtNew);
		    }
		}
	    }

	  // Definitions of a register
	  MachineRegisterInfo::def_instr_iterator def_iter = MRI.def_instr_begin(VirtReg);
	  for (; def_iter != MRI.def_instr_end(); ++def_iter)
	    {
	      // The first definition reuses the virtual register
	      // For the second definition introduce a new virtual register
	      Register VirtNew;
	      if (def_iter == MRI.def_instr_begin(VirtReg))
		VirtNew = VirtReg;
	      else
		{
		  VirtNew = MRI.createVirtualRegister (MRI.getRegClassOrNull (VirtReg));

		  // Replace register in using instruction with newly created virtual register
		  const iterator_range<MachineInstr::mop_iterator> &Range_defs = def_iter->defs();
		  for (MachineOperand *op_def = Range_defs.begin(); op_def != Range_defs.end (); ++op_def)
		    {
		      if (op_def->isReg() && (op_def->getReg() == VirtReg))
			vreg_replace.push_back (std::make_pair (op_def, VirtNew));
		    }
		}

	      MachineBasicBlock::instr_iterator MBBI(*def_iter);
	      insertTempStore (MBBI, MRI, VRM, VirtReg, VirtNew);
	    }
	}

      // Clone simple instruction
      if (stack_info[i].action == CloneInst)
	{
	  // Find definition of virtual register
	  MachineRegisterInfo::def_instr_iterator def_iter = MRI.def_instr_begin(VirtReg);

	  // A cloned definition is copied in front of the using instructions
	  MachineRegisterInfo::use_instr_nodbg_iterator use_iter = MRI.use_instr_nodbg_begin(VirtReg);
	  ++use_iter;  // The first use gets the original virtual register

	  for (; use_iter != MRI.use_instr_nodbg_end(); ++use_iter)
	    {
	      MachineBasicBlock *MBB = use_iter->getParent ();
	      MachineBasicBlock::instr_iterator MBBI_use(*use_iter);
	      MachineBasicBlock::instr_iterator MBBI_def(*def_iter);

	      // Clone instruction
	      MachineInstr &MI_Clone = MF.cloneMachineInstrBundle(*MBB, MBBI_use, *MBBI_def);

	      // Create new virtual register and replace in cloned definition
	      const iterator_range<MachineInstr::mop_iterator> &Range_defs = MI_Clone.defs();
	      Register VirtNew = MRI.createVirtualRegister (MRI.getRegClassOrNull (VirtReg));
	      Range_defs.begin()->setReg(VirtNew);

	      // Replace register in using instruction with newly created virtual register
	      const iterator_range<MachineInstr::mop_iterator> &Range_uses = use_iter->uses();
	      for (MachineOperand *op_use = Range_uses.begin(); op_use != Range_uses.end (); ++op_use)
		{
		  if (op_use->isReg() && (op_use->getReg() == VirtReg))
		    vreg_replace.push_back (std::make_pair (op_use, VirtNew));
		}
	    }
	}

      // Run through operands and replace registers with newly created virtual registers
      for (auto vi = vreg_replace.begin (); vi != vreg_replace.end (); ++vi)
	vi->first->setReg (vi->second);
    }

  VRM.grow ();

  // Note: After the floating point substitution every virtual register should be used at maximum one time
  // If a floating point register is loaded from a stack position, a new virtual register has been defined.

  LLVM_DEBUG({
      dbgs() << "After Reg Substitution\n";
      for (MachineBasicBlock &MBB : MF) {
	MBB.dump ();
      }
      dbgs() << "##########################\n";
    });

  // For debugging, count uses and defs again.
  std::vector<StackInfos> stack_info_new (MRI.getNumVirtRegs ());
  for (unsigned int i = 0, e = MRI.getNumVirtRegs (); i != e; ++i)
    {
      unsigned VirtReg = Register::index2VirtReg (i);
      LLVM_DEBUG(dbgs() << "I " << i << "  Reg " << VirtReg << "\n");

      // Depth is initialized with -1 (for "not calculated yet")
      stack_info_new[i].oreg_depth = -1;
      stack_info_new[i].fpreg_depth = -1;
      stack_info_new[i].action = None;

      LLVM_DEBUG(dbgs() << "Uses\n");
      // Uses of a register
      MachineRegisterInfo::use_instr_nodbg_iterator use_iter = MRI.use_instr_nodbg_begin(VirtReg);
      for (; use_iter != MRI.use_instr_nodbg_end(); ++use_iter)
	{
	  LLVM_DEBUG({
	      use_iter->dump ();
	    });
	  stack_info_new[i].nondbg_uses++;
	}

      // Definitions of a register
      if (stack_info_new[i].nondbg_uses == 0)
	{
	  // If a virtual register definition has no uses, remove it
	  MachineRegisterInfo::def_instr_iterator def_iter = MRI.def_instr_begin(VirtReg);
	  if (def_iter != MRI.def_instr_end())
	    {
	      MachineBasicBlock::instr_iterator MBBI_def(*def_iter);
	      MachineBasicBlock *MBB = def_iter->getParent ();
	      MBB->erase (MBBI_def);
	    }
	}
      else
	{
	  MachineRegisterInfo::def_instr_iterator def_iter = MRI.def_instr_begin(VirtReg);
	  for (; def_iter != MRI.def_instr_end(); ++def_iter)
	    {
	      LLVM_DEBUG({
		  def_iter->dump ();
		});
	      stack_info_new[i].defs++;
	    }
	}
    }

  // Debug info
  LLVM_DEBUG({
      for (unsigned int i = 0, e = MRI.getNumVirtRegs(); i != e; ++i)
	{
	  dbgs() << "I " << i <<
	    "  Defs " << stack_info_new[i].defs <<
	    "  Uses " << stack_info_new[i].nondbg_uses <<
	    "  OReg Depth " << stack_info_new[i].oreg_depth <<
	    "  FPReg Depth " << stack_info_new[i].fpreg_depth << "\n";
	}
    });

  
  // Collect instructions that only "consume" virtual registers.
  // Those instructions are the anchor points from which a recursive
  // rearrangement of the definitions is carried out.
  for (MachineBasicBlock &MBB : MF) {

    // Don't use a range-based for loop, because we modify the list as we're
    // iterating over it and the end iterator may change.
    std::vector<MachineInstr *> outvec;

    // Vector to keep a list of all "initial instructions" that
    // need "stackification".
    std::vector<MachineInstr *> proc_instr,
      proc_fp_instr;

    // Now do the recursive repositioning of the instructions to
    // have the proper stack ordering before the actual instructions
    // are executed
    for (auto MII = MBB.rbegin(); MII != MBB.rend(); ++MII)
      {
	MachineInstr *Insert = &*MII;

	// Don't nest anything inside an inline asm, because we don't have
	// constraints for $push inputs.
	if (Insert->isInlineAsm())
	  continue;

	// Ignore debugging intrinsics.
	if (Insert->isDebugValue())
	  continue;

	const iterator_range<MachineInstr::mop_iterator> &Range_defs = Insert->defs();
	const iterator_range<MachineInstr::mop_iterator> &Range_uses = Insert->explicit_uses();

	// When the instruction does not define anything, it is a store
	// instruction and should be recursed
	if (Range_defs.begin () == Range_defs.end ())
	  {
	    unsigned int reg_u_fp = 0,
	      reg_u_i = 0;

	    for (auto I = Range_uses.begin (); I != Range_uses.end (); ++I)
	      {
		const TargetRegisterClass *RC = NULL;
		if (I->isReg () && !I->getReg().isPhysical ())
		  RC = MRI.getRegClassOrNull (I->getReg ());

		if (I->isReg () &&
		    !I->getReg().isPhysical () &&
		    (RC->getID () == T8xx::ORegRegClassID))
		  ++reg_u_i;

		if (I->isReg () &&
		    !I->getReg().isPhysical () &&
		    (RC->getID () == T8xx::FPRegRegClassID))
		  ++reg_u_fp;

		if (I->isReg () &&
		    !I->getReg().isPhysical () &&
		    (RC->getID () == T8xx::DFPRegRegClassID))
		  ++reg_u_fp;
	      }

	    // Reorder instructions (according to Transputer compiler writing guide)
	    if (reg_u_i > 0)
	      proc_instr.push_back (Insert);
	    if (reg_u_fp > 0)
	      proc_fp_instr.push_back (Insert);
	  }
	else
	  {
	    // Special treatment for the "COPY" instruction before RET
	    MachineOperand *Def = Range_defs.begin();
	    Register Reg = Def->getReg();
	    if (Reg == T8xx::AREG)
	      proc_instr.push_back (Insert);

	    // Deal with floating point instructions separately as they
	    // have a separate stack.
	    // TODO: Think about mixed instructions, which consume FP and produce INT
	    if (Reg == T8xx::FAREG)
	      proc_fp_instr.push_back (Insert);
	  }
      }  // MachineInstruction


    // Now reorder instructions (floating point first)
    for (auto PI = proc_fp_instr.begin (); PI != proc_fp_instr.end (); ++PI)
      {
	LLVM_DEBUG({
	    MBB.dump ();
	    dbgs() << "Reorder FP\n";
	    (*PI)->dump ();
	  });
	reorderRecursive (MF, *PI, MRI, LIS, VRM, outvec);
      }

    LLVM_DEBUG({
	MBB.dump ();
      });

    // Now reorder instructions
    for (auto PI = proc_instr.begin (); PI != proc_instr.end (); ++PI)
      {
	LLVM_DEBUG({
	    MBB.dump ();
	    dbgs () << "Reorder INT\n";
	    (*PI)->dump ();
	  });
	reorderRecursive (MF, *PI, MRI, LIS, VRM, outvec);
      }

    LLVM_DEBUG({
	MBB.dump ();
      });

    LLVM_DEBUG(dbgs ()<<"Print sequence\n");
    std::map<Register, int> vreg_map;
    for (auto O = outvec.begin (); O != outvec.end (); ++O)
      {
	LLVM_DEBUG({
	    (*O)->dump ();
	  });

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
    LLVM_DEBUG(dbgs() << "End Print sequence\n\n");

    // Debug output about register usage
    LLVM_DEBUG({
	dbgs() << "Def usage\n";
	for (auto I = vreg_map.begin (); I != vreg_map.end (); ++I)
	  {
	    dbgs() << "ID " << I->first.id() <<
	      "  Count " << I->second << "  # ";
	    if (MRI.hasOneNonDBGUse(I->first))
	      dbgs() << "One Non DBG Use\n";
	    else
	      dbgs() << "Multiple Non DBG Use\n";
	  }
	dbgs() << "End Def usage\n";
      });
  }

  // ######### Replace virtual registers with the stack registers

  for (MachineBasicBlock &MBB : MF) {

  for (auto MII = MBB.begin(); MII != MBB.end(); ++MII) {
      MachineInstr *Instr = &*MII;

      // Don't nest anything inside an inline asm, because we don't have
      // constraints for $push inputs.
      if (Instr->isInlineAsm())
        continue;

      // Ignore debugging intrinsics.
      /*
      if (Instr->isDebugValue())
        continue;
      */

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

	      if (Reg.isVirtual())
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
    
    LLVM_DEBUG(dbgs() << "############ Register Map\n");

  //  return Changed;
  return false;
}
