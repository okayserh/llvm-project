//===-- X86FloatingPoint.cpp - Floating point Reg -> Stack converter ------===//
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
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/EdgeBundles.h"
#include "llvm/CodeGen/LivePhysRegs.h"
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

const unsigned ScratchFPReg = 7;

namespace llvm {

struct T8xxStackPass : public MachineFunctionPass {
  static char ID;
  T8xxStackPass() : MachineFunctionPass(ID) {
      // This is really only to keep valgrind quiet.
      // The logic in isLive() is too much for it.
      /*
      memset(Stack, 0, sizeof(Stack));
      memset(RegMap, 0, sizeof(RegMap));
      */
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

    StringRef getPassName() const override { return "T8xx INT Stackifier"; }

  private:
    const TargetInstrInfo *TII = nullptr; // Machine instruction info.

    // Two CFG edges are related if they leave the same block, or enter the same
    // block. The transitive closure of an edge under this relation is a
    // LiveBundle. It represents a set of CFG edges where the live FP stack
    // registers must be allocated identically in the x87 stack.
    //
    // A LiveBundle is usually all the edges leaving a block, or all the edges
    // entering a block, but it can contain more edges if critical edges are
    // present.
    //
    // The set of live FP registers in a LiveBundle is calculated by bundleCFG,
    // but the exact mapping of FP registers to stack slots is fixed later.
    struct LiveBundle {
      // Bit mask of live FP registers. Bit 0 = FP0, bit 1 = FP1, &c.
      unsigned Mask = 0;

      // Number of pre-assigned live registers in FixStack. This is 0 when the
      // stack order has not yet been fixed.
      unsigned FixCount = 0;

      // Assigned stack order for live-in registers.
      // FixStack[i] == getStackEntry(i) for all i < FixCount.
      unsigned char FixStack[8];

      LiveBundle() = default;

      // Have the live registers been assigned a stack order yet?
      bool isFixed() const { return !Mask || FixCount; }
    };

    // Numbered LiveBundle structs. LiveBundles[0] is used for all CFG edges
    // with no live FP registers.
    SmallVector<LiveBundle, 8> LiveBundles;

    // The edge bundle analysis provides indices into the LiveBundles vector.
    EdgeBundles *Bundles = nullptr;

    // Return a bitmask of FP registers in block's live-in list.
    static unsigned calcLiveInMask(MachineBasicBlock *MBB, bool RemoveFPs) {
      unsigned Mask = 0;
      for (MachineBasicBlock::livein_iterator I = MBB->livein_begin();
           I != MBB->livein_end(); ) {
        MCPhysReg Reg = I->PhysReg;
        static_assert(T8xx::R15 - T8xx::R0 == 15, "sequential regnums");
        if (Reg >= T8xx::R1 && Reg <= T8xx::R15) {
          Mask |= 1 << (Reg - T8xx::R1);
          if (RemoveFPs) {
            I = MBB->removeLiveIn(I);
            continue;
          }
        }
        ++I;
      }
      return Mask;
    }

    // Partition all the CFG edges into LiveBundles.
    void bundleCFGRecomputeKillFlags(MachineFunction &MF);

    MachineBasicBlock *MBB = nullptr;     // Current basic block

    // The hardware keeps track of how many FP registers are live, so we have
    // to model that exactly. Usually, each live register corresponds to an
    // FP<n> register, but when dealing with calls, returns, and inline
    // assembly, it is sometimes necessary to have live scratch registers.
    unsigned Stack[30];          // FP<n> Registers in each stack slot...
    unsigned StackTop = 0;      // The current top of the FP stack.

    enum {
      NumFPRegs = 15             // Including scratch pseudo-registers.
    };

    // For each live FP<n> register, point to its Stack[] entry.
    // The first entries correspond to FP0-FP6, the rest are scratch registers
    // used when we need slightly different live registers than what the
    // register allocator thinks.
    unsigned RegMap[NumFPRegs];

    // Set up our stack model to match the incoming registers to MBB.
    void setupBlockStack();

    // Shuffle live registers to match the expectations of successor blocks.
    void finishBlockStack();

  //#if !defined(NDEBUG) || defined(LLVM_ENABLE_DUMP)
    void dumpStack() const {
      dbgs() << "Stack contents:";
      for (unsigned i = 0; i != StackTop; ++i) {
        dbgs() << " FP" << Stack[i];
	//        assert(RegMap[Stack[i]] == i && "Stack[] doesn't match RegMap[]!");
      }
      dbgs() << "\n";
    }
  //#endif

    /// getSlot - Return the stack slot number a particular register number is
    /// in.
    unsigned getSlot(unsigned RegNo) const {
      assert(RegNo < NumFPRegs && "Regno out of range!");
      return RegMap[RegNo];
    }

    /// isLive - Is RegNo currently live in the stack?
    bool isLive(unsigned RegNo) const {
      unsigned Slot = getSlot(RegNo);
      return Slot < StackTop && Stack[Slot] == RegNo;
    }

    /// getStackEntry - Return the X86::FP<n> register in register ST(i).
    unsigned getStackEntry(unsigned STi) const {
      if (STi >= StackTop)
        report_fatal_error("Access past stack top!");
      return Stack[StackTop-1-STi];
    }

    /// getSTReg - Return the X86::ST(i) register which contains the specified
    /// FP<RegNo> register.
    unsigned getSTReg(unsigned RegNo) const {
      return StackTop - 1 - getSlot(RegNo) + T8xx::STA;
    }

    // pushReg - Push the specified FP<n> register onto the stack.
    void pushReg(unsigned Reg) {
      //assert(Reg < NumFPRegs && "Register number out of range!");
      /*
      if (StackTop >= 3)
        report_fatal_error("Stack overflow!");
      */
      if (StackTop >= 3)
	printf ("Stack overflow!\n");

      Stack[StackTop] = Reg;
      //      RegMap[Reg] = StackTop++;
      StackTop++;
    }

    // popReg - Pop a register from the stack.
    void popReg() {
      /*
      if (StackTop == 0)
        report_fatal_error("Cannot pop empty stack!");
      RegMap[Stack[--StackTop]] = ~0;     // Update state
      */
      if (StackTop > 0)
	--StackTop;
	//	RegMap[Stack[--StackTop]] = ~0;     // Update state
      else
	printf ("Cannot pop empty stack\n");
    }

    bool isAtTop(unsigned RegNo) const { return getSlot(RegNo) == StackTop-1; }
    void moveToTop(unsigned RegNo, MachineBasicBlock::iterator I) {
      DebugLoc dl = I == MBB->end() ? DebugLoc() : I->getDebugLoc();
      if (isAtTop(RegNo)) return;

      unsigned STReg = getSTReg(RegNo);
      unsigned RegOnTop = getStackEntry(0);

      // Swap the slots the regs are in.
      std::swap(RegMap[RegNo], RegMap[RegOnTop]);

      // Swap stack slot contents.
      if (RegMap[RegOnTop] >= StackTop)
        report_fatal_error("Access past stack top!");
      std::swap(Stack[RegMap[RegOnTop]], Stack[StackTop-1]);

      // Emit an fxch to update the runtime processors version of the state.
      // TODO: Might be REV
      //      BuildMI(*MBB, I, dl, TII->get(X86::XCH_F)).addReg(STReg);
      ++NumFXCH;
    }

    void duplicateToTop(unsigned RegNo, unsigned AsReg,
                        MachineBasicBlock::iterator I) {
      DebugLoc dl = I == MBB->end() ? DebugLoc() : I->getDebugLoc();
      unsigned STReg = getSTReg(RegNo);
      pushReg(AsReg);   // New register on top of stack

      //      BuildMI(*MBB, I, dl, TII->get(X86::LD_Frr)).addReg(STReg);
    }

    /// popStackAfter - Pop the current value off of the top of the FP stack
    /// after the specified instruction.
    void popStackAfter(MachineBasicBlock::iterator &I);

    /// freeStackSlotAfter - Free the specified register from the register
    /// stack, so that it is no longer in a register.  If the register is
    /// currently at the top of the stack, we just pop the current instruction,
    /// otherwise we store the current top-of-stack into the specified slot,
    /// then pop the top of stack.
    void freeStackSlotAfter(MachineBasicBlock::iterator &I, unsigned Reg);

    /// freeStackSlotBefore - Just the pop, no folding. Return the inserted
    /// instruction.
    MachineBasicBlock::iterator
    freeStackSlotBefore(MachineBasicBlock::iterator I, unsigned FPRegNo);

    /// Adjust the live registers to be the set in Mask.
    void adjustLiveRegs(unsigned Mask, MachineBasicBlock::iterator I);

    /// Shuffle the top FixCount stack entries such that FP reg FixStack[0] is
    /// st(0), FP reg FixStack[1] is st(1) etc.
    void shuffleStackTop(const unsigned char *FixStack, unsigned FixCount,
                         MachineBasicBlock::iterator I);

    bool processBasicBlock(MachineFunction &MF, MachineBasicBlock &MBB);

  void handleCall(MachineBasicBlock::iterator &I);
  void handleReturn(MachineBasicBlock::iterator &I);
  void handleZeroArgFP(MachineBasicBlock::iterator &I);
  void handleOneArgFP(MachineBasicBlock::iterator &I);
  void handleOneArgFPRW(MachineBasicBlock::iterator &I);
  void handleTwoArgFP(MachineBasicBlock::iterator &I);
  void handleCompareFP(MachineBasicBlock::iterator &I);
  void handleCondMovFP(MachineBasicBlock::iterator &I);
  void handleSpecialFP(MachineBasicBlock::iterator &I);

  // Check if a COPY instruction is using FP registers.
  static bool isFPCopy(MachineInstr &MI) {
    /*
      Register DstReg = MI.getOperand(0).getReg();
      Register SrcReg = MI.getOperand(1).getReg();
      
      return X86::RFP80RegClass.contains(DstReg) ||
      X86::RFP80RegClass.contains(SrcReg);
    */
    return false; // TODO
  }

  void setKillFlags(MachineBasicBlock &MBB) const;
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

/// getFPReg - Return the X86::FPx register number for the specified operand.
/// For example, this returns 3 for X86::FP3.
static unsigned getFPReg(const MachineOperand &MO) {
  //  assert(MO.isReg() && "Expected an FP register!");
  Register Reg = MO.getReg();
  /*
  assert(Reg >= T8xx::AREG && Reg <= T8xx::CREG && "Expected FP register!");
  return Reg - T8xx::AREG;
  */
  return Reg;
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

  // MRI doesn't know what the Def is. Try asking LIS.
  if (const VNInfo *ValNo = LIS.getInterval(Reg).getVNInfoBefore(
          LIS.getInstructionIndex(*Insert)))
    return LIS.getInstructionFromIndex(ValNo->def);

  return nullptr;
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
                                      MachineRegisterInfo &MRI) {
  LLVM_DEBUG(dbgs() << "Move for single use: "; Def->dump());

  //  WebAssemblyDebugValueManager DefDIs(Def);
  //  DefDIs.sink(Insert);
  LIS.handleMove(*Def);

  if (MRI.hasOneDef(Reg) && MRI.hasOneNonDBGUse(Reg)) {
    // No one else is using this register for anything so we can just stackify
    // it in place.
    //    MFI.stackifyVReg(MRI, Reg);
    printf ("Reg Stackified %u\n", Reg);
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
                     /*RemoveDeadValNo=*/true);

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
    SlotIndex Idx = LIS.getInstructionIndex(Def).getRegSlot();
    // ??
    //    LIS.removePhysRegDefAt(MCRegister::from(WebAssembly::ARGUMENTS), Idx);
    LIS.removeInterval(Reg);
    LIS.RemoveMachineInstrFromMaps(Def);
    // DefDIs.removeDef();
  }

  return Clone;
}


/// runOnMachineFunction - Loop over all of the basic blocks, transforming FP
/// register references into FP stack references.
///
bool T8xxStackPass::runOnMachineFunction(MachineFunction &MF) {

  printf ("runOnMachineFUnction\n");

  LLVM_DEBUG(dbgs() << "********** Register Stackifying **********\n"
                       "********** Function: "
                    << MF.getName() << '\n');

  bool Changed = false;
  MachineRegisterInfo &MRI = MF.getRegInfo();
  T8xxMachineFunctionInfo &MFI = *MF.getInfo<T8xxMachineFunctionInfo>();
  const auto *TII = MF.getSubtarget<T8xxSubtarget>().getInstrInfo();
  const auto *TRI = MF.getSubtarget<T8xxSubtarget>().getRegisterInfo();
  auto &MDT = getAnalysis<MachineDominatorTree>();
  auto &LIS = getAnalysis<LiveIntervals>();

  // LiveInterval dump
  LIS.dump ();
  

  // Walk the instructions from the bottom up. Currently we don't look past
  // block boundaries, and the blocks aren't ordered so the block visitation
  // order isn't significant, but we may want to change this in the future.
  for (MachineBasicBlock &MBB : MF) {
    // Don't use a range-based for loop, because we modify the list as we're
    // iterating over it and the end iterator may change.
    for (auto MII = MBB.rbegin(); MII != MBB.rend(); ++MII) {
    //    for (auto MII = MBB.begin(); MII != MBB.end(); ++MII) {
      MachineInstr *Insert = &*MII;
      MachineInstr &MI = *MII;
      // Don't nest anything inside an inline asm, because we don't have
      // constraints for $push inputs.
      if (Insert->isInlineAsm())
        continue;

      // Ignore debugging intrinsics.
      if (Insert->isDebugValue())
        continue;
      
      // Iterate through the inputs in reverse order, since we'll be pulling
      // operands off the stack in LIFO order.
      //      CommutingState Commuting;

#if 0
      // ###################################################
      // Idea for an algorithm:
      // Put value on stack and continue with subsequent
      // instructions until:
      // - Stack space (3 operand registers) is exceeded
      // - virtual register needs to be accessed and is
      //   not in a suitable position on the stack.
      // - Virtual register is read more than once
      // If either of the above cases happens, spill the register.

      MI.dump ();

      // Print register live ranges
      //unsigned int vrn = MRI.getNumVirtRegs ();
      printf ("Op dump begin \n");
      //      const iterator_range<MachineInstr::mop_iterator> &Range = Insert->explicit_uses();
      const iterator_range<MachineInstr::mop_iterator> &Range = Insert->defs();
      MachineInstr::mop_iterator miter;
      for (miter = Range.begin (); miter != Range.end (); ++miter)
	{
	  miter->dump ();
	  MachineOperand &Use = *miter;
	  if (!Use.isReg())
	    continue;
	  Register Reg = Use.getReg();
	  if (MRI.hasOneUse (Reg))
	    {
	      LIS.getInterval(Reg).dump ();
	      MachineInstrBundleIterator<llvm::MachineInstr> inst_iter = MII;
	      inst_iter++;
	      inst_iter->dump ();

	      SmallVector<MachineInstr::mop_iterator, 4> Worklist;
	      Worklist.push_back (miter);

	      

	      printf ("One use!\n");
	    }
	}
      printf ("Op dump end \n");

      /*
	   MachineOperand &Use = TreeWalker.pop();
      if (!Use.isReg())
	continue;

      Register Reg = Use.getReg();
      
      // Only registers with a single use qualify for elimination by clever usage of
      // stack registers
      if (MRI.hasOneUse (RegNo))
	{
	}
       
      */

      dumpStack ();
      
      // These push one element to the stack
      if ((MI.getOpcode() == T8xx::LDRi16wpop) ||
	  (MI.getOpcode() == T8xx::LDRi32wpop) ||
	  (MI.getOpcode() == T8xx::LEA_ADDri) ||
	  (MI.getOpcode() == T8xx::MOVimmr))
	{
	  unsigned DestReg = getFPReg(MI.getOperand(0));
	  //	  MI.getOperand(0).setReg (T8xx::STA);
	  pushReg(DestReg);
	}

      // Modify the top element of the stack
      if (MI.getOpcode() == T8xx::ADDimmr)
	{
	  unsigned DstReg = getFPReg(MI.getOperand(0));
	  unsigned SrcReg = getFPReg(MI.getOperand(1));
	  popReg();
	  pushReg(DstReg);
	}
      
      // These pop one elements from the stack
      if (MI.getOpcode() == T8xx::STRi32regop)
	{
	  popReg();	  
	}

      // Pops two elements from the stack
      if (MI.getOpcode() == T8xx::STRi8regop)
	{
	  popReg();	  
	  popReg();	  
	}

	  // Pops two elements from the stack and pushes one result to the stack
	  if ((MI.getOpcode() == T8xx::SHLregregop) ||
	      (MI.getOpcode() == T8xx::ORregregop) ||
	      (MI.getOpcode() == T8xx::ADDraw))
	{
	  unsigned DstReg = getFPReg(MI.getOperand(0));
	  popReg();	  
	  popReg();
	  pushReg (DstReg);
	}

      printf ("After Stackify\n");
      dumpStack ();
#endif


      //  Insert iterator to the used operands (may be registers)
      TreeWalkerState TreeWalker(Insert);
      while (!TreeWalker.done()) {
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

	/*
        // Argument instructions represent live-in registers and not real
        // instructions.
        if (WebAssembly::isArgument(DefI->getOpcode()))
          continue;
	*/

        MachineOperand *Def = DefI->findRegisterDefOperand(Reg);
        assert(Def != nullptr);

	printf ("Operand\n");
	Def->dump ();
	/* OKH: DefI is the instruction that defines the register, Def is the defined operand */

	
	// Decide which strategy to take. Prefer to move a single-use value
        // over cloning it, and prefer cloning over introducing a tee.
        // For moving, we require the def to be in the same block as the use;
        // this makes things simpler (LiveIntervals' handleMove function only
        // supports intra-block moves) and it's MachineSink's job to catch all
        // the sinking opportunities anyway.
        bool SameBlock = DefI->getParent() == &MBB;
        bool CanMove = SameBlock && isSafeToMove(Def, &Use, Insert, MFI, MRI) &&
                       !TreeWalker.isOnStack(Reg);

	printf ("Move for single use SameBlock %i   CanMove %i\n", SameBlock, CanMove);

	if (CanMove && hasOneNonDBGUse(Reg, DefI, MRI, MDT, LIS)) {
          Insert = moveForSingleUse(Reg, Use, DefI, MBB, Insert, LIS, MFI, MRI);

          // If we are removing the frame base reg completely, remove the debug
          // info as well.
          // TODO: Encode this properly as a stackified value.
	  /*
          if (MFI.isFrameBaseVirtual() && MFI.getFrameBaseVreg() == Reg)
            MFI.clearFrameBaseVreg();
	  */
        }
	/*
	else
	  {
	    printf ("MultiUse\n");
	    // TODO:
	    Insert = DefI;
	  }
	*/	

	else if (shouldRematerialize(*DefI, TII)) {
	  printf ("Rematerialize\n");
          Insert =
              rematerializeCheapDef(Reg, Use, *DefI, MBB, Insert->getIterator(),
                                    LIS, MFI, MRI, TII, TRI);
        } else if (CanMove && oneUseDominatesOtherUses(Reg, Use, MBB, MRI, MDT,
                                                       LIS, MFI)) {
	  printf ("MoveAndTeeForMultiUse\n");
	  /*          Insert = moveAndTeeForMultiUse(Reg, Use, DefI, MBB, Insert, LIS, MFI,
		      MRI, TII);*/
	    Insert = DefI;
        } else {
          // We failed to stackify the operand. If the problem was ordering
          // constraints, Commuting may be able to help.
	  /*
          if (!CanMove && SameBlock)
            Commuting.maybeCommute(Insert, TreeWalker, TII);
	  */
          // Proceed to the next operand.
          continue;
        }
	
	/*
        // Stackifying a multivalue def may unlock in-place stackification of
        // subsequent defs. TODO: Handle the case where the consecutive uses are
        // not all in the same instruction.
        auto *SubsequentDef = Insert->defs().begin();
        auto *SubsequentUse = &Use;
        while (SubsequentDef != Insert->defs().end() &&
               SubsequentUse != Use.getParent()->uses().end()) {
          if (!SubsequentDef->isReg() || !SubsequentUse->isReg())
            break;
          Register DefReg = SubsequentDef->getReg();
          Register UseReg = SubsequentUse->getReg();
          // TODO: This single-use restriction could be relaxed by using tees
          if (DefReg != UseReg || !MRI.hasOneNonDBGUse(DefReg))
            break;
          MFI.stackifyVReg(MRI, DefReg);
          ++SubsequentDef;
          ++SubsequentUse;
        }

        // If the instruction we just stackified is an IMPLICIT_DEF, convert it
        // to a constant 0 so that the def is explicit, and the push/pop
        // correspondence is maintained.
        if (Insert->getOpcode() == TargetOpcode::IMPLICIT_DEF)
          convertImplicitDefToConstZero(Insert, MRI, TII, MF, LIS);
	*/
	
        // We stackified an operand. Add the defining instruction's operands to
        // the worklist stack now to continue to build an ever deeper tree.
	//        Commuting.reset();
        TreeWalker.pushOperands(Insert);
      }

      /*
      // If we stackified any operands, skip over the tree to start looking for
      // the next instruction we can build a tree on.
      if (Insert != &*MII) {
        imposeStackOrdering(&*MII);
        MII = MachineBasicBlock::iterator(Insert).getReverse();
        Changed = true;
      }
      */

    }  // MachineInstruction
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
  
  //  return Changed;
  return false;
}

/// bundleCFG - Scan all the basic blocks to determine consistent live-in and
/// live-out sets for the FP registers. Consistent means that the set of
/// registers live-out from a block is identical to the live-in set of all
/// successors. This is not enforced by the normal live-in lists since
/// registers may be implicitly defined, or not used by all successors.
void T8xxStackPass::bundleCFGRecomputeKillFlags(MachineFunction &MF) {
  assert(LiveBundles.empty() && "Stale data in LiveBundles");
  LiveBundles.resize(Bundles->getNumBundles());

  // Gather the actual live-in masks for all MBBs.
  for (MachineBasicBlock &MBB : MF) {
    setKillFlags(MBB);

    const unsigned Mask = calcLiveInMask(&MBB, false);
    if (!Mask)
      continue;
    // Update MBB ingoing bundle mask.
    LiveBundles[Bundles->getBundle(MBB.getNumber(), false)].Mask |= Mask;
  }
}

/// processBasicBlock - Loop over all of the instructions in the basic block,
/// transforming FP instructions into their stack form.
///
bool T8xxStackPass::processBasicBlock(MachineFunction &MF, MachineBasicBlock &BB) {
  bool Changed = false;
  MBB = &BB;

  printf ("################### processBasicBlock\n");
  
  //  setupBlockStack();

  for (MachineBasicBlock::iterator I = BB.begin(); I != BB.end(); ++I) {
    MachineInstr &MI = *I;
    uint64_t Flags = MI.getDesc().TSFlags;

    // Debug stuff
    MI.dump ();

    /*    
    unsigned FPInstClass = Flags & X86II::FPTypeMask;
    if (MI.isInlineAsm())
      FPInstClass = X86II::SpecialFP;

    if (MI.isCopy() && isFPCopy(MI))
      FPInstClass = X86II::SpecialFP;

    if (MI.isImplicitDef() &&
        X86::RFP80RegClass.contains(MI.getOperand(0).getReg()))
      FPInstClass = X86II::SpecialFP;

    if (MI.isCall())
      FPInstClass = X86II::SpecialFP;

    if (FPInstClass == X86II::NotFP)
      continue;  // Efficiently ignore non-fp insts!
    */

    MachineInstr *PrevMI = nullptr;
    if (I != BB.begin())
      PrevMI = &*std::prev(I);

    ++NumFP;  // Keep track of # of pseudo instrs
    LLVM_DEBUG(dbgs() << "\nFPInst:\t" << MI);

    // Get dead variables list now because the MI pointer may be deleted as part
    // of processing!
    SmallVector<unsigned, 8> DeadRegs;
    for (const MachineOperand &MO : MI.operands())
      if (MO.isReg() && MO.isDead())
        DeadRegs.push_back(MO.getReg());

    if ((MI.getOpcode() == T8xx::LDRi16wpop) ||
	(MI.getOpcode() == T8xx::LDRi32wpop) ||
	(MI.getOpcode() == T8xx::LEA_ADDri))
      {
	handleZeroArgFP (I);
      }

    if ((MI.getOpcode() == T8xx::STRi32regop) ||
	(MI.getOpcode() == T8xx::STRi8regop))
      handleOneArgFP (I);

    /*
    if (MI.getOpcode() == T8xx::ADDimmr)
      handleOneArgFPRW (I);
    
    if (MI.getOpcode() == T8xx::ADDraw)
      handleTwoArgFP (I);
    */

    /*    
    switch (FPInstClass) {
    case X86II::ZeroArgFP:  handleZeroArgFP(I); break;
    case X86II::OneArgFP:   handleOneArgFP(I);  break;  // fstp ST(0)
    case X86II::OneArgFPRW: handleOneArgFPRW(I); break; // ST(0) = fsqrt(ST(0))
    case X86II::TwoArgFP:   handleTwoArgFP(I);  break;
    case X86II::CompareFP:  handleCompareFP(I); break;
    case X86II::CondMovFP:  handleCondMovFP(I); break;
    case X86II::SpecialFP:  handleSpecialFP(I); break;
    default: llvm_unreachable("Unknown FP Type!");
    }
    */


    // Check to see if any of the values defined by this instruction are dead
    // after definition.  If so, pop them.
    /*
    for (unsigned i = 0, e = DeadRegs.size(); i != e; ++i) {
      unsigned Reg = DeadRegs[i];
      // Check if Reg is live on the stack. An inline-asm register operand that
      // is in the clobber list and marked dead might not be live on the stack.
      static_assert(T8xx::CREG - T8xx::AREG == 3, "sequential Stack regnumbers");
      if (Reg >= T8xx::AREG && Reg <= T8xx::CREG && isLive(Reg-T8xx::AREG)) {
        LLVM_DEBUG(dbgs() << "Register FP#" << Reg - T8xx::AREG << " is dead!\n");
        freeStackSlotAfter(I, Reg-T8xx::AREG);
      }
    }
    */

    // Print out all of the instructions expanded to if -debug
    LLVM_DEBUG({
      MachineBasicBlock::iterator PrevI = PrevMI;
      if (I == PrevI) {
        dbgs() << "Just deleted pseudo instruction\n";
      } else {
        MachineBasicBlock::iterator Start = I;
        // Rewind to first instruction newly inserted.
        while (Start != BB.begin() && std::prev(Start) != PrevI)
          --Start;
        dbgs() << "Inserted instructions:\n\t";
        Start->print(dbgs());
        while (++Start != std::next(I)) {
        }
      }
      dumpStack();
    });
    (void)PrevMI;

    Changed = true;
  }

  finishBlockStack();

  return Changed;
}

/// setupBlockStack - Use the live bundles to set up our model of the stack
/// to match predecessors' live out stack.
void T8xxStackPass::setupBlockStack() {
  LLVM_DEBUG(dbgs() << "\nSetting up live-ins for " << printMBBReference(*MBB)
                    << " derived from " << MBB->getName() << ".\n");
  StackTop = 0;
  // Get the live-in bundle for MBB.
  const LiveBundle &Bundle =
    LiveBundles[Bundles->getBundle(MBB->getNumber(), false)];

  if (!Bundle.Mask) {
    LLVM_DEBUG(dbgs() << "Block has no FP live-ins.\n");
    return;
  }

  // Depth-first iteration should ensure that we always have an assigned stack.
  assert(Bundle.isFixed() && "Reached block before any predecessors");

  // Push the fixed live-in registers.
  for (unsigned i = Bundle.FixCount; i > 0; --i) {
    LLVM_DEBUG(dbgs() << "Live-in st(" << (i - 1) << "): %fp"
                      << unsigned(Bundle.FixStack[i - 1]) << '\n');
    pushReg(Bundle.FixStack[i-1]);
  }

  // Kill off unwanted live-ins. This can happen with a critical edge.
  // FIXME: We could keep these live registers around as zombies. They may need
  // to be revived at the end of a short block. It might save a few instrs.
  unsigned Mask = calcLiveInMask(MBB, /*RemoveFPs=*/true);
  adjustLiveRegs(Mask, MBB->begin());
  LLVM_DEBUG(MBB->dump());
}

/// finishBlockStack - Revive live-outs that are implicitly defined out of
/// MBB. Shuffle live registers to match the expected fixed stack of any
/// predecessors, and ensure that all predecessors are expecting the same
/// stack.
void T8xxStackPass::finishBlockStack() {
  // The RET handling below takes care of return blocks for us.
  if (MBB->succ_empty())
    return;

  LLVM_DEBUG(dbgs() << "Setting up live-outs for " << printMBBReference(*MBB)
                    << " derived from " << MBB->getName() << ".\n");

  // Get MBB's live-out bundle.
  unsigned BundleIdx = Bundles->getBundle(MBB->getNumber(), true);
  LiveBundle &Bundle = LiveBundles[BundleIdx];

  // We may need to kill and define some registers to match successors.
  // FIXME: This can probably be combined with the shuffle below.
  MachineBasicBlock::iterator Term = MBB->getFirstTerminator();
  adjustLiveRegs(Bundle.Mask, Term);

  if (!Bundle.Mask) {
    LLVM_DEBUG(dbgs() << "No live-outs.\n");
    return;
  }

  // Has the stack order been fixed yet?
  LLVM_DEBUG(dbgs() << "LB#" << BundleIdx << ": ");
  if (Bundle.isFixed()) {
    LLVM_DEBUG(dbgs() << "Shuffling stack to match.\n");
    shuffleStackTop(Bundle.FixStack, Bundle.FixCount, Term);
  } else {
    // Not fixed yet, we get to choose.
    LLVM_DEBUG(dbgs() << "Fixing stack order now.\n");
    Bundle.FixCount = StackTop;
    for (unsigned i = 0; i < StackTop; ++i)
      Bundle.FixStack[i] = getStackEntry(i);
  }
}


//===----------------------------------------------------------------------===//
// Efficient Lookup Table Support
//===----------------------------------------------------------------------===//

namespace {
  struct TableEntry {
    uint16_t from;
    uint16_t to;
    bool operator<(const TableEntry &TE) const { return from < TE.from; }
    friend bool operator<(const TableEntry &TE, unsigned V) {
      return TE.from < V;
    }
    friend bool LLVM_ATTRIBUTE_UNUSED operator<(unsigned V,
                                                const TableEntry &TE) {
      return V < TE.from;
    }
  };
}

static int Lookup(ArrayRef<TableEntry> Table, unsigned Opcode) {
  const TableEntry *I = llvm::lower_bound(Table, Opcode);
  if (I != Table.end() && I->from == Opcode)
    return I->to;
  return -1;
}

#ifdef NDEBUG
#define ASSERT_SORTED(TABLE)
#else
#define ASSERT_SORTED(TABLE)                                                   \
  {                                                                            \
    static std::atomic<bool> TABLE##Checked(false);                            \
    if (!TABLE##Checked.load(std::memory_order_relaxed)) {                     \
      assert(is_sorted(TABLE) &&                                               \
             "All lookup tables must be sorted for efficient access!");        \
      TABLE##Checked.store(true, std::memory_order_relaxed);                   \
    }                                                                          \
  }
#endif

//===----------------------------------------------------------------------===//
// Register File -> Register Stack Mapping Methods
//===----------------------------------------------------------------------===//

// OpcodeTable - Sorted map of register instructions to their stack version.
// The first element is an register file pseudo instruction, the second is the
// concrete X86 instruction which uses the register stack.
//
static const TableEntry OpcodeTable[] = {
  { T8xx::SUBregregop     , T8xx::SUB     },
};

static unsigned getConcreteOpcode(unsigned Opcode) {
  ASSERT_SORTED(OpcodeTable);
  int Opc = Lookup(OpcodeTable, Opcode);
  assert(Opc != -1 && "FP Stack instruction not in OpcodeTable!");
  return Opc;
}

//===----------------------------------------------------------------------===//
// Helper Methods
//===----------------------------------------------------------------------===//

// PopTable - Sorted map of instructions to their popping version.  The first
// element is an instruction, the second is the version which pops.
//
static const TableEntry PopTable[] = {
  { T8xx::SUBregregop , T8xx::SUB  },

};

/*

static bool doesInstructionSetFPSW(MachineInstr &MI) {
  if (const MachineOperand *MO = MI.findRegisterDefOperand(X86::FPSW))
    if (!MO->isDead())
      return true;
  return false;
}

static MachineBasicBlock::iterator
getNextFPInstruction(MachineBasicBlock::iterator I) {
  MachineBasicBlock &MBB = *I->getParent();
  while (++I != MBB.end()) {
    MachineInstr &MI = *I;
    if (X86::isX87Instruction(MI))
      return I;
  }
  return MBB.end();
}

/// popStackAfter - Pop the current value off of the top of the FP stack after
/// the specified instruction.  This attempts to be sneaky and combine the pop
/// into the instruction itself if possible.  The iterator is left pointing to
/// the last instruction, be it a new pop instruction inserted, or the old
/// instruction if it was modified in place.
///
void T8xxStackPass::popStackAfter(MachineBasicBlock::iterator &I) {
  MachineInstr &MI = *I;
  const DebugLoc &dl = MI.getDebugLoc();
  ASSERT_SORTED(PopTable);

  popReg();

  // Check to see if there is a popping version of this instruction...
  int Opcode = Lookup(PopTable, I->getOpcode());
  if (Opcode != -1) {
    I->setDesc(TII->get(Opcode));
    if (Opcode == X86::FCOMPP || Opcode == X86::UCOM_FPPr)
      I->removeOperand(0);
    MI.dropDebugNumber();
  } else {    // Insert an explicit pop
    // If this instruction sets FPSW, which is read in following instruction,
    // insert pop after that reader.
    if (doesInstructionSetFPSW(MI)) {
      MachineBasicBlock &MBB = *MI.getParent();
      MachineBasicBlock::iterator Next = getNextFPInstruction(I);
      if (Next != MBB.end() && Next->readsRegister(X86::FPSW))
        I = Next;
    }
    I = BuildMI(*MBB, ++I, dl, TII->get(X86::ST_FPrr)).addReg(X86::ST0);
  }
}

/// freeStackSlotAfter - Free the specified register from the register stack, so
/// that it is no longer in a register.  If the register is currently at the top
/// of the stack, we just pop the current instruction, otherwise we store the
/// current top-of-stack into the specified slot, then pop the top of stack.
void T8xxStackPass::freeStackSlotAfter(MachineBasicBlock::iterator &I, unsigned FPRegNo) {
  if (getStackEntry(0) == FPRegNo) {  // already at the top of stack? easy.
    popStackAfter(I);
    return;
  }

  // Otherwise, store the top of stack into the dead slot, killing the operand
  // without having to add in an explicit xchg then pop.
  //
  I = freeStackSlotBefore(++I, FPRegNo);
}

/// freeStackSlotBefore - Free the specified register without trying any
/// folding.
MachineBasicBlock::iterator
T8xxStackPass::freeStackSlotBefore(MachineBasicBlock::iterator I, unsigned FPRegNo) {
  unsigned STReg    = getSTReg(FPRegNo);
  unsigned OldSlot  = getSlot(FPRegNo);
  unsigned TopReg   = Stack[StackTop-1];
  Stack[OldSlot]    = TopReg;
  RegMap[TopReg]    = OldSlot;
  RegMap[FPRegNo]   = ~0;
  Stack[--StackTop] = ~0;
  return BuildMI(*MBB, I, DebugLoc(), TII->get(X86::ST_FPrr))
      .addReg(STReg)
      .getInstr();
}

*/

/// adjustLiveRegs - Kill and revive registers such that exactly the FP
/// registers with a bit in Mask are live.
void T8xxStackPass::adjustLiveRegs(unsigned Mask, MachineBasicBlock::iterator I) {
  unsigned Defs = Mask;
  unsigned Kills = 0;
  for (unsigned i = 0; i < StackTop; ++i) {
    unsigned RegNo = Stack[i];
    if (!(Defs & (1 << RegNo)))
      // This register is live, but we don't want it.
      Kills |= (1 << RegNo);
    else
      // We don't need to imp-def this live register.
      Defs &= ~(1 << RegNo);
  }
  assert((Kills & Defs) == 0 && "Register needs killing and def'ing?");

  /*
  
  // Produce implicit-defs for free by using killed registers.
  while (Kills && Defs) {
    unsigned KReg = countTrailingZeros(Kills);
    unsigned DReg = countTrailingZeros(Defs);
    LLVM_DEBUG(dbgs() << "Renaming %fp" << KReg << " as imp %fp" << DReg
                      << "\n");
    std::swap(Stack[getSlot(KReg)], Stack[getSlot(DReg)]);
    std::swap(RegMap[KReg], RegMap[DReg]);
    Kills &= ~(1 << KReg);
    Defs &= ~(1 << DReg);
  }

  // Kill registers by popping.
  if (Kills && I != MBB->begin()) {
    MachineBasicBlock::iterator I2 = std::prev(I);
    while (StackTop) {
      unsigned KReg = getStackEntry(0);
      if (!(Kills & (1 << KReg)))
        break;
      LLVM_DEBUG(dbgs() << "Popping %fp" << KReg << "\n");
      popStackAfter(I2);
      Kills &= ~(1 << KReg);
    }
  }

  // Manually kill the rest.
  while (Kills) {
    unsigned KReg = countTrailingZeros(Kills);
    LLVM_DEBUG(dbgs() << "Killing %fp" << KReg << "\n");
    freeStackSlotBefore(I, KReg);
    Kills &= ~(1 << KReg);
  }

  // Load zeros for all the imp-defs.
  while(Defs) {
    unsigned DReg = countTrailingZeros(Defs);
    LLVM_DEBUG(dbgs() << "Defining %fp" << DReg << " as 0\n");
    BuildMI(*MBB, I, DebugLoc(), TII->get(X86::LD_F0));
    pushReg(DReg);
    Defs &= ~(1 << DReg);
  }

  */

  // Now we should have the correct registers live.
  LLVM_DEBUG(dumpStack());
  assert(StackTop == countPopulation(Mask) && "Live count mismatch");
}

/// shuffleStackTop - emit fxch instructions before I to shuffle the top
/// FixCount entries into the order given by FixStack.
/// FIXME: Is there a better algorithm than insertion sort?
void T8xxStackPass::shuffleStackTop(const unsigned char *FixStack,
                          unsigned FixCount,
                          MachineBasicBlock::iterator I) {
  // Move items into place, starting from the desired stack bottom.
  while (FixCount--) {
    // Old register at position FixCount.
    unsigned OldReg = getStackEntry(FixCount);
    // Desired register at position FixCount.
    unsigned Reg = FixStack[FixCount];
    if (Reg == OldReg)
      continue;
    // (Reg st0) (OldReg st0) = (Reg OldReg st0)
    moveToTop(Reg, I);
    if (FixCount > 0)
      moveToTop(OldReg, I);
  }
  LLVM_DEBUG(dumpStack());
}

#if 0
//===----------------------------------------------------------------------===//
// Instruction transformation implementation
//===----------------------------------------------------------------------===//

void T8xxStackPass::handleCall(MachineBasicBlock::iterator &I) {
  MachineInstr &MI = *I;
  unsigned STReturns = 0;

  bool ClobbersFPStack = false;
  for (unsigned i = 0, e = MI.getNumOperands(); i != e; ++i) {
    MachineOperand &Op = MI.getOperand(i);
    // Check if this call clobbers the FP stack.
    // is sufficient.
    if (Op.isRegMask()) {
      bool ClobbersFP0 = Op.clobbersPhysReg(X86::FP0);
#ifndef NDEBUG
      static_assert(X86::FP7 - X86::FP0 == 7, "sequential FP regnumbers");
      for (unsigned i = 1; i != 8; ++i)
        assert(Op.clobbersPhysReg(X86::FP0 + i) == ClobbersFP0 &&
               "Inconsistent FP register clobber");
#endif

      if (ClobbersFP0)
        ClobbersFPStack = true;
    }

    if (!Op.isReg() || Op.getReg() < X86::FP0 || Op.getReg() > X86::FP6)
      continue;

    assert(Op.isImplicit() && "Expected implicit def/use");

    if (Op.isDef())
      STReturns |= 1 << getFPReg(Op);

    // Remove the operand so that later passes don't see it.
    MI.removeOperand(i);
    --i;
    --e;
  }

  // Most calls should have a regmask that clobbers the FP registers. If it
  // isn't present then the register allocator didn't spill the FP registers
  // so they are still on the stack.
  assert((ClobbersFPStack || STReturns == 0) &&
         "ST returns without FP stack clobber");
  if (!ClobbersFPStack)
    return;

  unsigned N = countTrailingOnes(STReturns);

  // FP registers used for function return must be consecutive starting at
  // FP0
  assert(STReturns == 0 || (isMask_32(STReturns) && N <= 2));

  // Reset the FP Stack - It is required because of possible leftovers from
  // passed arguments. The caller should assume that the FP stack is
  // returned empty (unless the callee returns values on FP stack).
  while (StackTop > 0)
    popReg();

  for (unsigned I = 0; I < N; ++I)
    pushReg(N - I - 1);

  // If this call has been modified, drop all variable values defined by it.
  // We can't track them once they've been stackified.
  if (STReturns)
    I->dropDebugNumber();
}

/// If RET has an FP register use operand, pass the first one in ST(0) and
/// the second one in ST(1).
void T8xxStackPass::handleReturn(MachineBasicBlock::iterator &I) {
  MachineInstr &MI = *I;

  // Find the register operands.
  unsigned FirstFPRegOp = ~0U, SecondFPRegOp = ~0U;
  unsigned LiveMask = 0;

  for (unsigned i = 0, e = MI.getNumOperands(); i != e; ++i) {
    MachineOperand &Op = MI.getOperand(i);
    if (!Op.isReg() || Op.getReg() < X86::FP0 || Op.getReg() > X86::FP6)
      continue;
    // FP Register uses must be kills unless there are two uses of the same
    // register, in which case only one will be a kill.
    assert(Op.isUse() &&
           (Op.isKill() ||                    // Marked kill.
            getFPReg(Op) == FirstFPRegOp ||   // Second instance.
            MI.killsRegister(Op.getReg())) && // Later use is marked kill.
           "Ret only defs operands, and values aren't live beyond it");

    if (FirstFPRegOp == ~0U)
      FirstFPRegOp = getFPReg(Op);
    else {
      assert(SecondFPRegOp == ~0U && "More than two fp operands!");
      SecondFPRegOp = getFPReg(Op);
    }
    LiveMask |= (1 << getFPReg(Op));

    // Remove the operand so that later passes don't see it.
    MI.removeOperand(i);
    --i;
    --e;
  }

  // We may have been carrying spurious live-ins, so make sure only the
  // returned registers are left live.
  adjustLiveRegs(LiveMask, MI);
  if (!LiveMask) return;  // Quick check to see if any are possible.

  // There are only four possibilities here:
  // 1) we are returning a single FP value.  In this case, it has to be in
  //    ST(0) already, so just declare success by removing the value from the
  //    FP Stack.
  if (SecondFPRegOp == ~0U) {
    // Assert that the top of stack contains the right FP register.
    assert(StackTop == 1 && FirstFPRegOp == getStackEntry(0) &&
           "Top of stack not the right register for RET!");

    // Ok, everything is good, mark the value as not being on the stack
    // anymore so that our assertion about the stack being empty at end of
    // block doesn't fire.
    StackTop = 0;
    return;
  }

  // Otherwise, we are returning two values:
  // 2) If returning the same value for both, we only have one thing in the FP
  //    stack.  Consider:  RET FP1, FP1
  if (StackTop == 1) {
    assert(FirstFPRegOp == SecondFPRegOp && FirstFPRegOp == getStackEntry(0)&&
           "Stack misconfiguration for RET!");

    // Duplicate the TOS so that we return it twice.  Just pick some other FPx
    // register to hold it.
    unsigned NewReg = ScratchFPReg;
    duplicateToTop(FirstFPRegOp, NewReg, MI);
    FirstFPRegOp = NewReg;
  }

  /// Okay we know we have two different FPx operands now:
  assert(StackTop == 2 && "Must have two values live!");

  /// 3) If SecondFPRegOp is currently in ST(0) and FirstFPRegOp is currently
  ///    in ST(1).  In this case, emit an fxch.
  if (getStackEntry(0) == SecondFPRegOp) {
    assert(getStackEntry(1) == FirstFPRegOp && "Unknown regs live");
    moveToTop(FirstFPRegOp, MI);
  }

  /// 4) Finally, FirstFPRegOp must be in ST(0) and SecondFPRegOp must be in
  /// ST(1).  Just remove both from our understanding of the stack and return.
  assert(getStackEntry(0) == FirstFPRegOp && "Unknown regs live");
  assert(getStackEntry(1) == SecondFPRegOp && "Unknown regs live");
  StackTop = 0;
}

#endif

/// handleZeroArgFP - ST(0) = fld0    ST(0) = flds <mem>
///
void T8xxStackPass::handleZeroArgFP(MachineBasicBlock::iterator &I) {
  MachineInstr &MI = *I;
  unsigned DestReg = getFPReg(MI.getOperand(0));

  // Change from the pseudo instruction to the concrete instruction.
  //  MI.removeOperand(0); // Remove the explicit ST(0) operand
  MI.getOperand(0).setReg (T8xx::STA);
  //  MI.setDesc(TII->get(getConcreteOpcode(MI.getOpcode())));
  //  MI.addOperand(
  //    MachineOperand::CreateReg(T8xx::STA, /*isDef*/ true, /*isImp*/ true));

  // Result gets pushed on the stack.
  pushReg(DestReg);

  MI.dropDebugNumber();
}


/// handleOneArgFP - fst <mem>, ST(0)
///
void T8xxStackPass::handleOneArgFP(MachineBasicBlock::iterator &I) {
  MachineInstr &MI = *I;
  unsigned NumOps = MI.getDesc().getNumOperands();
  /* TODO: See if we need this
  assert((NumOps == X86::AddrNumOperands + 1 || NumOps == 1) &&
         "Can only handle fst* & ftst instructions!");
  */

  // Is this the last use of the source register?
  unsigned Reg = getFPReg(MI.getOperand(NumOps - 1));
  bool KillsSrc = MI.killsRegister(T8xx::AREG + Reg);

  // FISTP64m is strange because there isn't a non-popping versions.
  // If we have one _and_ we don't want to pop the operand, duplicate the value
  // on the stack instead of moving it.  This ensure that popping the value is
  // always ok.
  // Ditto FISTTP16m, FISTTP32m, FISTTP64m, ST_FpP80m.
  //
  /*
  if (!KillsSrc && (MI.getOpcode() == X86::IST_Fp64m32 ||
                    MI.getOpcode() == X86::ISTT_Fp16m32 ||
                    MI.getOpcode() == X86::ISTT_Fp32m32 ||
                    MI.getOpcode() == X86::ISTT_Fp64m32 ||
                    MI.getOpcode() == X86::IST_Fp64m64 ||
                    MI.getOpcode() == X86::ISTT_Fp16m64 ||
                    MI.getOpcode() == X86::ISTT_Fp32m64 ||
                    MI.getOpcode() == X86::ISTT_Fp64m64 ||
                    MI.getOpcode() == X86::IST_Fp64m80 ||
                    MI.getOpcode() == X86::ISTT_Fp16m80 ||
                    MI.getOpcode() == X86::ISTT_Fp32m80 ||
                    MI.getOpcode() == X86::ISTT_Fp64m80 ||
                    MI.getOpcode() == X86::ST_FpP80m)) {
    duplicateToTop(Reg, ScratchFPReg, I);
  } else {
    moveToTop(Reg, I);            // Move to the top of the stack...
  }
  */

  // Convert from the pseudo instruction to the concrete instruction.
  MI.removeOperand(NumOps - 1); // Remove explicit ST(0) operand
  MI.setDesc(TII->get(getConcreteOpcode(MI.getOpcode())));
  MI.addOperand(
      MachineOperand::CreateReg(T8xx::STA, /*isDef*/ false, /*isImp*/ true));

  /*
  if (MI.getOpcode() == X86::IST_FP64m || MI.getOpcode() == X86::ISTT_FP16m ||
      MI.getOpcode() == X86::ISTT_FP32m || MI.getOpcode() == X86::ISTT_FP64m ||
      MI.getOpcode() == X86::ST_FP80m) {
    if (StackTop == 0)
      report_fatal_error("Stack empty??");
    --StackTop;
  } else if (KillsSrc) { // Last use of operand?
    popStackAfter(I);
  }
  */

  MI.dropDebugNumber();
}


#if 0

/// handleOneArgFPRW: Handle instructions that read from the top of stack and
/// replace the value with a newly computed value.  These instructions may have
/// non-fp operands after their FP operands.
///
///  Examples:
///     R1 = fchs R2
///     R1 = fadd R2, [mem]
///
void T8xxStackPass::handleOneArgFPRW(MachineBasicBlock::iterator &I) {
  MachineInstr &MI = *I;
#ifndef NDEBUG
  unsigned NumOps = MI.getDesc().getNumOperands();
  assert(NumOps >= 2 && "FPRW instructions must have 2 ops!!");
#endif

  // Is this the last use of the source register?
  unsigned Reg = getFPReg(MI.getOperand(1));
  bool KillsSrc = MI.killsRegister(X86::FP0 + Reg);

  if (KillsSrc) {
    // If this is the last use of the source register, just make sure it's on
    // the top of the stack.
    moveToTop(Reg, I);
    if (StackTop == 0)
      report_fatal_error("Stack cannot be empty!");
    --StackTop;
    pushReg(getFPReg(MI.getOperand(0)));
  } else {
    // If this is not the last use of the source register, _copy_ it to the top
    // of the stack.
    duplicateToTop(Reg, getFPReg(MI.getOperand(0)), I);
  }

  // Change from the pseudo instruction to the concrete instruction.
  MI.removeOperand(1); // Drop the source operand.
  MI.removeOperand(0); // Drop the destination operand.
  MI.setDesc(TII->get(getConcreteOpcode(MI.getOpcode())));
  MI.dropDebugNumber();
}


//===----------------------------------------------------------------------===//
// Define tables of various ways to map pseudo instructions
//

// ForwardST0Table - Map: A = B op C  into: ST(0) = ST(0) op ST(i)
static const TableEntry ForwardST0Table[] = {
  { X86::ADD_Fp32  , X86::ADD_FST0r },
  { X86::ADD_Fp64  , X86::ADD_FST0r },
  { X86::ADD_Fp80  , X86::ADD_FST0r },
  { X86::DIV_Fp32  , X86::DIV_FST0r },
  { X86::DIV_Fp64  , X86::DIV_FST0r },
  { X86::DIV_Fp80  , X86::DIV_FST0r },
  { X86::MUL_Fp32  , X86::MUL_FST0r },
  { X86::MUL_Fp64  , X86::MUL_FST0r },
  { X86::MUL_Fp80  , X86::MUL_FST0r },
  { X86::SUB_Fp32  , X86::SUB_FST0r },
  { X86::SUB_Fp64  , X86::SUB_FST0r },
  { X86::SUB_Fp80  , X86::SUB_FST0r },
};

// ReverseST0Table - Map: A = B op C  into: ST(0) = ST(i) op ST(0)
static const TableEntry ReverseST0Table[] = {
  { X86::ADD_Fp32  , X86::ADD_FST0r  },   // commutative
  { X86::ADD_Fp64  , X86::ADD_FST0r  },   // commutative
  { X86::ADD_Fp80  , X86::ADD_FST0r  },   // commutative
  { X86::DIV_Fp32  , X86::DIVR_FST0r },
  { X86::DIV_Fp64  , X86::DIVR_FST0r },
  { X86::DIV_Fp80  , X86::DIVR_FST0r },
  { X86::MUL_Fp32  , X86::MUL_FST0r  },   // commutative
  { X86::MUL_Fp64  , X86::MUL_FST0r  },   // commutative
  { X86::MUL_Fp80  , X86::MUL_FST0r  },   // commutative
  { X86::SUB_Fp32  , X86::SUBR_FST0r },
  { X86::SUB_Fp64  , X86::SUBR_FST0r },
  { X86::SUB_Fp80  , X86::SUBR_FST0r },
};

// ForwardSTiTable - Map: A = B op C  into: ST(i) = ST(0) op ST(i)
static const TableEntry ForwardSTiTable[] = {
  { X86::ADD_Fp32  , X86::ADD_FrST0  },   // commutative
  { X86::ADD_Fp64  , X86::ADD_FrST0  },   // commutative
  { X86::ADD_Fp80  , X86::ADD_FrST0  },   // commutative
  { X86::DIV_Fp32  , X86::DIVR_FrST0 },
  { X86::DIV_Fp64  , X86::DIVR_FrST0 },
  { X86::DIV_Fp80  , X86::DIVR_FrST0 },
  { X86::MUL_Fp32  , X86::MUL_FrST0  },   // commutative
  { X86::MUL_Fp64  , X86::MUL_FrST0  },   // commutative
  { X86::MUL_Fp80  , X86::MUL_FrST0  },   // commutative
  { X86::SUB_Fp32  , X86::SUBR_FrST0 },
  { X86::SUB_Fp64  , X86::SUBR_FrST0 },
  { X86::SUB_Fp80  , X86::SUBR_FrST0 },
};

// ReverseSTiTable - Map: A = B op C  into: ST(i) = ST(i) op ST(0)
static const TableEntry ReverseSTiTable[] = {
  { X86::ADD_Fp32  , X86::ADD_FrST0 },
  { X86::ADD_Fp64  , X86::ADD_FrST0 },
  { X86::ADD_Fp80  , X86::ADD_FrST0 },
  { X86::DIV_Fp32  , X86::DIV_FrST0 },
  { X86::DIV_Fp64  , X86::DIV_FrST0 },
  { X86::DIV_Fp80  , X86::DIV_FrST0 },
  { X86::MUL_Fp32  , X86::MUL_FrST0 },
  { X86::MUL_Fp64  , X86::MUL_FrST0 },
  { X86::MUL_Fp80  , X86::MUL_FrST0 },
  { X86::SUB_Fp32  , X86::SUB_FrST0 },
  { X86::SUB_Fp64  , X86::SUB_FrST0 },
  { X86::SUB_Fp80  , X86::SUB_FrST0 },
};


/// handleTwoArgFP - Handle instructions like FADD and friends which are virtual
/// instructions which need to be simplified and possibly transformed.
///
/// Result: ST(0) = fsub  ST(0), ST(i)
///         ST(i) = fsub  ST(0), ST(i)
///         ST(0) = fsubr ST(0), ST(i)
///         ST(i) = fsubr ST(0), ST(i)
///
void T8xxStackPass::handleTwoArgFP(MachineBasicBlock::iterator &I) {
  ASSERT_SORTED(ForwardST0Table); ASSERT_SORTED(ReverseST0Table);
  ASSERT_SORTED(ForwardSTiTable); ASSERT_SORTED(ReverseSTiTable);
  MachineInstr &MI = *I;

  unsigned NumOperands = MI.getDesc().getNumOperands();
  assert(NumOperands == 3 && "Illegal TwoArgFP instruction!");
  unsigned Dest = getFPReg(MI.getOperand(0));
  unsigned Op0 = getFPReg(MI.getOperand(NumOperands - 2));
  unsigned Op1 = getFPReg(MI.getOperand(NumOperands - 1));
  bool KillsOp0 = MI.killsRegister(X86::FP0 + Op0);
  bool KillsOp1 = MI.killsRegister(X86::FP0 + Op1);
  const DebugLoc &dl = MI.getDebugLoc();

  unsigned TOS = getStackEntry(0);

  // One of our operands must be on the top of the stack.  If neither is yet, we
  // need to move one.
  if (Op0 != TOS && Op1 != TOS) {   // No operand at TOS?
    // We can choose to move either operand to the top of the stack.  If one of
    // the operands is killed by this instruction, we want that one so that we
    // can update right on top of the old version.
    if (KillsOp0) {
      moveToTop(Op0, I);         // Move dead operand to TOS.
      TOS = Op0;
    } else if (KillsOp1) {
      moveToTop(Op1, I);
      TOS = Op1;
    } else {
      // All of the operands are live after this instruction executes, so we
      // cannot update on top of any operand.  Because of this, we must
      // duplicate one of the stack elements to the top.  It doesn't matter
      // which one we pick.
      //
      duplicateToTop(Op0, Dest, I);
      Op0 = TOS = Dest;
      KillsOp0 = true;
    }
  } else if (!KillsOp0 && !KillsOp1) {
    // If we DO have one of our operands at the top of the stack, but we don't
    // have a dead operand, we must duplicate one of the operands to a new slot
    // on the stack.
    duplicateToTop(Op0, Dest, I);
    Op0 = TOS = Dest;
    KillsOp0 = true;
  }

  // Now we know that one of our operands is on the top of the stack, and at
  // least one of our operands is killed by this instruction.
  assert((TOS == Op0 || TOS == Op1) && (KillsOp0 || KillsOp1) &&
         "Stack conditions not set up right!");

  // We decide which form to use based on what is on the top of the stack, and
  // which operand is killed by this instruction.
  ArrayRef<TableEntry> InstTable;
  bool isForward = TOS == Op0;
  bool updateST0 = (TOS == Op0 && !KillsOp1) || (TOS == Op1 && !KillsOp0);
  if (updateST0) {
    if (isForward)
      InstTable = ForwardST0Table;
    else
      InstTable = ReverseST0Table;
  } else {
    if (isForward)
      InstTable = ForwardSTiTable;
    else
      InstTable = ReverseSTiTable;
  }

  int Opcode = Lookup(InstTable, MI.getOpcode());
  assert(Opcode != -1 && "Unknown TwoArgFP pseudo instruction!");

  // NotTOS - The register which is not on the top of stack...
  unsigned NotTOS = (TOS == Op0) ? Op1 : Op0;

  // Replace the old instruction with a new instruction
  MBB->remove(&*I++);
  I = BuildMI(*MBB, I, dl, TII->get(Opcode)).addReg(getSTReg(NotTOS));

  if (!MI.mayRaiseFPException())
    I->setFlag(MachineInstr::MIFlag::NoFPExcept);

  // If both operands are killed, pop one off of the stack in addition to
  // overwriting the other one.
  if (KillsOp0 && KillsOp1 && Op0 != Op1) {
    assert(!updateST0 && "Should have updated other operand!");
    popStackAfter(I);   // Pop the top of stack
  }

  // Update stack information so that we know the destination register is now on
  // the stack.
  unsigned UpdatedSlot = getSlot(updateST0 ? TOS : NotTOS);
  assert(UpdatedSlot < StackTop && Dest < 7);
  Stack[UpdatedSlot]   = Dest;
  RegMap[Dest]         = UpdatedSlot;
  MBB->getParent()->deleteMachineInstr(&MI); // Remove the old instruction
}

/// handleCompareFP - Handle FUCOM and FUCOMI instructions, which have two FP
/// register arguments and no explicit destinations.
///
void T8xxStackPass::handleCompareFP(MachineBasicBlock::iterator &I) {
  MachineInstr &MI = *I;

  unsigned NumOperands = MI.getDesc().getNumOperands();
  assert(NumOperands == 2 && "Illegal FUCOM* instruction!");
  unsigned Op0 = getFPReg(MI.getOperand(NumOperands - 2));
  unsigned Op1 = getFPReg(MI.getOperand(NumOperands - 1));
  bool KillsOp0 = MI.killsRegister(X86::FP0 + Op0);
  bool KillsOp1 = MI.killsRegister(X86::FP0 + Op1);

  // Make sure the first operand is on the top of stack, the other one can be
  // anywhere.
  moveToTop(Op0, I);

  // Change from the pseudo instruction to the concrete instruction.
  MI.getOperand(0).setReg(getSTReg(Op1));
  MI.removeOperand(1);
  MI.setDesc(TII->get(getConcreteOpcode(MI.getOpcode())));
  MI.dropDebugNumber();

  // If any of the operands are killed by this instruction, free them.
  if (KillsOp0) freeStackSlotAfter(I, Op0);
  if (KillsOp1 && Op0 != Op1) freeStackSlotAfter(I, Op1);
}

/// handleCondMovFP - Handle two address conditional move instructions.  These
/// instructions move a st(i) register to st(0) iff a condition is true.  These
/// instructions require that the first operand is at the top of the stack, but
/// otherwise don't modify the stack at all.
void T8xxStackPass::handleCondMovFP(MachineBasicBlock::iterator &I) {
  MachineInstr &MI = *I;

  unsigned Op0 = getFPReg(MI.getOperand(0));
  unsigned Op1 = getFPReg(MI.getOperand(2));
  bool KillsOp1 = MI.killsRegister(X86::FP0 + Op1);

  // The first operand *must* be on the top of the stack.
  moveToTop(Op0, I);

  // Change the second operand to the stack register that the operand is in.
  // Change from the pseudo instruction to the concrete instruction.
  MI.removeOperand(0);
  MI.removeOperand(1);
  MI.getOperand(0).setReg(getSTReg(Op1));
  MI.setDesc(TII->get(getConcreteOpcode(MI.getOpcode())));
  MI.dropDebugNumber();

  // If we kill the second operand, make sure to pop it from the stack.
  if (Op0 != Op1 && KillsOp1) {
    // Get this value off of the register stack.
    freeStackSlotAfter(I, Op1);
  }
}


/// handleSpecialFP - Handle special instructions which behave unlike other
/// floating point instructions.  This is primarily intended for use by pseudo
/// instructions.
///
void T8xxStackPass::handleSpecialFP(MachineBasicBlock::iterator &Inst) {
  MachineInstr &MI = *Inst;

  if (MI.isCall()) {
    handleCall(Inst);
    return;
  }

  if (MI.isReturn()) {
    handleReturn(Inst);
    return;
  }

  switch (MI.getOpcode()) {
  default: llvm_unreachable("Unknown SpecialFP instruction!");
  case TargetOpcode::COPY: {
    // We handle three kinds of copies: FP <- FP, FP <- ST, and ST <- FP.
    const MachineOperand &MO1 = MI.getOperand(1);
    const MachineOperand &MO0 = MI.getOperand(0);
    bool KillsSrc = MI.killsRegister(MO1.getReg());

    // FP <- FP copy.
    unsigned DstFP = getFPReg(MO0);
    unsigned SrcFP = getFPReg(MO1);
    assert(isLive(SrcFP) && "Cannot copy dead register");
    if (KillsSrc) {
      // If the input operand is killed, we can just change the owner of the
      // incoming stack slot into the result.
      unsigned Slot = getSlot(SrcFP);
      Stack[Slot] = DstFP;
      RegMap[DstFP] = Slot;
    } else {
      // For COPY we just duplicate the specified value to a new stack slot.
      // This could be made better, but would require substantial changes.
      duplicateToTop(SrcFP, DstFP, Inst);
    }
    break;
  }

  case TargetOpcode::IMPLICIT_DEF: {
    // All FP registers must be explicitly defined, so load a 0 instead.
    unsigned Reg = MI.getOperand(0).getReg() - X86::FP0;
    LLVM_DEBUG(dbgs() << "Emitting LD_F0 for implicit FP" << Reg << '\n');
    BuildMI(*MBB, Inst, MI.getDebugLoc(), TII->get(X86::LD_F0));
    pushReg(Reg);
    break;
  }

  case TargetOpcode::INLINEASM:
  case TargetOpcode::INLINEASM_BR: {
    // The inline asm MachineInstr currently only *uses* FP registers for the
    // 'f' constraint.  These should be turned into the current ST(x) register
    // in the machine instr.
    //
    // There are special rules for x87 inline assembly. The compiler must know
    // exactly how many registers are popped and pushed implicitly by the asm.
    // Otherwise it is not possible to restore the stack state after the inline
    // asm.
    //
    // There are 3 kinds of input operands:
    //
    // 1. Popped inputs. These must appear at the stack top in ST0-STn. A
    //    popped input operand must be in a fixed stack slot, and it is either
    //    tied to an output operand, or in the clobber list. The MI has ST use
    //    and def operands for these inputs.
    //
    // 2. Fixed inputs. These inputs appear in fixed stack slots, but are
    //    preserved by the inline asm. The fixed stack slots must be STn-STm
    //    following the popped inputs. A fixed input operand cannot be tied to
    //    an output or appear in the clobber list. The MI has ST use operands
    //    and no defs for these inputs.
    //
    // 3. Preserved inputs. These inputs use the "f" constraint which is
    //    represented as an FP register. The inline asm won't change these
    //    stack slots.
    //
    // Outputs must be in ST registers, FP outputs are not allowed. Clobbered
    // registers do not count as output operands. The inline asm changes the
    // stack as if it popped all the popped inputs and then pushed all the
    // output operands.

    // Scan the assembly for ST registers used, defined and clobbered. We can
    // only tell clobbers from defs by looking at the asm descriptor.
    unsigned STUses = 0, STDefs = 0, STClobbers = 0;
    unsigned NumOps = 0;
    SmallSet<unsigned, 1> FRegIdx;
    unsigned RCID;

    for (unsigned i = InlineAsm::MIOp_FirstOperand, e = MI.getNumOperands();
         i != e && MI.getOperand(i).isImm(); i += 1 + NumOps) {
      unsigned Flags = MI.getOperand(i).getImm();

      NumOps = InlineAsm::getNumOperandRegisters(Flags);
      if (NumOps != 1)
        continue;
      const MachineOperand &MO = MI.getOperand(i + 1);
      if (!MO.isReg())
        continue;
      unsigned STReg = MO.getReg() - X86::FP0;
      if (STReg >= 8)
        continue;

      // If the flag has a register class constraint, this must be an operand
      // with constraint "f". Record its index and continue.
      if (InlineAsm::hasRegClassConstraint(Flags, RCID)) {
        FRegIdx.insert(i + 1);
        continue;
      }

      switch (InlineAsm::getKind(Flags)) {
      case InlineAsm::Kind_RegUse:
        STUses |= (1u << STReg);
        break;
      case InlineAsm::Kind_RegDef:
      case InlineAsm::Kind_RegDefEarlyClobber:
        STDefs |= (1u << STReg);
        break;
      case InlineAsm::Kind_Clobber:
        STClobbers |= (1u << STReg);
        break;
      default:
        break;
      }
    }

    if (STUses && !isMask_32(STUses))
      MI.emitError("fixed input regs must be last on the x87 stack");
    unsigned NumSTUses = countTrailingOnes(STUses);

    // Defs must be contiguous from the stack top. ST0-STn.
    if (STDefs && !isMask_32(STDefs)) {
      MI.emitError("output regs must be last on the x87 stack");
      STDefs = NextPowerOf2(STDefs) - 1;
    }
    unsigned NumSTDefs = countTrailingOnes(STDefs);

    // So must the clobbered stack slots. ST0-STm, m >= n.
    if (STClobbers && !isMask_32(STDefs | STClobbers))
      MI.emitError("clobbers must be last on the x87 stack");

    // Popped inputs are the ones that are also clobbered or defined.
    unsigned STPopped = STUses & (STDefs | STClobbers);
    if (STPopped && !isMask_32(STPopped))
      MI.emitError("implicitly popped regs must be last on the x87 stack");
    unsigned NumSTPopped = countTrailingOnes(STPopped);

    LLVM_DEBUG(dbgs() << "Asm uses " << NumSTUses << " fixed regs, pops "
                      << NumSTPopped << ", and defines " << NumSTDefs
                      << " regs.\n");

#ifndef NDEBUG
    // If any input operand uses constraint "f", all output register
    // constraints must be early-clobber defs.
    for (unsigned I = 0, E = MI.getNumOperands(); I < E; ++I)
      if (FRegIdx.count(I)) {
        assert((1 << getFPReg(MI.getOperand(I)) & STDefs) == 0 &&
               "Operands with constraint \"f\" cannot overlap with defs");
      }
#endif

    // Collect all FP registers (register operands with constraints "t", "u",
    // and "f") to kill afer the instruction.
    unsigned FPKills = ((1u << NumFPRegs) - 1) & ~0xff;
    for (const MachineOperand &Op : MI.operands()) {
      if (!Op.isReg() || Op.getReg() < X86::FP0 || Op.getReg() > X86::FP6)
        continue;
      unsigned FPReg = getFPReg(Op);

      // If we kill this operand, make sure to pop it from the stack after the
      // asm.  We just remember it for now, and pop them all off at the end in
      // a batch.
      if (Op.isUse() && Op.isKill())
        FPKills |= 1U << FPReg;
    }

    // Do not include registers that are implicitly popped by defs/clobbers.
    FPKills &= ~(STDefs | STClobbers);

    // Now we can rearrange the live registers to match what was requested.
    unsigned char STUsesArray[8];

    for (unsigned I = 0; I < NumSTUses; ++I)
      STUsesArray[I] = I;

    shuffleStackTop(STUsesArray, NumSTUses, Inst);
    LLVM_DEBUG({
      dbgs() << "Before asm: ";
      dumpStack();
    });

    // With the stack layout fixed, rewrite the FP registers.
    for (unsigned i = 0, e = MI.getNumOperands(); i != e; ++i) {
      MachineOperand &Op = MI.getOperand(i);
      if (!Op.isReg() || Op.getReg() < X86::FP0 || Op.getReg() > X86::FP6)
        continue;

      unsigned FPReg = getFPReg(Op);

      if (FRegIdx.count(i))
        // Operand with constraint "f".
        Op.setReg(getSTReg(FPReg));
      else
        // Operand with a single register class constraint ("t" or "u").
        Op.setReg(X86::ST0 + FPReg);
    }

    // Simulate the inline asm popping its inputs and pushing its outputs.
    StackTop -= NumSTPopped;

    for (unsigned i = 0; i < NumSTDefs; ++i)
      pushReg(NumSTDefs - i - 1);

    // If this asm kills any FP registers (is the last use of them) we must
    // explicitly emit pop instructions for them.  Do this now after the asm has
    // executed so that the ST(x) numbers are not off (which would happen if we
    // did this inline with operand rewriting).
    //
    // Note: this might be a non-optimal pop sequence.  We might be able to do
    // better by trying to pop in stack order or something.
    while (FPKills) {
      unsigned FPReg = countTrailingZeros(FPKills);
      if (isLive(FPReg))
        freeStackSlotAfter(Inst, FPReg);
      FPKills &= ~(1U << FPReg);
    }

    // Don't delete the inline asm!
    return;
  }
  }

  Inst = MBB->erase(Inst);  // Remove the pseudo instruction

  // We want to leave I pointing to the previous instruction, but what if we
  // just erased the first instruction?
  if (Inst == MBB->begin()) {
    LLVM_DEBUG(dbgs() << "Inserting dummy KILL\n");
    Inst = BuildMI(*MBB, Inst, DebugLoc(), TII->get(TargetOpcode::KILL));
  } else
    --Inst;
}

#endif


void T8xxStackPass::setKillFlags(MachineBasicBlock &MBB) const {
  const TargetRegisterInfo &TRI =
      *MBB.getParent()->getSubtarget().getRegisterInfo();
  LivePhysRegs LPR(TRI);

  LPR.addLiveOuts(MBB);

  for (MachineInstr &MI : llvm::reverse(MBB)) {
    if (MI.isDebugInstr())
      continue;

    std::bitset<8> Defs;
    SmallVector<MachineOperand *, 2> Uses;

    for (auto &MO : MI.operands()) {
      if (!MO.isReg())
        continue;

      unsigned Reg = MO.getReg() - T8xx::R1; // TODO: Check

      if (Reg >= 8)
        continue;

      if (MO.isDef()) {
        Defs.set(Reg);
        if (!LPR.contains(MO.getReg()))
          MO.setIsDead();
      } else
        Uses.push_back(&MO);
    }

    for (auto *MO : Uses)
      if (Defs.test(getFPReg(*MO)) || !LPR.contains(MO->getReg()))
        MO->setIsKill();

    LPR.stepBackward(MI);
  }
}

