//===-- T8xxInstrInfo.cpp - T8xx Instruction Information ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the T8xx implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "T8xxInstrInfo.h"
#include "T8xx.h"
#include "MCTargetDesc/T8xxMCExpr.h"
#include "T8xxMachineFunctionInfo.h"
#include "T8xxSubtarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/ScopeExit.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "t8xx-instr-info"

#define GET_INSTRINFO_CTOR_DTOR
#include "T8xxGenInstrInfo.inc"

// Pin the vtable to this file.
void T8xxInstrInfo::anchor() {}

T8xxInstrInfo::T8xxInstrInfo(T8xxSubtarget &ST)
    : T8xxGenInstrInfo(T8xx::ADJCALLSTACKDOWN, T8xx::ADJCALLSTACKUP), RI(),
      Subtarget(ST) {}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
Register T8xxInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  if (MI.getOpcode() == T8xx::LDL) {
    if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }
  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
Register T8xxInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                            int &FrameIndex) const {
  if (MI.getOpcode() == T8xx::STL) {
    if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
        MI.getOperand(1).getImm() == 0) {
      FrameIndex = MI.getOperand(0).getIndex();
      return MI.getOperand(2).getReg();
    }
  }
  return 0;
}




// The contents of values added to Cond are not examined outside of
// T8xxInstrInfo, giving us flexibility in what to push to it. For T8xx, we
// just push BranchOpcode
static void parseCondBranch(MachineInstr &LastInst, MachineBasicBlock *&Target,
                            SmallVectorImpl<MachineOperand> &Cond) {
  // Block ends with fall-through condbranch.
  assert(LastInst.getDesc().isConditionalBranch() &&
         "Unknown conditional branch");
  Cond.push_back(LastInst.getOperand(0));
  Target = LastInst.getOperand(1).getMBB();
}



//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//
//
/// AnalyzeBranch - Analyze the branching code at the end of MBB, returning
/// true if it cannot be understood (e.g. it's a switch dispatch or isn't
/// implemented for a target).  Upon success, this returns false and returns
/// with the following information in various cases:
///
/// 1. If this block ends with no branches (it just falls through to its succ)
///    just return false, leaving TBB/FBB null.
/// 2. If this block ends with only an unconditional branch, it sets TBB to be
///    the destination block.
/// 3. If this block ends with an conditional branch and it falls through to
///    an successor block, it sets TBB to be the branch destination block and a
///    list of operands that evaluate the condition. These
///    operands can be passed to other TargetInstrInfo methods to create new
///    branches.
/// 4. If this block ends with an conditional branch and an unconditional
///    block, it returns the 'true' destination in TBB, the 'false' destination
///    in FBB, and a list of operands that evaluate the condition. These
///    operands can be passed to other TargetInstrInfo methods to create new
///    branches.
///
/// Note that RemoveBranch and InsertBranch must be implemented to support
/// cases where this method returns success.
///
bool
T8xxInstrInfo::analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                            MachineBasicBlock *&FBB,
                            SmallVectorImpl<MachineOperand> &Cond,
                            bool AllowModify) const {
  LLVM_DEBUG({
      dbgs() << "T8xx::analyzeBranch\n";
      MBB.dump ();
    });

  TBB = FBB = nullptr;
  Cond.clear();

  // If the block has no terminators, it just falls into the block after it.
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end() || !isUnpredicatedTerminator(*I))
    return false;

  // Count the number of terminators and find the first unconditional or
  // indirect branch.
  MachineBasicBlock::iterator FirstUncondOrIndirectBr = MBB.end();
  int NumTerminators = 0;
  for (auto J = I.getReverse(); J != MBB.rend() && isUnpredicatedTerminator(*J);
       J++) {
    NumTerminators++;
    if (J->getDesc().isUnconditionalBranch() ||
        J->getDesc().isIndirectBranch()) {
      FirstUncondOrIndirectBr = J.getReverse();
    }
  }

  // If AllowModify is true, we can erase any terminators after
  // FirstUncondOrIndirectBR.
  if (AllowModify && FirstUncondOrIndirectBr != MBB.end()) {
    while (std::next(FirstUncondOrIndirectBr) != MBB.end()) {
      std::next(FirstUncondOrIndirectBr)->eraseFromParent();
      NumTerminators--;
    }
    I = FirstUncondOrIndirectBr;
  }

  // We can't handle blocks that end in an indirect branch.
  if (I->getDesc().isIndirectBranch())
    return true;

  // We can't handle Generic branch opcodes from Global ISel.
  if (I->isPreISelOpcode())
    return true;

  // We can't handle blocks with more than 2 terminators.
  if (NumTerminators > 2)
    return true;

  // Handle a single unconditional branch.
  if (NumTerminators == 1 && I->getDesc().isUnconditionalBranch()) {
    TBB = getBranchDestBlock(*I);
    return false;
  }

  // Handle a single conditional branch.
  if (NumTerminators == 1 && I->getDesc().isConditionalBranch()) {
    parseCondBranch(*I, TBB, Cond);
    return false;
  }

  // Handle a conditional branch followed by an unconditional branch.
  if (NumTerminators == 2 && std::prev(I)->getDesc().isConditionalBranch() &&
      I->getDesc().isUnconditionalBranch()) {
    parseCondBranch(*std::prev(I), TBB, Cond);
    FBB = getBranchDestBlock(*I);
    return false;
  }

  // Otherwise, we can't handle this.
  return true;
}

/// RemoveBranch - Remove the branching code at the end of the specific MBB.
/// This is only invoked in cases where AnalyzeBranch returns success. It
/// returns the number of instructions that were removed.
unsigned
T8xxInstrInfo::removeBranch(MachineBasicBlock &MBB,
			   int *BytesRemoved) const {
  LLVM_DEBUG(dbgs() << "T8xx::removeBranch\n");

  if (BytesRemoved)
    *BytesRemoved = 0;
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end())
    return 0;

  if (!I->getDesc().isUnconditionalBranch() &&
      !I->getDesc().isConditionalBranch())
    return 0;

  // Remove the branch.
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  I->eraseFromParent();

  I = MBB.end();

  if (I == MBB.begin())
    return 1;
  --I;
  if (!I->getDesc().isConditionalBranch())
    return 1;

  // Remove the branch.
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  I->eraseFromParent();
  return 2;
}

/// InsertBranch - Insert branch code into the end of the specified
/// MachineBasicBlock.  The operands to this method are the same as those
/// returned by AnalyzeBranch.  This is only invoked in cases where
/// AnalyzeBranch returns success. It returns the number of instructions
/// inserted.
///
/// It is also invoked by tail merging to add unconditional branches in
/// cases where AnalyzeBranch doesn't apply because there was no original
/// branch to analyze.  At least this much must be implemented, else tail
/// merging needs to be disabled.
unsigned T8xxInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL,
				    int *BytesAdded) const {
  unsigned NumInserted = 0;
  LLVM_DEBUG(dbgs() << "T8xx::insertBranch\n");

  if (BytesAdded)
    *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "T8xx branch conditions have one components!");

  // Unconditional branch.
  if (Cond.empty()) {
    MachineInstr &MI = *BuildMI(&MBB, DL, get(T8xx::JUMP)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Either a one or two-way conditional branch.
  MachineInstr &CondMI = *BuildMI(&MBB, DL, get(T8xx::CJ))
    .addReg(Cond[0].getReg())
    .addMBB(TBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(CondMI);

  // One-way conditional branch.
  if (!FBB)
    return 1;

  // Two-way conditional branch.
  MachineInstr &MI = *BuildMI(&MBB, DL, get(T8xx::JUMP)).addMBB(FBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(MI);
  return 2;
}



MachineBasicBlock *
T8xxInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  assert(MI.getDesc().isBranch() && "Unexpected opcode!");

  LLVM_DEBUG({
      MI.dump ();
    });

  // The branch target is always the last operand.
  int NumOp = MI.getNumExplicitOperands();
  return MI.getOperand(NumOp - 1).getMBB();
}


// ----

void T8xxInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 const DebugLoc &DL, MCRegister DestReg,
                                 MCRegister SrcReg, bool KillSrc,
                                 bool RenamableDest, bool RenamableSrc) const {
  /*
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  uint16_t hweSrcReg = TRI->getEncodingValue (SrcReg);
  uint16_t hweDstReg = TRI->getEncodingValue (DestReg);
  */
}

void T8xxInstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                    Register SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI,
		    Register VReg) const {
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addReg(SrcReg, getKillRegState(true))
    .addFrameIndex(FI).addImm(0);
}

void T8xxInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     Register DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI,
		     Register VReg) const {
  if (RC == &T8xx::ORegRegClass)
    BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL), DestReg).addFrameIndex(FI).addImm(0);
  else
    llvm_unreachable("Can't load this register from stack slot");
}


void T8xxInstrInfo::storeRegStack (MachineInstr &MI, const unsigned int OpNum,
				   const bool InsertPostMI) const
{
  DebugLoc DL = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  const MachineOperand::MachineOperandType MOT = MI.getOperand(OpNum).getType ();  // X

  LLVM_DEBUG(dbgs() << "storeRegStack TYPE: " << (int) MOT << "\n");

  switch (MOT)
    {
    case MachineOperand::MO_Register:
      {
	MachineBasicBlock::iterator MBBI = MI;
	if (InsertPostMI)
	  ++MBBI;
	BuildMI(MBB, MBBI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue(MI.getOperand(OpNum).getReg().asMCReg()));
      }
      break;

    default:
      llvm_unreachable("Failed in storeRegStack! Wrong destination operand type\n");
      break;
    }
}


bool T8xxInstrInfo::expandPostRAPseudo(MachineInstr &MI) const
{
  LLVM_DEBUG(dbgs()<< "expandPostRAPseudo Opcode: " << MI.getOpcode () << "\n");

  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  DebugLoc DL = MI.getDebugLoc();

  switch (MI.getOpcode())
  {
  default:
    return false;

  case T8xx::MoveLoad:
  case T8xx::MoveSEXTLoad:
  case T8xx::MoveZEXTLoad:
    {
      dbgs()<<"Expand MoveLoad\n";
      MI.dump ();
      int64_t FI = MI.getOperand(5).getImm ();
      BuildMI (MBB, MI, DL, get(T8xx::MOVE)).addReg(T8xx::AREG).
	addReg(T8xx::BREG).addReg(T8xx::CREG);
      BuildMI (MBB, MI, DL, get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(FI);

      if (MI.getOpcode() == T8xx::MoveSEXTLoad)
	{
	  BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addImm(32768);
	  BuildMI (MBB, MI, DL, get(T8xx::XWORD), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);
	}

      if (MI.getOpcode() == T8xx::MoveZEXTLoad)
	{
	  BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addImm(65535);
	  BuildMI (MBB, MI, DL, get(T8xx::AND), T8xx::AREG).addReg(T8xx::AREG).addReg(T8xx::BREG);
	}

      MBB.erase(MI);
      return true;
    }
    break;

    // This is a special instruction to introduce a way to get effective addresses
    // that are not aligned
  case T8xx::AddWptrImm:
  case T8xx::LDLPb:
    {
      int64_t rem = MI.getOperand(2).getImm () % 4;
      MCRegister DstReg = MI.getOperand(0).getReg ();
      MCRegister SrcReg = MI.getOperand(1).getReg ();
      BuildMI (MBB, MI, DL, get(T8xx::LDLP), DstReg).addReg(SrcReg).
	addImm((MI.getOperand(2).getImm () - rem) / 4);  // Divide by 4 to get offset in words
      if (rem != 0)
	{
	  MI.getOperand(2).setImm (MI.getOperand(2).getImm() - rem);
	  BuildMI (MBB, MI, DL, get(T8xx::ADC), T8xx::AREG).addReg(T8xx::AREG).addImm(rem);
	}
      MBB.erase(MI);
      return true;
    }
    break;

    // Pseudo instruction needs to be removed
  case T8xx::SELLOW:
  case T8xx::JOIN:
    MBB.erase (MI);
    break;

  case T8xx::RET:
    {
      BuildMI (MBB, MI, DL, get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, MI, DL, get(T8xx::GCALL)).addReg(T8xx::AREG);
      MBB.erase(MI);
      return true;
    }
    break;

    // Floating point comparisons
  case T8xx::FPOGTSN:
    {
      BuildMI (MBB, MI, DL, get(T8xx::FPORDEREDSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::CJ)).addReg(T8xx::AREG).addImm(2);
      BuildMI (MBB, MI, DL, get(T8xx::FPGTSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      MBB.erase(MI);
      return true;
    }
    break;
  case T8xx::FPOGTDB:
    {
      BuildMI (MBB, MI, DL, get(T8xx::FPORDEREDDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::CJ)).addReg(T8xx::AREG).addImm(2);
      BuildMI (MBB, MI, DL, get(T8xx::FPGTDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::FPOLESN:
    {
      BuildMI (MBB, MI, DL, get(T8xx::FPORDEREDSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::CJ)).addReg(T8xx::AREG).addImm(3);
      BuildMI (MBB, MI, DL, get(T8xx::FPGTSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
      MBB.erase(MI);
      return true;
    }
    break;
  case T8xx::FPOLEDB:
    {
      BuildMI (MBB, MI, DL, get(T8xx::FPORDEREDDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::CJ)).addReg(T8xx::AREG).addImm(3);
      BuildMI (MBB, MI, DL, get(T8xx::FPGTDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::FPOEQSN:
    {
      BuildMI (MBB, MI, DL, get(T8xx::FPORDEREDSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::CJ)).addReg(T8xx::AREG).addImm(2);
      BuildMI (MBB, MI, DL, get(T8xx::FPEQSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      MBB.erase(MI);
      return true;
    }
    break;
  case T8xx::FPOEQDB:
    {
      BuildMI (MBB, MI, DL, get(T8xx::FPORDEREDDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::CJ)).addReg(T8xx::AREG).addImm(2);
      BuildMI (MBB, MI, DL, get(T8xx::FPEQDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::FPONESN:
    {
      BuildMI (MBB, MI, DL, get(T8xx::FPORDEREDSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::CJ)).addReg(T8xx::AREG).addImm(3);
      BuildMI (MBB, MI, DL, get(T8xx::FPEQSN), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
      MBB.erase(MI);
      return true;
    }
    break;
  case T8xx::FPONEDB:
    {
      BuildMI (MBB, MI, DL, get(T8xx::FPORDEREDDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::CJ)).addReg(T8xx::AREG).addImm(3);
      BuildMI (MBB, MI, DL, get(T8xx::FPEQDB), T8xx::AREG).addReg(T8xx::FAREG).addReg(T8xx::FBREG);
      BuildMI (MBB, MI, DL, get(T8xx::EQC), T8xx::AREG).addReg(T8xx::AREG).addImm(0);
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::FPR64TOI32:
  case T8xx::FPR32TOI32:
    {
      // Note: Implementation uses WPtr + 0. This position should be kept free by
      // regular instructions
      BuildMI (MBB, MI, DL, get(T8xx::FPURZ), T8xx::FPRMREG);
      BuildMI (MBB, MI, DL, get((MI.getOpcode() == T8xx::FPR32TOI32) ? T8xx::FPINTSN : T8xx::FPINTDB), T8xx::FAREG).
	addReg(T8xx::FAREG).addReg(T8xx::FPRMREG);
      BuildMI (MBB, MI, DL, get(T8xx::LDLP), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      BuildMI (MBB, MI, DL, get(T8xx::FPSTNLI32)).addReg(T8xx::FAREG).addReg(T8xx::AREG);
      BuildMI (MBB, MI, DL, get(T8xx::LDL), T8xx::AREG).addReg(T8xx::WPTR).addImm(0);
      MBB.erase(MI);
      return true;
    }
    break;

    // Attempt to fix the jump table problem
  case T8xx::BRIND:
    {
      BuildMI (MBB, MI, DL, get(T8xx::GCALL)).addReg(T8xx::AREG);
      MBB.erase(MI);
    }
    break;


  case T8xx::CALL:
    {
      LLVM_DEBUG({
	  for (unsigned int i = 0; i < MI.getNumOperands (); ++i)
	    {
	      dbgs () << "CALL Op" << i << " " << MI.getOperand (i).getType () << "\n";
	      MI.getOperand (i).dump ();
	    }
	});

      // First OP is MO_GlobalAddress
      // Second OP is MO_RegisterMask
      // Third and Fourth are MO_Register

      // Load offset to global address into AREG and correct by bytecount of LDPI and GCALL
      if (MI.getOperand(0).isGlobal ())
	BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addGlobalAddress(MI.getOperand(0).getGlobal (), 0, T8xxMCExpr::VK_T8xx_GLOBAL);
      if (MI.getOperand(0).isSymbol ())
	BuildMI (MBB, MI, DL, get(T8xx::LDC), T8xx::AREG).addExternalSymbol(MI.getOperand(0).getSymbolName (), T8xxMCExpr::VK_T8xx_GLOBAL);

      BuildMI (MBB, MI, DL, get(T8xx::GCALL), T8xx::ABREG).addReg(T8xx::AREG);
      BuildMI (MBB, MI, DL, get(T8xx::REV), T8xx::AREG).addReg(T8xx::ABREG);
      MBB.erase(MI);
      return true;
    }
    break;
  }
  return false;
}
