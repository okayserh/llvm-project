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
#include "T8xxMachineFunctionInfo.h"
#include "T8xxSubtarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

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
unsigned T8xxInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  assert(0 && "Unimplemented");
  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned T8xxInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                            int &FrameIndex) const {
  assert(0 && "Unimplemented");
  return 0;
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
  bool HasCondBranch = false;
  TBB = nullptr;
  FBB = nullptr;

  printf ("T8xx::analyzeBranch\n");

  for (MachineInstr &MI : MBB) {
    if (MI.getOpcode() == T8xx::BRimm2) {
      MachineBasicBlock *TargetBB = MI.getOperand(0).getMBB();
      if (HasCondBranch) {
        FBB = TargetBB;
      } else {
        TBB = TargetBB;
      }
    } else if (MI.getOpcode() == T8xx::Bcc) {
      MachineBasicBlock *TargetBB = MI.getOperand(1).getMBB();
      TBB = TargetBB;
      Cond.push_back(MI.getOperand(0));
      HasCondBranch = true;
    }
  }
  return false;
}

/// RemoveBranch - Remove the branching code at the end of the specific MBB.
/// This is only invoked in cases where AnalyzeBranch returns success. It
/// returns the number of instructions that were removed.
unsigned
T8xxInstrInfo::removeBranch(MachineBasicBlock &MBB,
			   int *BytesRemoved) const {
  printf ("T8xx::removeBranch\n");

  if (MBB.empty())
    return 0;
  unsigned NumRemoved = 0;
  auto I = MBB.end();
  do {
    --I;
    unsigned Opc = I->getOpcode();
    if ((Opc == T8xx::BRimm2) || (Opc == T8xx::Bcc)) {
      auto ToDelete = I;
      ++I;
      MBB.erase(ToDelete);
      NumRemoved++;
    }
  } while (I != MBB.begin());
  return NumRemoved;
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
  printf ("T8xx::insertBranch\n");
  
  // Insert any conditional branch.
  // TOOD: addImm(Cond[0]) is jsut a quick fix! Needs to be checked anyway
  if (Cond.size() > 0) {
    BuildMI(MBB, MBB.end(), DL, get(T8xx::Bcc)).addImm(Cond[0].getImm()).addMBB(TBB);
    NumInserted++;
  }
  
  // Insert any unconditional branch.
  if (Cond.empty() || FBB) {
    BuildMI(MBB, MBB.end(), DL, get(T8xx::BRimm2)).addMBB(Cond.empty() ? TBB : FBB);
    NumInserted++;
  }
  return NumInserted;
}

// ---- 

void T8xxInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 const DebugLoc &DL, MCRegister DestReg,
                                 MCRegister SrcReg, bool KillSrc) const {
  /* 16 Register in Workspace approach */
  BuildMI(MBB, I, DL, get(T8xx::LDL)).addImm(SrcReg);
  BuildMI(MBB, I, DL, get(T8xx::STL)).addImm(DestReg);
}

void T8xxInstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                    Register SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {
  uint16_t hweSrcReg = TRI->getEncodingValue (SrcReg.asMCReg());

  /* With Stack relative to WPTR */
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL)).addImm(hweSrcReg);
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addImm(FI);
}

void T8xxInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     Register DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {
  uint16_t hweDestReg = TRI->getEncodingValue (DestReg.asMCReg());

  /* With Stack relative to WPTR */
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL)).addImm(FI);
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL)).addImm(hweDestReg);
}


bool T8xxInstrInfo::expandPostRAPseudo(MachineInstr &MI) const
{
  printf ("expandPostRAPseudo %i %i %i\n", MI.getOpcode (), T8xx::MOVimmr);

  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MachineRegisterInfo &MRI = MF->getRegInfo();
  const TargetRegisterInfo *TRI = MRI.getTargetRegisterInfo();

  switch (MI.getOpcode())
  {
  default:
    return false;

  case T8xx::STRi:
    {
      DebugLoc DL = MI.getDebugLoc();
    
      // Destination register (outs!?)
      const Register SrcReg = MI.getOperand(0).getReg();
      const Register AddBaseReg = MI.getOperand(1).getReg();  // TODO: Clarify whether that's always the frameindex?
      const unsigned FI = MI.getOperand(2).getImm();

      /* With stack relative to WPTR */
      BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue(SrcReg.asMCReg())); // SrcReg to BREG
      BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(FI);  // Stack Offset goes via OREG
      MBB.erase(MI);
      return true;
    }
    break;

  case T8xx::LDRi: {
    DebugLoc DL = MI.getDebugLoc();

    // Destination register (outs!?)
    const Register DstReg = MI.getOperand(0).getReg();
    const unsigned AddBaseReg = MI.getOperand(1).getReg();
    const unsigned FI = MI.getOperand(2).getImm();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(FI);
    BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue(DstReg.asMCReg()));
    MBB.erase(MI);
    return true;
  }
    break;

    // Currently never selected!?
  case T8xx::STR:
    {
    DebugLoc DL = MI.getDebugLoc();
    
    // Destination register (outs!?)
    const Register SrcReg = MI.getOperand(0).getReg();
    const unsigned AddBaseReg = MI.getOperand(1).getReg();  // TODO: Clarify whether that's always the frameindex?
    const unsigned FI = MI.getOperand(2).getImm();

    /* With stack relative to WPTR */
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue(SrcReg.asMCReg())); // SrcReg to BREG
    BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(FI);  // Stack Offset goes via OREG
    MBB.erase(MI);
    return true;
  }
    break;

    // Never selected?
  case T8xx::LDR: {
    DebugLoc DL = MI.getDebugLoc();

    // Destination register (outs!?)
    const Register DstReg = MI.getOperand(0).getReg();
    const unsigned AddBaseReg = MI.getOperand(1).getReg();
    const unsigned FI = MI.getOperand(2).getImm();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(FI);
    BuildMI(MBB, MI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue(DstReg.asMCReg()));
    MBB.erase(MI);
    return true;
  }
    break;

  case T8xx::ADDimmr:
    {
    DebugLoc DL = MI.getDebugLoc();

    // Destination register (outs!?)
    const Register DstReg = MI.getOperand(0).getReg();
    const Register SrcReg1 = MI.getOperand(1).getReg();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue(SrcReg1.asMCReg())); // SrcReg1 to BREG

    // Inserts after MI
    MachineBasicBlock::iterator MBBI = MI;
    BuildMI(MBB, ++MBBI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue(DstReg.asMCReg()));  // Stack Offset goes via OREG
    return true;
  }
    break;

  case T8xx::ADDregregop:
  case T8xx::SUBregregop:
  case T8xx::MULregregop:
  case T8xx::SHLregregop:
  case T8xx::SHRregregop:
    {
    DebugLoc DL = MI.getDebugLoc();

    // Destination register (outs!?)
    const Register DstReg = MI.getOperand(0).getReg();
    const Register SrcReg1 = MI.getOperand(1).getReg();
    const Register SrcReg2 = MI.getOperand(2).getReg();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue (SrcReg1.asMCReg())); // SrcReg1 to BREG
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue (SrcReg2.asMCReg())); // SrcReg2 to AREG

    // Inserts after MI
    MachineBasicBlock::iterator MBBI = MI;
    BuildMI(MBB, ++MBBI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue (DstReg.asMCReg()));
    return true;
  }
    break;


  case T8xx::ADDregimmop:
  case T8xx::SUBregimmop:
  case T8xx::MULregimmop:
  case T8xx::SHLregimmop:
  case T8xx::SHRregimmop:
    {
    DebugLoc DL = MI.getDebugLoc();

    // Destination register (outs!?)
    const Register DstReg = MI.getOperand(0).getReg();
    const Register SrcReg1 = MI.getOperand(1).getReg();
    const unsigned SrcImm2 = MI.getOperand(2).getImm();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addImm(TRI->getEncodingValue (SrcReg1.asMCReg())); // SrcReg1 to BREG
    BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(SrcImm2); // SrcImm2 to AREG

    // Inserts after MI
    MachineBasicBlock::iterator MBBI = MI;
    BuildMI(MBB, ++MBBI, DL, get(T8xx::STL)).addImm(TRI->getEncodingValue (DstReg.asMCReg()));
    return true;
  }
    break;

  }
}
