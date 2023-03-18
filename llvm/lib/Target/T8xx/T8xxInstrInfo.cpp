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


void T8xxInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 const DebugLoc &DL, MCRegister DestReg,
                                 MCRegister SrcReg, bool KillSrc) const {
  if (SrcReg == T8xx::WPTR)
    {
      BuildMI(MBB, I, DL, get(T8xx::LDNLP)).addImm(0);
      BuildMI(MBB, I, DL, get(T8xx::STL), DestReg);
    }
  else
    {
      if (DestReg == T8xx::WPTR)
	{
	  BuildMI(MBB, I, DL, get(T8xx::LDL)).addReg(SrcReg, getKillRegState(KillSrc));
	  BuildMI(MBB, I, DL, get(T8xx::GAJW));
	}
      else
	{
	  /* 16 Register in Workspace approach */
	  BuildMI(MBB, I, DL, get(T8xx::LDL)).addReg(SrcReg, getKillRegState(KillSrc));
	  BuildMI(MBB, I, DL, get(T8xx::STL), DestReg);
	}
    }
}

void T8xxInstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                    Register SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {

  /* With Stack relative to WPTR */
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL)).addReg(SrcReg);
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL_stack)).addImm(FI);
}

void T8xxInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     Register DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {

  /* With Stack relative to WPTR */
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDL_stack)).addImm(FI);
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STL),DestReg);
}


bool T8xxInstrInfo::expandPostRAPseudo(MachineInstr &MI) const
{
  printf ("expandPostRAPseudo %i %i %i\n", MI.getOpcode (), T8xx::MOVimmr);

  switch (MI.getOpcode())
  {
  default:
    return false;
  case T8xx::STR: {
    DebugLoc DL = MI.getDebugLoc();
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned SrcReg = MI.getOperand(0).getReg();
    const unsigned AddBaseReg = MI.getOperand(1).getReg();
    const unsigned FI = MI.getOperand(2).getImm();

    /* With stack relative to WPTR */
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addReg(SrcReg); // SrcReg to BREG
    BuildMI(MBB, MI, DL, get(T8xx::STL_stack)).addImm(FI);  // Stack Offset goes via OREG
    MBB.erase(MI);
    return true;
  }
    break;

  case T8xx::LDR: {
    DebugLoc DL = MI.getDebugLoc();
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned DstReg = MI.getOperand(0).getReg();
    const unsigned AddBaseReg = MI.getOperand(1).getReg();
    const unsigned FI = MI.getOperand(2).getImm();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL_stack)).addImm(FI);
    BuildMI(MBB, MI, DL, get(T8xx::STL),DstReg);
    MBB.erase(MI);
    return true;
  }
    break;

  case T8xx::ADDimmr:
    {
    DebugLoc DL = MI.getDebugLoc();
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned DstReg = MI.getOperand(0).getReg();
    const unsigned SrcReg1 = MI.getOperand(1).getReg();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addReg(SrcReg1); // SrcReg1 to BREG

    // Inserts after MI
    MachineBasicBlock::iterator MBBI = MI;
    BuildMI(MBB, ++MBBI, DL, get(T8xx::STL),DstReg);  // Stack Offset goes via OREG
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
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned DstReg = MI.getOperand(0).getReg();
    const unsigned SrcReg1 = MI.getOperand(1).getReg();
    const unsigned SrcReg2 = MI.getOperand(2).getReg();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addReg(SrcReg1); // SrcReg1 to BREG
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addReg(SrcReg2); // SrcReg2 to AREG

    // Inserts after MI
    MachineBasicBlock::iterator MBBI = MI;
    BuildMI(MBB, ++MBBI, DL, get(T8xx::STL),DstReg);  // Stack Offset goes via OREG
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
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned DstReg = MI.getOperand(0).getReg();
    const unsigned SrcReg1 = MI.getOperand(1).getReg();
    const unsigned SrcImm2 = MI.getOperand(2).getImm();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::LDL)).addReg(SrcReg1); // SrcReg1 to BREG
    BuildMI(MBB, MI, DL, get(T8xx::LDC)).addImm(SrcImm2); // SrcImm2 to AREG

    // Inserts after MI
    MachineBasicBlock::iterator MBBI = MI;
    BuildMI(MBB, ++MBBI, DL, get(T8xx::STL),DstReg);  // Stack Offset goes via OREG
    return true;
  }
    break;

  }
}
