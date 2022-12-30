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
  /*
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::MOVrr), DestReg)
      .addReg(SrcReg, getKillRegState(KillSrc));
  */
  printf ("copyPhysReg, STL \n");
  MBB.dump ();

  auto LDL =  BuildMI(MBB, I, DL, get(T8xx::ldl));
  LDL.addReg(SrcReg, getKillRegState(KillSrc));

  printf ("STL \n");
  auto STL = BuildMI(MBB, I, DL, get(T8xx::stl), DestReg);

  MBB.dump ();
}

void T8xxInstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                    Register SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {
  /*
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::STR))
    .addReg(SrcReg, getKillRegState(isKill))
    .addFrameIndex(FI).addImm(0);
  */
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::ldl)).addReg(SrcReg); // SrcReg to BREG
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::ldl)).addReg(T8xx::R15); // Stack Addr to AREG
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::stnl)).addImm(FI);  // Stack Offset goes via OREG
}

void T8xxInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     Register DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {
  /*
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::LDR), DestReg)
      .addFrameIndex(FI).addImm(0);
  */
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::ldl)).addReg(T8xx::R15); // Stack Addr to AREG
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::ldnl)).addImm(FI);  // Offset via ORGE, Result is in AREG
  BuildMI(MBB, I, I->getDebugLoc(), get(T8xx::stl), DestReg);  // Save AREG to DstReg
}


bool T8xxInstrInfo::expandPostRAPseudo(MachineInstr &MI) const
{
  printf ("expandPostRAPseudo %i %i %i\n", MI.getOpcode (), T8xx::MOVimmr, T8xx::MOVrr);

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

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::ldl)).addReg(SrcReg); // SrcReg to BREG
    BuildMI(MBB, MI, DL, get(T8xx::ldl)).addReg(T8xx::R15); // Stack Addr to AREG
    BuildMI(MBB, MI, DL, get(T8xx::stnl)).addImm(FI);  // Stack Offset goes via OREG
    MBB.erase(MI);
    return true;
  }
    break;

  case T8xx::ADDregregop:
  case T8xx::SUBregregop:
  case T8xx::MULregregop:
    {
    DebugLoc DL = MI.getDebugLoc();
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned DstReg = MI.getOperand(0).getReg();
    const unsigned SrcReg1 = MI.getOperand(1).getReg();
    const unsigned SrcReg2 = MI.getOperand(2).getReg();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::ldl)).addReg(SrcReg1); // SrcReg1 to BREG
    BuildMI(MBB, MI, DL, get(T8xx::ldl)).addReg(SrcReg2); // SrcReg2 to AREG

    // Inserts after MI
    MachineBasicBlock::iterator MBBI = MI;
    BuildMI(MBB, ++MBBI, DL, get(T8xx::stl),DstReg);  // Stack Offset goes via OREG
    return true;
  }
    break;


  case T8xx::ADDregimmop:
  case T8xx::SUBregimmop:
  case T8xx::MULregimmop:
    {
    DebugLoc DL = MI.getDebugLoc();
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned DstReg = MI.getOperand(0).getReg();
    const unsigned SrcReg1 = MI.getOperand(1).getReg();
    const unsigned SrcImm2 = MI.getOperand(2).getImm();

    // BuildMI inserts before "MI"
    BuildMI(MBB, MI, DL, get(T8xx::ldc)).addImm(SrcImm2); // SrcReg1 to BREG
    BuildMI(MBB, MI, DL, get(T8xx::ldl)).addReg(SrcReg1); // SrcReg2 to AREG

    // Inserts after MI
    MachineBasicBlock::iterator MBBI = MI;
    BuildMI(MBB, ++MBBI, DL, get(T8xx::stl),DstReg);  // Stack Offset goes via OREG
    return true;
  }
    break;

    
    /*
  case T8xx::MOVimmr: {
    DebugLoc DL = MI.getDebugLoc();
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned DstReg = MI.getOperand(0).getReg();
    const bool DstIsDead = MI.getOperand(0).isDead();

    // Input operand (ins, immediate)
    const MachineOperand &MO = MI.getOperand(1);

    printf ("MOVimmr PostRAP\n");
    MI.dump();
    
    auto LDC = BuildMI(MBB, MI, DL, get(T8xx::ldc), DstReg);
    auto STL = BuildMI(MBB, MI, DL, get(T8xx::stl))
                    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
                    .addReg(DstReg);

    if (MO.isImm()) {
      const unsigned Imm = MO.getImm();
      const unsigned Lo16 = Imm & 0xffff;
      const unsigned Hi16 = (Imm >> 16) & 0xffff;
      LDC = LDC.addImm(Lo16);
      STL = STL.addImm(Hi16);
    } else {
      const GlobalValue *GV = MO.getGlobal();
      const unsigned TF = MO.getTargetFlags();
    }

    MBB.erase(MI);
    return true;
  }
*/
  case T8xx::MOVrr: {
    DebugLoc DL = MI.getDebugLoc();
    MachineBasicBlock &MBB = *MI.getParent();

    // Destination register (outs!?)
    const unsigned DstReg = MI.getOperand(0).getReg();
    const bool DstIsDead = MI.getOperand(0).isDead();

    const unsigned SrcReg = MI.getOperand(1).getReg();

    // Input operand (ins, immediate)
    const MachineOperand &MO = MI.getOperand(1);

    printf ("MOVimmr PostRAP\n");
    MI.dump();
    
    auto LDL = BuildMI(MBB, MI, DL, get(T8xx::ldl), SrcReg);
    auto STL = BuildMI(MBB, MI, DL, get(T8xx::stl), DstReg);

    MBB.erase(MI);
    return true;
  }
  }
}
