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
    : T8xxGenInstrInfo(T8::ADJCALLSTACKDOWN, T8::ADJCALLSTACKUP), RI(),
      Subtarget(ST) {}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned T8xxInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned T8xxInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                            int &FrameIndex) const {
  return 0;
}


void T8xxInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 const DebugLoc &DL, MCRegister DestReg,
                                 MCRegister SrcReg, bool KillSrc) const {
  /*
  unsigned numSubRegs = 0;
  unsigned movOpc     = 0;
  const unsigned *subRegIdx = nullptr;
  bool ExtraG0 = false;

  const unsigned DW_SubRegsIdx[]  = { T8::sub_even, T8::sub_odd };
  const unsigned DFP_FP_SubRegsIdx[]  = { T8::sub_even, T8::sub_odd };
  const unsigned QFP_DFP_SubRegsIdx[] = { T8::sub_even64, T8::sub_odd64 };
  const unsigned QFP_FP_SubRegsIdx[]  = { T8::sub_even, T8::sub_odd,
                                          T8::sub_odd64_then_sub_even,
                                          T8::sub_odd64_then_sub_odd };

  if (T8::IntRegsRegClass.contains(DestReg, SrcReg))
    BuildMI(MBB, I, DL, get(T8::ORrr), DestReg).addReg(T8::G0)
      .addReg(SrcReg, getKillRegState(KillSrc));
  else if (T8::IntPairRegClass.contains(DestReg, SrcReg)) {
    subRegIdx  = DW_SubRegsIdx;
    numSubRegs = 2;
    movOpc     = T8::ORrr;
    ExtraG0 = true;
  } else if (T8::FPRegsRegClass.contains(DestReg, SrcReg))
    BuildMI(MBB, I, DL, get(T8::FMOVS), DestReg)
      .addReg(SrcReg, getKillRegState(KillSrc));
  else if (T8::DFPRegsRegClass.contains(DestReg, SrcReg)) {
    if (Subtarget.isV9()) {
      BuildMI(MBB, I, DL, get(T8::FMOVD), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
    } else {
      // Use two FMOVS instructions.
      subRegIdx  = DFP_FP_SubRegsIdx;
      numSubRegs = 2;
      movOpc     = T8::FMOVS;
    }
  } else if (T8::QFPRegsRegClass.contains(DestReg, SrcReg)) {
    if (Subtarget.isV9()) {
      if (Subtarget.hasHardQuad()) {
        BuildMI(MBB, I, DL, get(T8::FMOVQ), DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
      } else {
        // Use two FMOVD instructions.
        subRegIdx  = QFP_DFP_SubRegsIdx;
        numSubRegs = 2;
        movOpc     = T8::FMOVD;
      }
    } else {
      // Use four FMOVS instructions.
      subRegIdx  = QFP_FP_SubRegsIdx;
      numSubRegs = 4;
      movOpc     = T8::FMOVS;
    }
  } else if (T8::ASRRegsRegClass.contains(DestReg) &&
             T8::IntRegsRegClass.contains(SrcReg)) {
    BuildMI(MBB, I, DL, get(T8::WRASRrr), DestReg)
        .addReg(T8::G0)
        .addReg(SrcReg, getKillRegState(KillSrc));
  } else if (T8::IntRegsRegClass.contains(DestReg) &&
             T8::ASRRegsRegClass.contains(SrcReg)) {
    BuildMI(MBB, I, DL, get(T8::RDASR), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
  } else
    llvm_unreachable("Impossible reg-to-reg copy");

  if (numSubRegs == 0 || subRegIdx == nullptr || movOpc == 0)
    return;

  const TargetRegisterInfo *TRI = &getRegisterInfo();
  MachineInstr *MovMI = nullptr;

  for (unsigned i = 0; i != numSubRegs; ++i) {
    Register Dst = TRI->getSubReg(DestReg, subRegIdx[i]);
    Register Src = TRI->getSubReg(SrcReg, subRegIdx[i]);
    assert(Dst && Src && "Bad sub-register");

    MachineInstrBuilder MIB = BuildMI(MBB, I, DL, get(movOpc), Dst);
    if (ExtraG0)
      MIB.addReg(T8::G0);
    MIB.addReg(Src);
    MovMI = MIB.getInstr();
  }
  // Add implicit super-register defs and kills to the last MovMI.
  MovMI->addRegisterDefined(DestReg, TRI);
  if (KillSrc)
    MovMI->addRegisterKilled(SrcReg, TRI);
  */
}

void T8xxInstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                    Register SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {
  /*
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();

  MachineFunction *MF = MBB.getParent();
  const MachineFrameInfo &MFI = MF->getFrameInfo();
  MachineMemOperand *MMO = MF->getMachineMemOperand(
      MachinePointerInfo::getFixedStack(*MF, FI), MachineMemOperand::MOStore,
      MFI.getObjectSize(FI), MFI.getObjectAlign(FI));

  // On the order of operands here: think "[FrameIdx + 0] = SrcReg".
  if (RC == &T8::I64RegsRegClass)
    BuildMI(MBB, I, DL, get(T8::STXri)).addFrameIndex(FI).addImm(0)
      .addReg(SrcReg, getKillRegState(isKill)).addMemOperand(MMO);
  else if (RC == &T8::IntRegsRegClass)
    BuildMI(MBB, I, DL, get(T8::STri)).addFrameIndex(FI).addImm(0)
      .addReg(SrcReg, getKillRegState(isKill)).addMemOperand(MMO);
  else if (RC == &T8::IntPairRegClass)
    BuildMI(MBB, I, DL, get(T8::STDri)).addFrameIndex(FI).addImm(0)
      .addReg(SrcReg, getKillRegState(isKill)).addMemOperand(MMO);
  else if (RC == &T8::FPRegsRegClass)
    BuildMI(MBB, I, DL, get(T8::STFri)).addFrameIndex(FI).addImm(0)
      .addReg(SrcReg,  getKillRegState(isKill)).addMemOperand(MMO);
  else if (T8::DFPRegsRegClass.hasSubClassEq(RC))
    BuildMI(MBB, I, DL, get(T8::STDFri)).addFrameIndex(FI).addImm(0)
      .addReg(SrcReg,  getKillRegState(isKill)).addMemOperand(MMO);
  else if (T8::QFPRegsRegClass.hasSubClassEq(RC))
    // Use STQFri irrespective of its legality. If STQ is not legal, it will be
    // lowered into two STDs in eliminateFrameIndex.
    BuildMI(MBB, I, DL, get(T8::STQFri)).addFrameIndex(FI).addImm(0)
      .addReg(SrcReg,  getKillRegState(isKill)).addMemOperand(MMO);
  else
    llvm_unreachable("Can't store this register to stack slot");
  */
}

void T8xxInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     Register DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {
  /*
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();

  MachineFunction *MF = MBB.getParent();
  const MachineFrameInfo &MFI = MF->getFrameInfo();
  MachineMemOperand *MMO = MF->getMachineMemOperand(
      MachinePointerInfo::getFixedStack(*MF, FI), MachineMemOperand::MOLoad,
      MFI.getObjectSize(FI), MFI.getObjectAlign(FI));

  if (RC == &T8::I64RegsRegClass)
    BuildMI(MBB, I, DL, get(T8::LDXri), DestReg).addFrameIndex(FI).addImm(0)
      .addMemOperand(MMO);
  else if (RC == &T8::IntRegsRegClass)
    BuildMI(MBB, I, DL, get(T8::LDri), DestReg).addFrameIndex(FI).addImm(0)
      .addMemOperand(MMO);
  else if (RC == &T8::IntPairRegClass)
    BuildMI(MBB, I, DL, get(T8::LDDri), DestReg).addFrameIndex(FI).addImm(0)
      .addMemOperand(MMO);
  else if (RC == &T8::FPRegsRegClass)
    BuildMI(MBB, I, DL, get(T8::LDFri), DestReg).addFrameIndex(FI).addImm(0)
      .addMemOperand(MMO);
  else if (T8::DFPRegsRegClass.hasSubClassEq(RC))
    BuildMI(MBB, I, DL, get(T8::LDDFri), DestReg).addFrameIndex(FI).addImm(0)
      .addMemOperand(MMO);
  else if (T8::QFPRegsRegClass.hasSubClassEq(RC))
    // Use LDQFri irrespective of its legality. If LDQ is not legal, it will be
    // lowered into two LDDs in eliminateFrameIndex.
    BuildMI(MBB, I, DL, get(T8::LDQFri), DestReg).addFrameIndex(FI).addImm(0)
      .addMemOperand(MMO);
  else
    llvm_unreachable("Can't load this register from stack slot");
  */
}

Register T8xxInstrInfo::getGlobalBaseReg(MachineFunction *MF) const {
  /*
  T8xxMachineFunctionInfo *T8xxFI = MF->getInfo<T8xxMachineFunctionInfo>();
  Register GlobalBaseReg = T8xxFI->getGlobalBaseReg();
  if (GlobalBaseReg)
    return GlobalBaseReg;

  // Insert the set of GlobalBaseReg into the first MBB of the function
  MachineBasicBlock &FirstMBB = MF->front();
  MachineBasicBlock::iterator MBBI = FirstMBB.begin();
  MachineRegisterInfo &RegInfo = MF->getRegInfo();

  const TargetRegisterClass *PtrRC =
    Subtarget.is64Bit() ? &T8::I64RegsRegClass : &T8::IntRegsRegClass;
  GlobalBaseReg = RegInfo.createVirtualRegister(PtrRC);

  DebugLoc dl;

  BuildMI(FirstMBB, MBBI, dl, get(T8::GETPCX), GlobalBaseReg);
  T8xxFI->setGlobalBaseReg(GlobalBaseReg);
  return GlobalBaseReg;
  */
}

bool T8xxInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  /*
  switch (MI.getOpcode()) {
  case TargetOpcode::LOAD_STACK_GUARD: {
    assert(Subtarget.isTargetLinux() &&
           "Only Linux target is expected to contain LOAD_STACK_GUARD");
    // offsetof(tcbhead_t, stack_guard) from sysdeps/sparc/nptl/tls.h in glibc.
    const int64_t Offset = Subtarget.is64Bit() ? 0x28 : 0x14;
    MI.setDesc(get(Subtarget.is64Bit() ? T8::LDXri : T8::LDri));
    MachineInstrBuilder(*MI.getParent()->getParent(), MI)
        .addReg(T8::G7)
        .addImm(Offset);
    return true;
  }
  }
  return false;
  */
}
