//===-- T8xxRegisterInfo.cpp - SPARC Register Information ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the SPARC implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "T8xxRegisterInfo.h"
#include "T8xx.h"
#include "T8xxMachineFunctionInfo.h"
#include "T8xxSubtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define GET_REGINFO_TARGET_DESC
#include "T8xxGenRegisterInfo.inc"

static cl::opt<bool>
ReserveAppRegisters("t8xx-reserve-app-registers", cl::Hidden, cl::init(false),
                    cl::desc("Reserve application registers (%g2-%g4)"));

T8xxRegisterInfo::T8xxRegisterInfo() : T8xxGenRegisterInfo(T8::R15) {}

const MCPhysReg*
T8xxRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  //  return CSR_SaveList;
}

const uint32_t *
T8xxRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                        CallingConv::ID CC) const {
  //  return CSR_RegMask;
}

const uint32_t*
T8xxRegisterInfo::getRTCallPreservedMask(CallingConv::ID CC) const {
  //  return RTCSR_RegMask;
}

BitVector T8xxRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  const T8xxSubtarget &Subtarget = MF.getSubtarget<T8xxSubtarget>();

  return Reserved;
}

const TargetRegisterClass*
T8xxRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                      unsigned Kind) const {
  const T8xxSubtarget &Subtarget = MF.getSubtarget<T8xxSubtarget>();
  return &T8::IntRegsRegClass;
}

static void replaceFI(MachineFunction &MF, MachineBasicBlock::iterator II,
                      MachineInstr &MI, const DebugLoc &dl,
                      unsigned FIOperandNum, int Offset, unsigned FramePtr) {
  /*
  // Replace frame index with a frame pointer reference.
  if (Offset >= -4096 && Offset <= 4095) {
    // If the offset is small enough to fit in the immediate field, directly
    // encode it.
    MI.getOperand(FIOperandNum).ChangeToRegister(FramePtr, false);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
    return;
  }

  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();

  // FIXME: it would be better to scavenge a register here instead of
  // reserving G1 all of the time.
  if (Offset >= 0) {
    // Emit nonnegaive immediates with sethi + or.
    // sethi %hi(Offset), %g1
    // add %g1, %fp, %g1
    // Insert G1+%lo(offset) into the user.
    BuildMI(*MI.getParent(), II, dl, TII.get(T8::SETHIi), T8::G1)
      .addImm(HI22(Offset));


    // Emit G1 = G1 + I6
    BuildMI(*MI.getParent(), II, dl, TII.get(T8::ADDrr), T8::G1).addReg(T8::G1)
      .addReg(FramePtr);
    // Insert: G1+%lo(offset) into the user.
    MI.getOperand(FIOperandNum).ChangeToRegister(T8::G1, false);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(LO10(Offset));
    return;
  }

  // Emit Negative numbers with sethi + xor
  // sethi %hix(Offset), %g1
  // xor  %g1, %lox(offset), %g1
  // add %g1, %fp, %g1
  // Insert: G1 + 0 into the user.
  BuildMI(*MI.getParent(), II, dl, TII.get(T8::SETHIi), T8::G1)
    .addImm(HIX22(Offset));
  BuildMI(*MI.getParent(), II, dl, TII.get(T8::XORri), T8::G1)
    .addReg(T8::G1).addImm(LOX10(Offset));

  BuildMI(*MI.getParent(), II, dl, TII.get(T8::ADDrr), T8::G1).addReg(T8::G1)
    .addReg(FramePtr);
  // Insert: G1+%lo(offset) into the user.
  MI.getOperand(FIOperandNum).ChangeToRegister(T8::G1, false);
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(0);
  */
}


bool
T8xxRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                       int SPAdj, unsigned FIOperandNum,
                                       RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected");
  
  MachineInstr &MI = *II;
  DebugLoc dl = MI.getDebugLoc();
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  MachineFunction &MF = *MI.getParent()->getParent();
  const T8xxSubtarget &Subtarget = MF.getSubtarget<T8xxSubtarget>();
  const T8xxFrameLowering *TFI = getFrameLowering(MF);

  Register FrameReg;
  int Offset;
  Offset = TFI->getFrameIndexReference(MF, FrameIndex, FrameReg).getFixed();

  Offset += MI.getOperand(FIOperandNum + 1).getImm();

  /*
  if (!Subtarget.isV9() || !Subtarget.hasHardQuad()) {
    if (MI.getOpcode() == T8::STQFri) {
      const TargetInstrInfo &TII = *Subtarget.getInstrInfo();
      Register SrcReg = MI.getOperand(2).getReg();
      Register SrcEvenReg = getSubReg(SrcReg, T8::sub_even64);
      Register SrcOddReg = getSubReg(SrcReg, T8::sub_odd64);
      MachineInstr *StMI =
        BuildMI(*MI.getParent(), II, dl, TII.get(T8::STDFri))
        .addReg(FrameReg).addImm(0).addReg(SrcEvenReg);
      replaceFI(MF, *StMI, *StMI, dl, 0, Offset, FrameReg);
      MI.setDesc(TII.get(T8::STDFri));
      MI.getOperand(2).setReg(SrcOddReg);
      Offset += 8;
    } else if (MI.getOpcode() == T8::LDQFri) {
      const TargetInstrInfo &TII = *Subtarget.getInstrInfo();
      Register DestReg = MI.getOperand(0).getReg();
      Register DestEvenReg = getSubReg(DestReg, T8::sub_even64);
      Register DestOddReg = getSubReg(DestReg, T8::sub_odd64);
      MachineInstr *LdMI =
        BuildMI(*MI.getParent(), II, dl, TII.get(T8::LDDFri), DestEvenReg)
        .addReg(FrameReg).addImm(0);
      replaceFI(MF, *LdMI, *LdMI, dl, 1, Offset, FrameReg);

      MI.setDesc(TII.get(T8::LDDFri));
      MI.getOperand(0).setReg(DestOddReg);
      Offset += 8;
    }
  }
  */
  replaceFI(MF, II, MI, dl, FIOperandNum, Offset, FrameReg);
  // replaceFI never removes II
  return false;
}

Register T8xxRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  return T8::R6;
}

// T8xx has no architectural need for stack realignment support,
// except that LLVM unfortunately currently implements overaligned
// stack objects by depending upon stack realignment support.
// If that ever changes, this can probably be deleted.
bool T8xxRegisterInfo::canRealignStack(const MachineFunction &MF) const {
  if (!TargetRegisterInfo::canRealignStack(MF))
    return false;

  // T8xx always has a fixed frame pointer register, so don't need to
  // worry about needing to reserve it. [even if we don't have a frame
  // pointer for our frame, it still cannot be used for other things,
  // or register window traps will be SADNESS.]

  // If there's a reserved call frame, we can use SP to access locals.
  if (getFrameLowering(MF)->hasReservedCallFrame(MF))
    return true;

  // Otherwise, we'd need a base pointer, but those aren't implemented
  // for SPARC at the moment.

  return false;
}
