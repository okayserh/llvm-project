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

// The first parameter is RAReg (MCRegisterInfo.h -> "Return address register"!?)
T8xxRegisterInfo::T8xxRegisterInfo() : T8xxGenRegisterInfo(T8xx::R0, 0, 0, T8xx::IPTR) {}

const MCPhysReg*
T8xxRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  // Note: These are the top 4 entries of the register stack.
  // Hence the adjustment of the Workspace pointer will
  // make these unaccessible by the short register accesses (ldl/stl).
  static const uint16_t CalleeSavedRegs[] = { 0 };
  return CalleeSavedRegs;
}

const uint32_t *
T8xxRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                        CallingConv::ID CC) const {
  // This is defined in CallingConv.td via
  // def CC_Save : CalleeSavedRegs<(add R4, R5, R6, R7, R8, R9)>;  
  return CC_Save_RegMask;
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
  //  return &T8xx::IntRegsRegClass;
  return &T8xx::IntRegsRegClass;
}


bool
T8xxRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                       int SPAdj, unsigned FIOperandNum,
                                       RegScavenger *RS) const {
  MachineInstr &MI = *II;
  const MachineFunction &MF = *MI.getParent()->getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineOperand &FIOp = MI.getOperand(FIOperandNum);
  unsigned FI = FIOp.getIndex();

  printf ("eliminateFrameIndex  FI: %i  OpNum: %i   SPAdj: %i  StackSize %i\n", FI, FIOperandNum, SPAdj, MFI.getStackSize());
  MI.dump ();

  // To be verified if this code is needed at this place, or if there's build in functionality for this
  const MachineRegisterInfo &RI = MF.getRegInfo ();
  const TargetRegisterInfo *TRI = RI.getTargetRegisterInfo ();
  int max_reg_id = 0;
  int used_regs = 0;
  
  for (TargetRegisterInfo::regclass_iterator ri = TRI->regclass_begin (); ri != TRI->regclass_end (); ++ri)
    {
      const TargetRegisterClass *rc = (*ri);
      for (int j = 0; j < (*ri)->getNumRegs (); ++j)
	{
	  unsigned reg_id = (*ri)->getRegister(j).id();
	  if (!RI.reg_empty (reg_id))
	    used_regs++;
	}
    }
  printf ("Function uses %i regs\n", used_regs);
  
  // Determine if we can eliminate the index from this kind of instruction.
  unsigned ImmOpIdx = 0;
  switch (MI.getOpcode()) {
  default:
    // Not supported yet.
    return false;
  case T8xx::LDRi:
  case T8xx::LDRi8:
  case T8xx::LDRzi8:
  case T8xx::LDRsi8:
  case T8xx::LDRi16:
  case T8xx::LDRzi16:
  case T8xx::LDRsi16:
  case T8xx::STRi:
  case T8xx::STRi8:
  case T8xx::STRi16:
  case T8xx::STRimm8:
    //  case T8xx::STRimm32:
  case T8xx::LEA_ADDri:
  case T8xx::ADDmemmemop:
  case T8xx::SUBmemmemop:
  case T8xx::MULmemmemop:
  case T8xx::SHLmemmemop:
  case T8xx::SHRmemmemop:
  case T8xx::XORmemmemop:
  case T8xx::ORmemmemop:
  case T8xx::ANDmemmemop:
    ImmOpIdx = FIOperandNum + 1;
    break;
  }

  // FIXME: check the size of offset.
  MachineOperand &ImmOp = MI.getOperand(ImmOpIdx);

  // TODO: Somehow the stack references seem not to be correct.
  /*
Function uses 3 regs
emitEpilogue End
eliminateFrameIndex  FI: 0  OpNum: 1   SPAdj: 0  StackSize 17
  STRi renamable $r1, %stack.0, 0 :: (store (s32) into %ir.3)
Function uses 3 regs
After eliminateFrameIndex
  STRi renamable $r1, $wptr, 25 :: (store (s32) into %ir.3)
eliminateFrameIndex  FI: 1  OpNum: 1   SPAdj: 0  StackSize 17
  STRi renamable $r2, %stack.1, 0 :: (store (s32) into %ir.4)
Function uses 3 regs
After eliminateFrameIndex
  STRi renamable $r2, $wptr, 21 :: (store (s32) into %ir.4)
eliminateFrameIndex  FI: 2  OpNum: 1   SPAdj: 0  StackSize 17
  STRi16 renamable $r2, %stack.2, 0 :: (store (s16) into %ir.5, align 4)
Function uses 3 regs
After eliminateFrameIndex
  STRi16 renamable $r2, $wptr, 17 :: (store (s16) into %ir.5, align 4)
eliminateFrameIndex  FI: 3  OpNum: 1   SPAdj: 0  StackSize 17
  STRi killed renamable $r3, %stack.3, 0 :: (store (s32) into %ir.6)
Function uses 3 regs
After eliminateFrameIndex
  STRi killed renamable $r3, $wptr, 13 :: (store (s32) into %ir.6)
eliminateFrameIndex  FI: 4  OpNum: 1   SPAdj: 0  StackSize 17
  STRi8 killed renamable $r2, %stack.4, 0 :: (store (s8) into %ir.7)
Function uses 3 regs
After eliminateFrameIndex
  STRi8 killed renamable $r2, $wptr, 12 :: (store (s8) into %ir.7)

Frame Objects:
  fi#0: size=4, align=4, at location [SP-4]  -> R1 (erste Parameter)   16
  fi#1: size=4, align=4, at location [SP-8]  -> R2 (zweiter Parameter) 20
  fi#2: size=4, align=4, at location [SP-12] -> short i   24
  fi#3: size=4, align=4, at location [SP-16] -> int buf   28
  fi#4: size=1, align=1, at location [SP-17] -> char abc  29

  12 + 17 = 29

  frame-setup AJW -36
  LDL 1
  STL 25
  LDL 2
  STL 21
  LDL 1
  LDC 17
  renamable $r3 = SHLregimmop renamable $r1, 17
  STL 3
  renamable $r3 = SRAregimmop killed renamable $r3, 16
  LDL 3
  LDL 2
  */
  
  //  int Offset = -MFI.getObjectOffset(FI) + used_regs * 4 + ImmOp.getImm() ;  // Instead of 4 = Sizeof(register)

  // This approach gets the direction right, but does not work when the
  // later objects are not properly aligned
  // int Offset = (MFI.getStackSize() + (used_regs + 1) * 4)+MFI.getObjectOffset(FI) + ImmOp.getImm() ;  // Instead of 4 = Sizeof(register)

  int Offset = -MFI.getObjectOffset(FI) -MFI.getObjectSize(FI) + (used_regs + 1) * 4 + ImmOp.getImm() ;  // Instead of 4 = Sizeof(register)

  // Note: There was erroneous behavior in the initial version
  // Since the R15 was "used", the next call to eliminateFrameIndex
  // counted one additional used register, which led to
  // a double usage of certain stack positions.
  FIOp.ChangeToRegister(T8xx::WPTR, false);  // TODO: Just a fix to make it compile
  ImmOp.setImm(Offset);
  
  printf ("After eliminateFrameIndex\n");
  MI.dump ();

  return false;
}

Register T8xxRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  return T8xx::WPTR;
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
