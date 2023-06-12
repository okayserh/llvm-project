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


// Copied from old version
inline uint64_t RoundUpToAlignment(uint64_t Value, uint64_t Align,
                                   uint64_t Skew = 0) {
  Skew %= Align;
  return (Value + Align - 1 - Skew) / Align * Align + Skew;
}


bool
T8xxRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                       int SPAdj, unsigned FIOperandNum,
                                       RegScavenger *RS) const {
  MachineInstr &MI = *II;
  const MachineFunction &MF = *MI.getParent()->getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineOperand &FIOp = MI.getOperand(FIOperandNum);
  int FI = FIOp.getIndex();

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
	  // Note: only R1 to R15 as used stack registers
	  // are relevant as they need some reserved space on the stack
	  unsigned reg_id = (*ri)->getRegister(j).id();
	  if ((reg_id >= T8xx::R1) && (reg_id <= T8xx::R15))
	    {
	      if (!RI.reg_empty (reg_id))
		used_regs++;
	    }
	}
    }
  printf ("Function uses %i regs\n", used_regs);
  
  // Determine if we can eliminate the index from this kind of instruction.
  unsigned ImmOpIdx = 0;
  switch (MI.getOpcode()) {
  default:
    // Not supported yet.
    return false;
  case T8xx::LDRi8regop:
  case T8xx::LDRi16regop:
  case T8xx::LDRi32regop:
  case T8xx::LDRzi8regop:
  case T8xx::LDRzi16regop:
  case T8xx::LDRsi8regop:
  case T8xx::LDRsi16regop:

  case T8xx::STRi8regop:
  case T8xx::STRi16regop:
  case T8xx::STRi32regop:
  case T8xx::STRi8immop:
  case T8xx::STRi16immop:
  case T8xx::STRi32immop:

  case T8xx::LEA_ADDri:
  case T8xx::ADDmemmemop:
  case T8xx::SUBmemmemop:
  case T8xx::MULmemmemop:
  case T8xx::SHLmemmemop:
  case T8xx::SHRmemmemop:
  case T8xx::SDIVmemmemop:
  case T8xx::SREMmemmemop:
  case T8xx::UDIVmemmemop:
  case T8xx::UREMmemmemop:
  case T8xx::SRAmemmemop:
  case T8xx::ROTRmemmemop:
  case T8xx::ROTLmemmemop:
  case T8xx::XORmemmemop:
  case T8xx::ORmemmemop:
  case T8xx::ANDmemmemop:
    ImmOpIdx = FIOperandNum + 1;
    break;
  }

  // FIXME: check the size of offset.
  MachineOperand &ImmOp = MI.getOperand(ImmOpIdx);

  // Get the size of parameters on the stack
  unsigned fixed_obj_size = 0;
  for (int i = MFI.getObjectIndexBegin (); i < 0; ++i)
    fixed_obj_size += RoundUpToAlignment (MFI.getObjectSize (i), 4);
  printf ("Fixed objects size = %i\n", fixed_obj_size);
  fixed_obj_size = (fixed_obj_size + 3) / 4 * 4;
  printf ("Aligned Fixed objects size = %i\n", fixed_obj_size);

  /*
  unsigned obj_size = 0;
  for (int i = 0; i < MFI.getObjectIndexEnd (); ++i)    
    obj_size += MFI.getObjectSize (i) > 0 ? RoundUpToAlignment (MFI.getObjectSize (i), 4) : 0;
  printf ("Objects size = %i\n", obj_size);
  obj_size = (obj_size + 3) / 4 * 4;
  printf ("Aligned Objects size = %i\n", obj_size);
  */
  unsigned obj_size = 0;
  for (int i = 0; i < MFI.getObjectIndexEnd (); ++i)
    if (MFI.getObjectSize (i) > 0)
      if (-MFI.getObjectOffset (i) > obj_size)
	obj_size = -MFI.getObjectOffset (i);
  printf ("Aligned Objects size = %i\n", obj_size);
  
  int Offset = 0;
  // FI < 0 = fixed stack objects (i.e. call parameters)
  if (FI < 0)
    {
      Offset = obj_size + MFI.getObjectOffset(FI) + ImmOp.getImm() ;
    }
  else
    {
      Offset = obj_size + MFI.getObjectOffset(FI) + ImmOp.getImm() ;
    }
  
  // Note: getObjectOffset is positive for the function parameter (0, 4, 8)
  // getObjectOffset is negative for the frame object (-4, -8, -12)
  
  // Note: This is set in "emit_prologue" (T8xxFrameLowering.cpp)
  Offset += MFI.getOffsetAdjustment ();
  
  printf ("eliminateFrameIndex  FI: %i Offset: %i Size: %i StackSize %i  ImmOp %i  ResOffset %i\n", FI, MFI.getObjectOffset(FI), MFI.getObjectSize(FI), MFI.getStackSize(), ImmOp.getImm(), Offset);


  // Note: There was erroneous behavior in the initial version
  // Since the R15 was "used", the next call to eliminateFrameIndex
  // counted one additional used register, which led to
  // a double usage of certain stack positions.
  // Note: This error is back :-/. Presumably since WPTR is now
  // included as real register, the function also takes this up.
  FIOp.ChangeToRegister(T8xx::WPTR, false);
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
