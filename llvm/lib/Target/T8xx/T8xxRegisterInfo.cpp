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
T8xxRegisterInfo::T8xxRegisterInfo() : T8xxGenRegisterInfo(T8xx::AREG, 0, 0, T8xx::IPTR) {}

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
  //  const T8xxSubtarget &Subtarget = MF.getSubtarget<T8xxSubtarget>();

  return Reserved;
}

const TargetRegisterClass*
T8xxRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                      unsigned Kind) const {
  return &T8xx::ORegRegClass;
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

  bool bWordAlignedFO = false;
  
  printf ("eliminateFrameIndex  FI: %i  OpNum: %i   SPAdj: %i  StackSize %li\n", FI, FIOperandNum, SPAdj, MFI.getStackSize());
  MI.dump ();

  // Test if the new frame element has arrived
  // MFI.dump (MF);

  // Determine if we can eliminate the index from this kind of instruction.
  unsigned ImmOpIdx = 0;
  switch (MI.getOpcode()) {
  default:
    // Not supported yet.
    return false;
  case T8xx::STL:
  case T8xx::LDL:
  case T8xx::LDLP:
    bWordAlignedFO = true;
    [[fallthrough]];
    
  case T8xx::LDLPb:
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

  // Find start of first "frame" object (parameters are treated separately)
  unsigned obj_size = MFI.getStackSize ();
  for (int i = 0; i < MFI.getObjectIndexEnd (); ++i)
    if (MFI.getObjectSize (i) > 0)
      if (MFI.getObjectOffset (i) < obj_size)
	obj_size = MFI.getObjectOffset (i);
  printf ("Aligned Objects size = %i\n", obj_size);

  int Offset = 0;
  // FI < 0 = fixed stack objects (i.e. call parameters)
  if (FI < 0)
    {
      Offset = obj_size + fixed_obj_size - (MFI.getObjectOffset(FI) + 4) + ImmOp.getImm() ;
    }
  else
    {
      Offset = MFI.getObjectOffset(FI) - obj_size + ImmOp.getImm() ;
    }
  
  // Note: getObjectOffset is positive for the function parameter (0, 4, 8)
  // getObjectOffset is negative for the frame object (-4, -8, -12)
  
  // Note: This is set in "emit_prologue" (T8xxFrameLowering.cpp)
  Offset += MFI.getOffsetAdjustment ();
  
  printf ("eliminateFrameIndex  FI: %i Offset: %li Size: %li StackSize %li  ImmOp %li  ResOffset %i\n", FI, MFI.getObjectOffset(FI), MFI.getObjectSize(FI), MFI.getStackSize(), ImmOp.getImm(), Offset);

  // If FI is smaller 0, use the "spilled" WPtr
  if (FI < 0)
    {
      MachineBasicBlock *MBB = MI.getParent ();
      DebugLoc dl = MI.getDebugLoc();
      const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
      const T8xxMachineFunctionInfo &TMFI = *MF.getInfo<T8xxMachineFunctionInfo> ();
      
      // Directly replace with $areg = LDL $wptr, <xx>
      int WPtrOffset = MFI.getObjectOffset(TMFI.getWPtrSlot ()) - obj_size;
      WPtrOffset += MFI.getOffsetAdjustment ();
      BuildMI(*MBB, *II, dl, TII.get(T8xx::LDL), T8xx::AREG)
	.addReg(T8xx::WPTR)
	.addImm(WPtrOffset / 4);
      
      Offset = fixed_obj_size - (MFI.getObjectOffset(FI) + 4) + ImmOp.getImm();

      // LDLP -> TODO
      if (MI.getOpcode() == T8xx::LDLP)
	BuildMI(*MBB, *II, dl, TII.get(T8xx::ADC), T8xx::AREG)
	  .addReg(T8xx::AREG)
	  .addImm(Offset);

      // LDL
      if (MI.getOpcode() == T8xx::LDL)
	BuildMI(*MBB, *II, dl, TII.get(T8xx::LDNL), T8xx::AREG)
	  .addReg(T8xx::AREG)
	  .addImm(Offset / 4);

      // STL
      if (MI.getOpcode() == T8xx::STL)
	BuildMI(*MBB, *II, dl, TII.get(T8xx::STNL))
	  .addReg(T8xx::AREG)
	  .addReg(T8xx::BREG)
	  .addImm(Offset / 4);

      MI.eraseFromParent ();
    }
  else
    // Regular case for frame and parameters when no alignment > 4 is requested
    {
      FIOp.ChangeToRegister(T8xx::WPTR, false);

      if (bWordAlignedFO)
	{
	  assert ((Offset % 4 == 0) && "Framepointer offset must be word aligned!");
	  ImmOp.setImm(Offset / 4);
	}
      else
	ImmOp.setImm(Offset);
    }
      
  printf ("After eliminateFrameIndex\n");
  //  MI.dump ();

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
