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
#include "llvm/Support/Alignment.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "t8xx-register"

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
  return Reserved;
}

const TargetRegisterClass*
T8xxRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                      unsigned Kind) const {
  // The T8xx has only two types of registers. The integer
  // operand stack and the floating point operand stack.
  // Thus, the integer operand stack is used for pointers.
  return &T8xx::ORegRegClass;
}

bool
T8xxRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                       int SPAdj, unsigned FIOperandNum,
                                       RegScavenger *RS) const {
  MachineInstr &MI = *II;
  const MachineFunction &MF = *MI.getParent()->getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const T8xxMachineFunctionInfo &TMFI = *MF.getInfo<T8xxMachineFunctionInfo> ();

  MachineOperand &FIOp = MI.getOperand(FIOperandNum);
  int FI = FIOp.getIndex();

  // Needed to get infos about stack alignment
  const T8xxFrameLowering *TFL = getFrameLowering(MF);

  bool bWordAlignedFO = false;

  // Note: Calculation of stack offsets happens in PrologEpilogInserter
  LLVM_DEBUG({
      dbgs() << "eliminateFrameIndex  FI: " <<
	FI << "  OpNum: " <<
	FIOperandNum << "   SPAdj: " <<
	SPAdj << "  StackSize " <<
	MFI.getStackSize() << "\n";
      MI.dump ();
    });

  // Determine if we can eliminate the index from this kind of instruction.
  unsigned ImmOpIdx = 0;
  switch (MI.getOpcode()) {
  default:
    // Not supported yet.
    return false;
  case T8xx::MoveLoad:
  case T8xx::MoveSEXTLoad:
  case T8xx::MoveZEXTLoad:
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
    fixed_obj_size += alignTo (MFI.getObjectSize (i), TFL->getStackAlign ());

  // Find start of first "frame" object (parameters are treated separately)
  unsigned first_frame_pos = MFI.getStackSize ();
  for (int i = 0; i < MFI.getObjectIndexEnd (); ++i)
    if (MFI.getObjectSize (i) > 0)
      if (MFI.getObjectOffset (i) < first_frame_pos)
	first_frame_pos = MFI.getObjectOffset (i);

  // Align first frame pos to required alignment of MachineFunction
  Align MaxAlign = MFI.getMaxAlign();
  first_frame_pos = alignTo (first_frame_pos, MaxAlign);

  LLVM_DEBUG(dbgs() << "Aligned Objects size = " << first_frame_pos << "\n");

  // The fixed stack is positioned "above" the frame. If the stack
  // has an unaligned size (due to small objects like characters)
  // the size needs to be aligned.
  unsigned StackSizeAligned = alignTo (MFI.getStackSize (), TFL->getStackAlign ());

  int Offset = 0;

  // If FI is smaller 0, use the "spilled" WPtr
  if ((FI < 0) && MFI.shouldRealignStack())
    {
      MachineBasicBlock *MBB = MI.getParent ();
      DebugLoc dl = MI.getDebugLoc();
      const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
      const T8xxMachineFunctionInfo &TMFI = *MF.getInfo<T8xxMachineFunctionInfo> ();

      // Directly replace with $areg = LDL $wptr, <xx>
      int WPtrOffset = MFI.getObjectOffset(TMFI.getWPtrSlot ()) - first_frame_pos;
      WPtrOffset += MFI.getOffsetAdjustment ();
      BuildMI(*MBB, *II, dl, TII.get(T8xx::LDL), T8xx::AREG)
	.addReg(T8xx::WPTR)
	.addImm(WPtrOffset / 4);

      Offset = MFI.getObjectOffset(FI);
      if (TMFI.getVarArgsSaveSize () != 0)
	Offset += 4;

      // MoveLoad -> TODO
      // The MoveLoad instruction uses a workspace location
      // and therefore needs special treatment in this context as well.
      // Probable solution might include not introducing the initial "LDL"
      // before the MoveLoad instruction.
      // MOVE ....
      // LDL WPtrOffset (see above)
      // LDNL Offset
      // EXT/ZEXT/SEXT

      // LDLP -> TODO
      // Note: Unclear what the initial TODO was meant to be. Probably
      // "aligned" offsets should be included in the intrinsic offset?
      if (MI.getOpcode() == T8xx::LDLP)
	{
	  // Save an embarrassing "adc 0" when offset is zero
	  if (Offset != 0)
	    BuildMI(*MBB, *II, dl, TII.get(T8xx::ADC), T8xx::AREG)
	      .addReg(T8xx::AREG)
	      .addImm(Offset);
	}

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
      if (FI < 0)
	{
	  // FI < 0   -> fixed stack objects (i.e. call parameters)
	  Offset = (StackSizeAligned - fixed_obj_size) + MFI.getObjectOffset(FI) + ImmOp.getImm();
	  
	  // The + 4 are for one additional Workspace place to hold the return address when variable
	  // parameters are used.
	  // TODO: Replace 4 with a properly determined value representing one space on the workspace
	  if (TMFI.getVarArgsSaveSize () != 0)
	    Offset += 4;
	}
      else
	// FI >= 0  -> stack frame objects (i.e. function variables and temporary stack objects)
	Offset = MFI.getObjectOffset(FI) - first_frame_pos + ImmOp.getImm() ;

      // Add offset for WPTR Loc 0 (used internally)
      // Note: This is set in "emit_prologue" (T8xxFrameLowering.cpp)
      Offset += MFI.getOffsetAdjustment ();

      LLVM_DEBUG(dbgs() << "eliminateFrameIndex FI: " << FI <<
		 " Offset: " << MFI.getObjectOffset(FI) <<
		 " Size: " << MFI.getObjectSize(FI) <<
		 " StackSize " << MFI.getStackSize() <<
		 " ImmOp " << ImmOp.getImm() <<
		 " ResOffset " << Offset << "\n");

      // Emit changed instruction
      FIOp.ChangeToRegister(T8xx::WPTR, false);
      if (bWordAlignedFO)
	{
	  assert ((Offset % 4 == 0) && "Framepointer offset must be word aligned!");
	  ImmOp.setImm(Offset / 4);
	}
      else
	ImmOp.setImm(Offset);
    }

  LLVM_DEBUG(dbgs() << "After eliminateFrameIndex\n\n");

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
