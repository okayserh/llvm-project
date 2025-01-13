//===-- T8xxFrameLowering.cpp - T8xx Frame Information ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the T8xx implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "T8xxFrameLowering.h"
#include "T8xxInstrInfo.h"
#include "T8xxMachineFunctionInfo.h"
#include "T8xxSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

// Copied from old version
inline uint64_t RoundUpToAlignment(uint64_t Value, uint64_t Align,
                                   uint64_t Skew = 0) {
  Skew %= Align;
  return (Value + Align - 1 - Skew) / Align * Align + Skew;
}

static cl::opt<bool>
DisableLeafProc("disable-t8xx-leaf-proc",
                cl::init(false),
                cl::desc("Disable T8xx leaf procedure optimization."),
                cl::Hidden);

T8xxFrameLowering::T8xxFrameLowering(const T8xxSubtarget &ST)
  : TargetFrameLowering(TargetFrameLowering::StackGrowsDown,  // StackDir
			Align(4),  // StackAlignment
			0,      // LocalAreaOffset
			Align(4)) {}   // TransientRealignment

void T8xxFrameLowering::emitSPAdjustment(MachineFunction &MF,
                                          MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator MBBI,
                                          int NumBytes,
                                          unsigned ADDrr,
                                          unsigned ADDri) const {

  printf ("emitSPAdjustment\n");

  /* TODO: This was used for the SPARC to adjust the stack pointer register.
     Could possible be the WPtr register that needs adjustment.
  DebugLoc dl;
  const T8xxInstrInfo &TII =
      *static_cast<const T8xxInstrInfo *>(MF.getSubtarget().getInstrInfo());

  if (NumBytes >= -4096 && NumBytes < 4096) {
    BuildMI(MBB, MBBI, dl, TII.get(ADDri), T8xx::R6)
      .addReg(T8xx::R6).addImm(NumBytes);
    return;
  }
  */
}


uint64_t T8xxFrameLowering::computeStackSize(MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  // Get the size of parameters on the stack
  int64_t fixed_obj_size = 0;
  for (int i = MFI.getObjectIndexBegin (); i < 0; ++i)
    fixed_obj_size += RoundUpToAlignment (MFI.getObjectSize (i), getStackAlignment ());
  int64_t obj_size = 0;
  /* Old version, where the object size with stack alignment is used. Produces
     incorrect results, when the larger alignments are requested the LLVM code */
  for (int i = 0; i < MFI.getObjectIndexEnd (); ++i)    
    obj_size += MFI.getObjectSize (i) > 0 ?
      RoundUpToAlignment (MFI.getObjectSize (i), getStackAlignment ()) : 0;

  /* New version, Obj size is determined as the maximum negative index in the frame */
  for (int i = 0; i < MFI.getObjectIndexEnd (); ++i)
    if (MFI.getObjectSize (i) > 0)
      if (-MFI.getObjectOffset (i) > obj_size)
	obj_size = -MFI.getObjectOffset (i);
  
  return (obj_size + fixed_obj_size);
}


void T8xxFrameLowering::emitPrologue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  printf ("emitPrologue\n");
  
  // Compute the stack size, to determine if we need a prologue at all.
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  uint64_t StackSize = computeStackSize(MF);
  if (!StackSize) {
    return;
  }

  // Attempt to adjust stack offset
  /* Note: This is just a helper variable in the MFI object. */
  printf ("Current FI Offset = %i\n", MFI.getOffsetAdjustment ());
  // Note: The +1 is for R0, which is reserved for the return address
  MFI.setOffsetAdjustment (4);
  
  // Adjust the stack pointer.
  /* Save the return address on old stack position 0 */ 
  BuildMI(MBB, MBBI, dl, TII.get(T8xx::STL)).addReg(T8xx::AREG).addReg(T8xx::WPTR).addImm(0);

  /* Real adjustment via AJW */
  BuildMI(MBB, MBBI, dl, TII.get(T8xx::AJW))
    .addImm(-((StackSize / 4) + 1))
        .setMIFlag(MachineInstr::FrameSetup);
}

MachineBasicBlock::iterator T8xxFrameLowering::
eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const {
  if (!hasReservedCallFrame(MF)) {
    MachineInstr &MI = *I;
    int Size = MI.getOperand(0).getImm();
    if (MI.getOpcode() == T8xx::ADJCALLSTACKDOWN)
      Size = -Size;
  }
  return MBB.erase(I);
  //  return MBB.end ();
}


void T8xxFrameLowering::emitEpilogue(MachineFunction &MF,
                                  MachineBasicBlock &MBB) const {
  printf ("emitEpilogue\n");

  // Compute the stack size, to determine if we need an epilogue at all.
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  DebugLoc dl = MBBI->getDebugLoc();
  uint64_t StackSize = computeStackSize(MF);
  if (!StackSize) {
    return;
  }

  // First write down all registers
  MachineRegisterInfo &RI = MF.getRegInfo ();

  // Restore the stack pointer to what it was at the beginning of the function.
  /* Real stack adjustment */
  BuildMI(MBB, MBBI, dl, TII.get(T8xx::AJW))
    .addImm((StackSize / 4) + 1)
        .setMIFlag(MachineInstr::FrameSetup);

  printf ("emitEpilogue End\n");
}

bool T8xxFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  // Reserve call frame if there are no variable sized objects on the stack.
  return !MF.getFrameInfo().hasVarSizedObjects();
}

// hasFP - Return true if the specified function should have a dedicated frame
// pointer register.  This is true if the function has variable sized allocas or
// if frame pointer elimination is disabled.
bool T8xxFrameLowering::hasFPImpl(const MachineFunction &MF) const {
  const TargetRegisterInfo *RegInfo = MF.getSubtarget().getRegisterInfo();

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
         RegInfo->hasStackRealignment(MF) || MFI.hasVarSizedObjects() ||
         MFI.isFrameAddressTaken();
}

StackOffset
T8xxFrameLowering::getFrameIndexReference(const MachineFunction &MF, int FI,
                                           Register &FrameReg) const {
  const T8xxSubtarget &Subtarget = MF.getSubtarget<T8xxSubtarget>();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const T8xxRegisterInfo *RegInfo = Subtarget.getRegisterInfo();
  const T8xxMachineFunctionInfo *FuncInfo = MF.getInfo<T8xxMachineFunctionInfo>();
  bool isFixed = MFI.isFixedObjectIndex(FI);

  // Addressable stack objects are accessed using neg. offsets from
  // %fp, or positive offsets from %sp.
  bool UseFP;

  // T8xx uses FP-based references in general, even when "hasFP" is
  // false. That function is rather a misnomer, because %fp is
  // actually always available, unless isLeafProc.
  if (FuncInfo->isLeafProc()) {
    // If there's a leaf proc, all offsets need to be %sp-based,
    // because we haven't caused %fp to actually point to our frame.
    UseFP = false;
  } else if (isFixed) {
    // Otherwise, argument access should always use %fp.
    UseFP = true;
  } else if (RegInfo->hasStackRealignment(MF)) {
    // If there is dynamic stack realignment, all local object
    // references need to be via %sp, to take account of the
    // re-alignment.
    UseFP = false;
  } else {
    // Finally, default to using %fp.
    UseFP = true;
  }

  int64_t FrameOffset = MF.getFrameInfo().getObjectOffset(FI) +
      Subtarget.getStackPointerBias();

  if (UseFP) {
    FrameReg = RegInfo->getFrameRegister(MF);
    return StackOffset::getFixed(FrameOffset);
  } else {
    FrameReg = T8xx::WPTR; // %sp
    return StackOffset::getFixed(FrameOffset + MF.getFrameInfo().getStackSize());
  }
}


void T8xxFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                              BitVector &SavedRegs,
                                              RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  /*
  if (!DisableLeafProc && isLeafProc(MF)) {
    T8xxMachineFunctionInfo *MFI = MF.getInfo<T8xxMachineFunctionInfo>();
    MFI->setLeafProc(true);

    remapRegsForLeafProc(MF);
  }
  */
}
