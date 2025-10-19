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
#include "llvm/Support/Alignment.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

#define DEBUG_TYPE "t8xx-frame-lowering"


// Current stack implementation at T8xx
// |                                  |  Higher address
// |----------------------------------|
// |                                  |
// | IPtr when function was called    |  WPtr+0 (WPtr at function entry and exit)
// |                                  |
// |----------------------------------|
// |                                  |
// | arguments passed on the stack    |
// |                                  |  WPtr+n+1
// |----------------------------------|
// |                                  |  WPtr+n
// | local variables of fixed size    |
// | including spill slots            |  WPtr+1
// |----------------------------------|
// |                                  |
// |                                  |  (WPtr during function execution), 0 reserved
// |----------------------------------|

// TODO: Idea is to pass function parameters in a separate stack
// Idea 2: Adjust workspace before function call and
// load WPtr to BReg
// Within function, BReg is then saved in aligned frame and used to access
// the function parameters, if needed.


static cl::opt<bool>
DisableLeafProc("disable-t8xx-leaf-proc",
                cl::init(false),
                cl::desc("Disable T8xx leaf procedure optimization."),
                cl::Hidden);

T8xxFrameLowering::T8xxFrameLowering(const T8xxSubtarget &ST)
  : TargetFrameLowering(TargetFrameLowering::StackGrowsUp,  // StackDir
			Align(4),  // StackAlignment
			0,      // LocalAreaOffset
			Align(4)) {}   // TransientRealignment

void T8xxFrameLowering::emitSPAdjustment(MachineFunction &MF,
                                          MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator MBBI,
                                          int NumBytes,
                                          unsigned ADDrr,
                                          unsigned ADDri) const
{
  LLVM_DEBUG(dbgs() << "emitSPAdjustment \n");
}


uint64_t T8xxFrameLowering::computeParameterSize(MachineFunction &MF) const
{
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  // Get the size of parameters on the stack
  int64_t fixed_obj_size = 0;
  for (int i = MFI.getObjectIndexBegin (); i < 0; ++i)
    fixed_obj_size += alignTo (MFI.getObjectSize (i), getStackAlignment ());

  return ((uint64_t) fixed_obj_size);
}


// If an alignment > 4 is required, the original WPtr is
// reduced such that sufficient space is reserved on the stack.
// However, the aligned FramePointer is stored in a newly
// introduced spill space on the stack.
void T8xxFrameLowering::spillFPBP(MachineFunction &MF) const
{
  MachineFrameInfo &MFI = MF.getFrameInfo();
  T8xxMachineFunctionInfo *TMFI = MF.getInfo<T8xxMachineFunctionInfo> ();

  if (MFI.shouldRealignStack ())
    TMFI->setWPtrSlot (MFI.CreateSpillStackObject (4, Align(4)));
}


void T8xxFrameLowering::emitPrologue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  LLVM_DEBUG(dbgs() << "emitPrologue\n");

  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc dl = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  T8xxMachineFunctionInfo &TMFI = *MF.getInfo<T8xxMachineFunctionInfo> ();

  // Debugging output. Print current frame info
  LLVM_DEBUG ({
      MFI.dump (MF);
    });

  // Save the return address on old stack position 0
  // Note: This is always needed! Otherwise, the function does not know where
  // to return to.
  BuildMI(MBB, MBBI, dl, TII.get(T8xx::STL)).addReg(T8xx::AREG).addReg(T8xx::WPTR).addImm(0);

  // Dynamic stack realignment
  Align MaxAlign = MFI.getMaxAlign();

  LLVM_DEBUG (dbgs() << "Requested Alignment " << MaxAlign.value () << "\n");

  // Compute the stack size, to determine if we need a prologue at all.
  uint64_t FixedStackSize = computeParameterSize (MF);
  uint64_t StackSize = alignTo (MFI.getStackSize (), getStackAlign ());
  uint64_t OffsetAdj = MaxAlign.value ();

  LLVM_DEBUG (dbgs() << "Fixed Stack " <<
	      FixedStackSize << "   Stack " <<
	      StackSize << "   OffsetAdj " <<
	      OffsetAdj << "   TMFI Var Arg Size " <<
	      TMFI.getVarArgsSaveSize () << "\n");

  // If not stack alignment is needed, skip rest of prologue
  if ((FixedStackSize + StackSize) == 0) {
    return;
  }

  // Attempt to adjust stack offset
  /* Note: This is just a helper variable in the MFI object. */
  LLVM_DEBUG (dbgs() << "Current FI Offset = " << MFI.getOffsetAdjustment () << "\n");

  // Note: Stack position 0 may be used by some Transputer internals
  // Hence do not use that. However, when alignments other than the natural
  // 4 bytes are used, adjust the offset accordingly.
  // TODO: The current approach is rather wasteful with stack space.
  // Maybe the required stack slot 0 can be already included in the
  // calculation of the aligned workspace pointer?

  MFI.setOffsetAdjustment (MaxAlign.value ());

  // Adjust the stack pointer.

  // Now some dynamic alignment would be needed if the requested alignment is above 4 bytes
  if (MFI.shouldRealignStack ())
    {
      // Dynamic realignment
      // Adjust WPtr by required space for parameters
      // Note: For VarArgs, the WPtr is already adjusted before the function call
      // Therefore, it points correctly to the function parameter area and no further
      // adjustment is necessary.
      if (TMFI.getVarArgsSaveSize () == 0)
	{
	  BuildMI(MBB, MBBI, dl, TII.get(T8xx::AJW))
	    .addImm(-(FixedStackSize / 4))
	    .setMIFlag(MachineInstr::FrameSetup);
	}

      // Now adjust WPtr by required space for frame and add alignment as required
      // Start with WPtr in AReg
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::LDLP), T8xx::AREG)
	.addReg(T8xx::WPTR)
	.addImm(0)
        .setMIFlag(MachineInstr::FrameSetup);

      // Subtract required space
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::ADC), T8xx::AREG)
	.addReg(T8xx::AREG)
	.addImm(-((StackSize - FixedStackSize) + OffsetAdj))  // One additional space is required to avoid conflict with Parameters
        .setMIFlag(MachineInstr::FrameSetup);

      // And with 11111100 (where the number of 0s depends on the required alignment)
      // Note this operations nulls the lower bits. Hence it reduces the WPtr to
      // the next properly aligned position!
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::LDC), T8xx::AREG)
	.addImm(MaxAlign.value () - 1)
        .setMIFlag(MachineInstr::FrameSetup);
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::NOT), T8xx::AREG)
	.addReg(T8xx::AREG)
	.setMIFlag(MachineInstr::FrameSetup);
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::AND), T8xx::AREG)
	.addReg(T8xx::AREG)
	.addReg(T8xx::BREG)
	.setMIFlag(MachineInstr::FrameSetup);

      // Adjust WPtr accordingly
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::GAJW), T8xx::AREG)
	.addReg(T8xx::AREG)
        .setMIFlag(MachineInstr::FrameSetup);

      // Now the AReg holds the WPtr with just space for parameters
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::STL))
	.addReg(T8xx::AREG)
	.addFrameIndex(TMFI.getWPtrSlot ())
	.addImm(0)
	.setMIFlag(MachineInstr::FrameSetup);
    }
  else
    {
      if (TMFI.getVarArgsSaveSize () == 0)
	{
	  // Real adjustment via AJW
	  BuildMI(MBB, MBBI, dl, TII.get(T8xx::AJW))
	    .addImm(-((StackSize + OffsetAdj) / 4))
	    .setMIFlag(MachineInstr::FrameSetup);
	}
      else
	{
	  // Real adjustment via AJW
	  BuildMI(MBB, MBBI, dl, TII.get(T8xx::AJW))
	    .addImm(-((StackSize - (FixedStackSize + 4) + OffsetAdj) / 4))
	    .setMIFlag(MachineInstr::FrameSetup);
	}
    }
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
}


void T8xxFrameLowering::emitEpilogue(MachineFunction &MF,
                                  MachineBasicBlock &MBB) const {
  LLVM_DEBUG (dbgs() << "emitEpilogue\n");

  // Compute the stack size, to determine if we need an epilogue at all.
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  DebugLoc dl = MBBI->getDebugLoc();
  T8xxMachineFunctionInfo &TMFI = *MF.getInfo<T8xxMachineFunctionInfo> ();

  uint64_t FixedStackSize = computeParameterSize (MF);
  uint64_t StackSize = alignTo (MFI.getStackSize (), getStackAlign ());
  uint64_t OffsetAdj = MFI.getOffsetAdjustment ();

  if ((FixedStackSize + StackSize) == 0) {
    return;
  }

  // The backend has to take care that the requested alignment is met
  // https://groups.google.com/g/llvm-dev/c/U3r-kxd1Loc?pli=1

  // Dynamic stack realignment
  Align MaxAlign = MFI.getMaxAlign();
  LLVM_DEBUG (dbgs() << "Requested Alignment " << MaxAlign.value () << "\n");

  // Now some dynamic alignment would be needed if the requested alignment is above 4 bytes
  if (MFI.shouldRealignStack ())
    {
      // Retrieve "old" WPtr from spill location
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::LDL), T8xx::AREG)
	.addFrameIndex(TMFI.getWPtrSlot ())
	.addImm(0)
	.setMIFlag(MachineInstr::FrameSetup);
      // Set WPtr to "old" WPtr
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::GAJW), T8xx::ABREG)
	.addReg(T8xx::AREG)
        .setMIFlag(MachineInstr::FrameSetup);

      // Since GAJW does not "pop" an element from the register stack,
      // the return value is in BREG, while it needs to be in AREG
      // at this place! Hence swap AREG and BREG
      BuildMI(MBB, MBBI, dl, TII.get(T8xx::REV), T8xx::AREG)
	.addReg(T8xx::ABREG)
        .setMIFlag(MachineInstr::FrameSetup);

      // Note for VarArgs, the WPtr is already adjusted before the function
      // call and readjusted after the function call.
      if (TMFI.getVarArgsSaveSize () == 0)
	{
	  // Finally adjust by parameter space
	  BuildMI(MBB, MBBI, dl, TII.get(T8xx::AJW))
	    .addImm(FixedStackSize / 4)
	    .setMIFlag(MachineInstr::FrameSetup);
	}
    }
  else
    {
      // Restore the stack pointer to what it was at the beginning of the function.
      if (TMFI.getVarArgsSaveSize () == 0)
	{
	  BuildMI(MBB, MBBI, dl, TII.get(T8xx::AJW))
	    .addImm((StackSize + OffsetAdj) / 4)
	    .setMIFlag(MachineInstr::FrameSetup);
	}
      else
	{
	  // For VarArgs, just do the function space but not the function parameters (FixedStackSize)
	  BuildMI(MBB, MBBI, dl, TII.get(T8xx::AJW))
	    .addImm(((StackSize - (FixedStackSize + 4) + OffsetAdj) / 4))
	    .setMIFlag(MachineInstr::FrameSetup);
	}
    }
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
    // TODO: This is currently always returned as T8xx::WPTR (cf. T8xxRegisterInfo.cpp)
    FrameReg = RegInfo->getFrameRegister(MF);
    return StackOffset::getFixed(FrameOffset);
  } else {
    FrameReg = T8xx::WPTR; // %sp
    return StackOffset::getFixed(FrameOffset + MF.getFrameInfo().getStackSize());
  }
}
