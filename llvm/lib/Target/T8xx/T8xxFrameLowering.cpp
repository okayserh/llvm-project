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
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

static cl::opt<bool>
DisableLeafProc("disable-t8xx-leaf-proc",
                cl::init(false),
                cl::desc("Disable T8xx leaf procedure optimization."),
                cl::Hidden);

T8xxFrameLowering::T8xxFrameLowering(const T8xxSubtarget &ST)
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown,
                          ST.is64Bit() ? Align(16) : Align(8), 0,
                          ST.is64Bit() ? Align(16) : Align(8)) {}

void T8xxFrameLowering::emitSPAdjustment(MachineFunction &MF,
                                          MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator MBBI,
                                          int NumBytes,
                                          unsigned ADDrr,
                                          unsigned ADDri) const {

  DebugLoc dl;
  const T8xxInstrInfo &TII =
      *static_cast<const T8xxInstrInfo *>(MF.getSubtarget().getInstrInfo());

  if (NumBytes >= -4096 && NumBytes < 4096) {
    BuildMI(MBB, MBBI, dl, TII.get(ADDri), T8::O6)
      .addReg(T8::O6).addImm(NumBytes);
    return;
  }

  // Emit this the hard way.  This clobbers G1 which we always know is
  // available here.
  if (NumBytes >= 0) {
    // Emit nonnegative numbers with sethi + or.
    // sethi %hi(NumBytes), %g1
    // or %g1, %lo(NumBytes), %g1
    // add %sp, %g1, %sp
    BuildMI(MBB, MBBI, dl, TII.get(T8::SETHIi), T8::G1)
      .addImm(HI22(NumBytes));
    BuildMI(MBB, MBBI, dl, TII.get(T8::ORri), T8::G1)
      .addReg(T8::G1).addImm(LO10(NumBytes));
    BuildMI(MBB, MBBI, dl, TII.get(ADDrr), T8::O6)
      .addReg(T8::O6).addReg(T8::G1);
    return ;
  }

  // Emit negative numbers with sethi + xor.
  // sethi %hix(NumBytes), %g1
  // xor %g1, %lox(NumBytes), %g1
  // add %sp, %g1, %sp
  BuildMI(MBB, MBBI, dl, TII.get(T8::SETHIi), T8::G1)
    .addImm(HIX22(NumBytes));
  BuildMI(MBB, MBBI, dl, TII.get(T8::XORri), T8::G1)
    .addReg(T8::G1).addImm(LOX10(NumBytes));
  BuildMI(MBB, MBBI, dl, TII.get(ADDrr), T8::O6)
    .addReg(T8::O6).addReg(T8::G1);
}

void T8xxFrameLowering::emitPrologue(MachineFunction &MF,
                                      MachineBasicBlock &MBB) const {
  T8xxMachineFunctionInfo *FuncInfo = MF.getInfo<T8xxMachineFunctionInfo>();

  assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const T8xxSubtarget &Subtarget = MF.getSubtarget<T8xxSubtarget>();
  const T8xxInstrInfo &TII =
      *static_cast<const T8xxInstrInfo *>(Subtarget.getInstrInfo());
  const T8xxRegisterInfo &RegInfo =
      *static_cast<const T8xxRegisterInfo *>(Subtarget.getRegisterInfo());
  MachineBasicBlock::iterator MBBI = MBB.begin();
  // Debug location must be unknown since the first debug location is used
  // to determine the end of the prologue.
  DebugLoc dl;
  bool NeedsStackRealignment = RegInfo.shouldRealignStack(MF);

  if (NeedsStackRealignment && !RegInfo.canRealignStack(MF))
    report_fatal_error("Function \"" + Twine(MF.getName()) + "\" required "
                       "stack re-alignment, but LLVM couldn't handle it "
                       "(probably because it has a dynamic alloca).");

  // Get the number of bytes to allocate from the FrameInfo
  int NumBytes = (int) MFI.getStackSize();

  unsigned SAVEri = T8::SAVEri;
  unsigned SAVErr = T8::SAVErr;
  if (FuncInfo->isLeafProc()) {
    if (NumBytes == 0)
      return;
    SAVEri = T8::ADDri;
    SAVErr = T8::ADDrr;
  }

  // The SPARC ABI is a bit odd in that it requires a reserved 92-byte
  // (128 in v9) area in the user's stack, starting at %sp. Thus, the
  // first part of the stack that can actually be used is located at
  // %sp + 92.
  //
  // We therefore need to add that offset to the total stack size
  // after all the stack objects are placed by
  // PrologEpilogInserter calculateFrameObjectOffsets. However, since the stack needs to be
  // aligned *after* the extra size is added, we need to disable
  // calculateFrameObjectOffsets's built-in stack alignment, by having
  // targetHandlesStackFrameRounding return true.


  // Add the extra call frame stack size, if needed. (This is the same
  // code as in PrologEpilogInserter, but also gets disabled by
  // targetHandlesStackFrameRounding)
  if (MFI.adjustsStack() && hasReservedCallFrame(MF))
    NumBytes += MFI.getMaxCallFrameSize();

  // Adds the SPARC subtarget-specific spill area to the stack
  // size. Also ensures target-required alignment.
  NumBytes = Subtarget.getAdjustedFrameSize(NumBytes);

  // Finally, ensure that the size is sufficiently aligned for the
  // data on the stack.
  NumBytes = alignTo(NumBytes, MFI.getMaxAlign());

  // Update stack size with corrected value.
  MFI.setStackSize(NumBytes);

  emitSPAdjustment(MF, MBB, MBBI, -NumBytes, SAVErr, SAVEri);

  unsigned regFP = RegInfo.getDwarfRegNum(T8::I6, true);

  // Emit ".cfi_def_cfa_register 30".
  unsigned CFIIndex =
      MF.addFrameInst(MCCFIInstruction::createDefCfaRegister(nullptr, regFP));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  // Emit ".cfi_window_save".
  CFIIndex = MF.addFrameInst(MCCFIInstruction::createWindowSave(nullptr));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  unsigned regInRA = RegInfo.getDwarfRegNum(T8::I7, true);
  unsigned regOutRA = RegInfo.getDwarfRegNum(T8::O7, true);
  // Emit ".cfi_register 15, 31".
  CFIIndex = MF.addFrameInst(
      MCCFIInstruction::createRegister(nullptr, regOutRA, regInRA));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  if (NeedsStackRealignment) {
    int64_t Bias = Subtarget.getStackPointerBias();
    unsigned regUnbiased;
    if (Bias) {
      // This clobbers G1 which we always know is available here.
      regUnbiased = T8::G1;
      // add %o6, BIAS, %g1
      BuildMI(MBB, MBBI, dl, TII.get(T8::ADDri), regUnbiased)
        .addReg(T8::O6).addImm(Bias);
    } else
      regUnbiased = T8::O6;

    // andn %regUnbiased, MaxAlign-1, %regUnbiased
    Align MaxAlign = MFI.getMaxAlign();
    BuildMI(MBB, MBBI, dl, TII.get(T8::ANDNri), regUnbiased)
        .addReg(regUnbiased)
        .addImm(MaxAlign.value() - 1U);

    if (Bias) {
      // add %g1, -BIAS, %o6
      BuildMI(MBB, MBBI, dl, TII.get(T8::ADDri), T8::O6)
        .addReg(regUnbiased).addImm(-Bias);
    }
  }
}

MachineBasicBlock::iterator T8xxFrameLowering::
eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const {
  if (!hasReservedCallFrame(MF)) {
    MachineInstr &MI = *I;
    int Size = MI.getOperand(0).getImm();
    if (MI.getOpcode() == T8::ADJCALLSTACKDOWN)
      Size = -Size;

    if (Size)
      emitSPAdjustment(MF, MBB, I, Size, T8::ADDrr, T8::ADDri);
  }
  return MBB.erase(I);
}


void T8xxFrameLowering::emitEpilogue(MachineFunction &MF,
                                  MachineBasicBlock &MBB) const {
  T8xxMachineFunctionInfo *FuncInfo = MF.getInfo<T8xxMachineFunctionInfo>();
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  const T8xxInstrInfo &TII =
      *static_cast<const T8xxInstrInfo *>(MF.getSubtarget().getInstrInfo());
  DebugLoc dl = MBBI->getDebugLoc();
  assert((MBBI->getOpcode() == T8::RETL || MBBI->getOpcode() == T8::TAIL_CALL ||
          MBBI->getOpcode() == T8::TAIL_CALLri) &&
         "Can only put epilog before 'retl' or 'tail_call' instruction!");
  if (!FuncInfo->isLeafProc()) {
    BuildMI(MBB, MBBI, dl, TII.get(T8::RESTORErr), T8::G0).addReg(T8::G0)
      .addReg(T8::G0);
    return;
  }
  MachineFrameInfo &MFI = MF.getFrameInfo();

  int NumBytes = (int) MFI.getStackSize();
  if (NumBytes != 0)
    emitSPAdjustment(MF, MBB, MBBI, NumBytes, T8::ADDrr, T8::ADDri);

  // Preserve return address in %o7
  if (MBBI->getOpcode() == T8::TAIL_CALL) {
    MBB.addLiveIn(T8::O7);
    BuildMI(MBB, MBBI, dl, TII.get(T8::ORrr), T8::G1)
        .addReg(T8::G0)
        .addReg(T8::O7);
    BuildMI(MBB, MBBI, dl, TII.get(T8::ORrr), T8::O7)
        .addReg(T8::G0)
        .addReg(T8::G1);
  }
}

bool T8xxFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  // Reserve call frame if there are no variable sized objects on the stack.
  return !MF.getFrameInfo().hasVarSizedObjects();
}

// hasFP - Return true if the specified function should have a dedicated frame
// pointer register.  This is true if the function has variable sized allocas or
// if frame pointer elimination is disabled.
bool T8xxFrameLowering::hasFP(const MachineFunction &MF) const {
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
    FrameReg = T8::O6; // %sp
    return StackOffset::getFixed(FrameOffset + MF.getFrameInfo().getStackSize());
  }
}

static bool LLVM_ATTRIBUTE_UNUSED verifyLeafProcRegUse(MachineRegisterInfo *MRI)
{

  for (unsigned reg = T8::I0; reg <= T8::I7; ++reg)
    if (MRI->isPhysRegUsed(reg))
      return false;

  for (unsigned reg = T8::L0; reg <= T8::L7; ++reg)
    if (MRI->isPhysRegUsed(reg))
      return false;

  return true;
}

bool T8xxFrameLowering::isLeafProc(MachineFunction &MF) const
{

  MachineRegisterInfo &MRI = MF.getRegInfo();
  MachineFrameInfo    &MFI = MF.getFrameInfo();

  return !(MFI.hasCalls()               // has calls
           || MRI.isPhysRegUsed(T8::L0) // Too many registers needed
           || MRI.isPhysRegUsed(T8::O6) // %sp is used
           || hasFP(MF)                 // need %fp
           || MF.hasInlineAsm());       // has inline assembly
}

void T8xxFrameLowering::remapRegsForLeafProc(MachineFunction &MF) const {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  // Remap %i[0-7] to %o[0-7].
  for (unsigned reg = T8::I0; reg <= T8::I7; ++reg) {
    if (!MRI.isPhysRegUsed(reg))
      continue;

    unsigned mapped_reg = reg - T8::I0 + T8::O0;

    // Replace I register with O register.
    MRI.replaceRegWith(reg, mapped_reg);

    // Also replace register pair super-registers.
    if ((reg - T8::I0) % 2 == 0) {
      unsigned preg = (reg - T8::I0) / 2 + T8::I0_I1;
      unsigned mapped_preg = preg - T8::I0_I1 + T8::O0_O1;
      MRI.replaceRegWith(preg, mapped_preg);
    }
  }

  // Rewrite MBB's Live-ins.
  for (MachineBasicBlock &MBB : MF) {
    for (unsigned reg = T8::I0_I1; reg <= T8::I6_I7; ++reg) {
      if (!MBB.isLiveIn(reg))
        continue;
      MBB.removeLiveIn(reg);
      MBB.addLiveIn(reg - T8::I0_I1 + T8::O0_O1);
    }
    for (unsigned reg = T8::I0; reg <= T8::I7; ++reg) {
      if (!MBB.isLiveIn(reg))
        continue;
      MBB.removeLiveIn(reg);
      MBB.addLiveIn(reg - T8::I0 + T8::O0);
    }
  }

  assert(verifyLeafProcRegUse(&MRI));
#ifdef EXPENSIVE_CHECKS
  MF.verify(0, "After LeafProc Remapping");
#endif
}

void T8xxFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                              BitVector &SavedRegs,
                                              RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
  if (!DisableLeafProc && isLeafProc(MF)) {
    T8xxMachineFunctionInfo *MFI = MF.getInfo<T8xxMachineFunctionInfo>();
    MFI->setLeafProc(true);

    remapRegsForLeafProc(MF);
  }

}
