//===-- T8xxSubtarget.cpp - SPARC Subtarget Information ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the SPARC specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "T8xxSubtarget.h"
#include "T8xx.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/MathExtras.h"

using namespace llvm;

#define DEBUG_TYPE "t8xx-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "T8xxGenSubtargetInfo.inc"

void T8xxSubtarget::anchor() { }

T8xxSubtarget &T8xxSubtarget::initializeSubtargetDependencies(StringRef CPU,
                                                                StringRef FS) {
  UseFPU = false;
  UseSoftFloat = false;

  // Determine default and user specified characteristics
  std::string CPUName = std::string(CPU);
  if (CPUName.empty())
    CPUName = "t8xx";

  // Parse features string.
  ParseSubtargetFeatures(CPUName, /*TuneCPU*/ CPUName, FS);

  return *this;
}

T8xxSubtarget::T8xxSubtarget(const Triple &TT, const std::string &CPU,
                               const std::string &FS, const TargetMachine &TM)
    : T8xxGenSubtargetInfo(TT, CPU, /*TuneCPU*/ CPU, FS), TargetTriple(TT),
      InstrInfo(initializeSubtargetDependencies(CPU, FS)),
      TLInfo(TM, *this), FrameLowering(*this) {}

int T8xxSubtarget::getAdjustedFrameSize(int frameSize) const {

    // Emit the correct save instruction based on the number of bytes in
    // the frame. Minimum stack frame size according to V8 ABI is:
    //   16 words for register window spill
    //    1 word for address of returned aggregate-value
    // +  6 words for passing parameters on the stack
    // ----------
    //   23 words * 4 bytes per word = 92 bytes
    frameSize += 92;

    // Round up to next doubleword boundary -- a double-word boundary
    // is required by the ABI.
    frameSize = alignTo(frameSize, 8);

  return frameSize;
}

bool T8xxSubtarget::enableMachineScheduler() const {
  return true;
}
