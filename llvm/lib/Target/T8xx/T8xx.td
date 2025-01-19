//===-- T8xx.td - Describe the T8xx Target Machine -------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces which we are implementing
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Transputer Subtarget features.
//

def FeatureFPU
  : SubtargetFeature<"fpu", "UseFPU", "true",
                     "Use integrated FPU">;

def FeatureSoftFloat : SubtargetFeature<"soft-float", "UseSoftFloat", "true",
                              "Use software emulation for floating point">;

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "T8xxRegisterInfo.td"
include "T8xxCallingConv.td"
include "T8xxSchedule.td"
include "T8xxInstrInfo.td"

def T8xxInstrInfo : InstrInfo;

def T8xxAsmParser : AsmParser {
  bit ShouldEmitMatchRegisterName = 0;
}

//===----------------------------------------------------------------------===//
// T8xx processors supported.
//===----------------------------------------------------------------------===//

class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, NoItineraries, Features>;

def : Proc<"t8xx",              [FeatureFPU]>;

def : Processor<"generic", NoItineraries, []>;

//===----------------------------------------------------------------------===//
// Declare the target which we are implementing
//===----------------------------------------------------------------------===//

def T8xxAsmWriter : AsmWriter {
  string AsmWriterClassName  = "InstPrinter";
  int PassSubtarget = 1;
  int Variant = 0;
}

def T8xx : Target {
  // Pull in Instruction Info:
  let InstructionSet = T8xxInstrInfo;
  let AssemblyParsers  = [T8xxAsmParser];
  let AssemblyWriters = [T8xxAsmWriter];
  let AllowRegisterRenaming = 1;
}
