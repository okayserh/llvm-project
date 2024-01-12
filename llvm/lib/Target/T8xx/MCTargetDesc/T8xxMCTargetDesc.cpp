//===-- T8xxMCTargetDesc.cpp - T8xx Target Descriptions -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides T8xx specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "T8xxMCTargetDesc.h"
#include "T8xxInstPrinter.h"
#include "T8xxMCAsmInfo.h"
#include "T8xxTargetStreamer.h"
#include "TargetInfo/T8xxTargetInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#define ENABLE_INSTR_PREDICATE_VERIFIER
#include "T8xxGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "T8xxGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "T8xxGenRegisterInfo.inc"

static MCAsmInfo *createT8xxMCAsmInfo(const MCRegisterInfo &MRI,
                                       const Triple &TT,
                                       const MCTargetOptions &Options) {
  MCAsmInfo *MAI = new T8xxELFMCAsmInfo(TT);
  unsigned Reg = MRI.getDwarfRegNum(T8xx::R6, true); // TODO: Replace O6 with R6 to remove compiler error, but functionality is questionable
  //  MCCFIInstruction Inst = MCCFIInstruction::cfiDefCfa(nullptr, Reg, 0);
  //  MAI->addInitialFrameState(Inst);
  return MAI;
}

static MCAsmInfo *createT8xxV9MCAsmInfo(const MCRegisterInfo &MRI,
                                         const Triple &TT,
                                         const MCTargetOptions &Options) {
  MCAsmInfo *MAI = new T8xxELFMCAsmInfo(TT);
  unsigned Reg = MRI.getDwarfRegNum(T8xx::R6, true); // TODO: see above
  MCCFIInstruction Inst = MCCFIInstruction::cfiDefCfa(nullptr, Reg, 2047);
  MAI->addInitialFrameState(Inst);
  return MAI;
}

static MCInstrInfo *createT8xxMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitT8xxMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createT8xxMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitT8xxMCRegisterInfo(X, T8xx::R7);  // TODO: see above
  return X;
}

static MCSubtargetInfo *
createT8xxMCSubtargetInfo(const Triple &TT, StringRef CPU, StringRef FS) {
  if (CPU.empty())
    //    CPU = (TT.getArch() == Triple::sparcv9) ? "v9" : "v8";
    CPU = "t8xx";
  return createT8xxMCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCTargetStreamer *
createObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new T8xxTargetELFStreamer(S);
}

static MCTargetStreamer *createTargetAsmStreamer(MCStreamer &S,
                                                 formatted_raw_ostream &OS,
                                                 MCInstPrinter *InstPrint,
                                                 bool isVerboseAsm) {
  return new T8xxTargetAsmStreamer(S, OS);
}

static MCTargetStreamer *createNullTargetStreamer(MCStreamer &S) {
  return new T8xxTargetStreamer(S);
}

static MCInstPrinter *createT8xxMCInstPrinter(const Triple &T,
                                               unsigned SyntaxVariant,
                                               const MCAsmInfo &MAI,
                                               const MCInstrInfo &MII,
                                               const MCRegisterInfo &MRI) {
  return new T8xxInstPrinter(MAI, MII, MRI);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeT8xxTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfoFn X(getTheT8xxTarget(), createT8xxMCAsmInfo);

  for (Target *T :
       {&getTheT8xxTarget()}) {
    // Register the MC instruction info.
    TargetRegistry::RegisterMCInstrInfo(*T, createT8xxMCInstrInfo);

    // Register the MC register info.
    TargetRegistry::RegisterMCRegInfo(*T, createT8xxMCRegisterInfo);

    // Register the MC subtarget info.
    TargetRegistry::RegisterMCSubtargetInfo(*T, createT8xxMCSubtargetInfo);

    // Register the MC Code Emitter.
    TargetRegistry::RegisterMCCodeEmitter(*T, createT8xxMCCodeEmitter);

    // Register the asm backend.
    TargetRegistry::RegisterMCAsmBackend(*T, createT8xxAsmBackend);

    // Register the object target streamer.
    TargetRegistry::RegisterObjectTargetStreamer(*T,
                                                 createObjectTargetStreamer);

    // Register the asm streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createTargetAsmStreamer);

    // Register the null streamer.
    TargetRegistry::RegisterNullTargetStreamer(*T, createNullTargetStreamer);

    // Register the MCInstPrinter
    TargetRegistry::RegisterMCInstPrinter(*T, createT8xxMCInstPrinter);
  }
}
