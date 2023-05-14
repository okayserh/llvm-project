//===-- T8xxTargetStreamer.cpp - T8xx Target Streamer Methods -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides T8xx specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "T8xxTargetStreamer.h"
#include "T8xxInstPrinter.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

// pin vtable to this file
T8xxTargetStreamer::T8xxTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

void T8xxTargetStreamer::anchor() {}

T8xxTargetAsmStreamer::T8xxTargetAsmStreamer(MCStreamer &S,
                                               formatted_raw_ostream &OS)
    : T8xxTargetStreamer(S), OS(OS) {}

void T8xxTargetAsmStreamer::emitT8xxRegisterIgnore(MCRegister reg) {
  OS << "\t.register "
     << "%" << StringRef(T8xxInstPrinter::getRegisterName(reg)).lower()
     << ", #ignore\n";
}

void T8xxTargetAsmStreamer::emitT8xxRegisterScratch(MCRegister reg) {
  OS << "\t.register "
     << "%" << StringRef(T8xxInstPrinter::getRegisterName(reg)).lower()
     << ", #scratch\n";
}

T8xxTargetELFStreamer::T8xxTargetELFStreamer(MCStreamer &S)
    : T8xxTargetStreamer(S) {}

MCELFStreamer &T8xxTargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}
