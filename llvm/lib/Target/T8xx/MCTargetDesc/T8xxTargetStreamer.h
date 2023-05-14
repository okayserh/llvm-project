//===-- T8xxTargetStreamer.h - T8xx Target Streamer ----------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SPARC_MCTARGETDESC_SPARCTARGETSTREAMER_H
#define LLVM_LIB_TARGET_SPARC_MCTARGETDESC_SPARCTARGETSTREAMER_H

#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCRegister.h"

namespace llvm {

class formatted_raw_ostream;

class T8xxTargetStreamer : public MCTargetStreamer {
  virtual void anchor();

public:
  T8xxTargetStreamer(MCStreamer &S);
  /// Emit ".register <reg>, #ignore".
  virtual void emitT8xxRegisterIgnore(MCRegister reg){};
  /// Emit ".register <reg>, #scratch".
  virtual void emitT8xxRegisterScratch(MCRegister reg){};
};

// This part is for ascii assembly output
class T8xxTargetAsmStreamer : public T8xxTargetStreamer {
  formatted_raw_ostream &OS;

public:
  T8xxTargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);
  void emitT8xxRegisterIgnore(MCRegister reg) override;
  void emitT8xxRegisterScratch(MCRegister reg) override;
};

// This part is for ELF object output
class T8xxTargetELFStreamer : public T8xxTargetStreamer {
public:
  T8xxTargetELFStreamer(MCStreamer &S);
  MCELFStreamer &getStreamer();
  void emitT8xxRegisterIgnore(MCRegister reg) override {}
  void emitT8xxRegisterScratch(MCRegister reg) override {}
};
} // end namespace llvm

#endif
