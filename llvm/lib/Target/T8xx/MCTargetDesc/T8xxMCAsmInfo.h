//===- T8xxMCAsmInfo.h - T8xx asm properties -----------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the T8xxMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_T8XX_MCTARGETDESC_T8XXMCASMINFO_H
#define LLVM_LIB_TARGET_T8XX_MCTARGETDESC_T8XXMCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"
#include "llvm/MC/MCFixup.h"

namespace llvm {

class Triple;

class T8xxELFMCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit T8xxELFMCAsmInfo(const Triple &TheTriple);

  /*
  const MCExpr*
  getExprForPersonalitySymbol(const MCSymbol *Sym, unsigned Encoding,
                              MCStreamer &Streamer) const override;
  const MCExpr* getExprForFDESymbol(const MCSymbol *Sym,
                                    unsigned Encoding,
                                    MCStreamer &Streamer) const override;
  */
  void printSpecifierExpr(raw_ostream &OS,
                          const MCSpecifierExpr &Expr) const override;
};

namespace T8xx {
using Specifier = uint16_t;
  enum {   // Not really needed for T8xx
  S_None,
  S_ADDR = FirstTargetFixupKind,
};
  
uint16_t parseSpecifier(StringRef name);
StringRef getSpecifierName(uint16_t S);
} // namespace Sparc
  
} // end namespace llvm

#endif // LLVM_LIB_TARGET_T8XX_MCTARGETDESC_T8XXMCASMINFO_H
