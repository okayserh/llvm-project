//===- T8xxMCAsmInfo.cpp - T8xx asm properties --------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the T8xxMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "T8xxMCAsmInfo.h"
#include "T8xxMCExpr.h"
//#include "llvm/ADT/Triple.h"
#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/TargetParser/Triple.h"

using namespace llvm;

void T8xxELFMCAsmInfo::anchor() {}

T8xxELFMCAsmInfo::T8xxELFMCAsmInfo(const Triple &TheTriple) {
  IsLittleEndian = true; // TODO: Remove -> Should be the default?

  Data16bitsDirective = "\t.half\t";
  Data32bitsDirective = "\t.word\t";
  // .xword is only supported by V9.
  //  Data64bitsDirective = (isV9) ? "\t.xword\t" : nullptr;
  ZeroDirective = "\t.skip\t";
  CommentString = "//";
  SupportsDebugInformation = true;

  ExceptionsType = ExceptionHandling::DwarfCFI;

  UsesELFSectionDirectiveForBSS = true;
}

const MCExpr*
T8xxELFMCAsmInfo::getExprForPersonalitySymbol(const MCSymbol *Sym,
                                               unsigned Encoding,
                                               MCStreamer &Streamer) const {
  if (Encoding & dwarf::DW_EH_PE_pcrel) {
    MCContext &Ctx = Streamer.getContext();
    return T8xxMCExpr::create(T8xxMCExpr::VK_T8xx_IPTRREL,
                               MCSymbolRefExpr::create(Sym, Ctx), Ctx);
  }

  return MCAsmInfo::getExprForPersonalitySymbol(Sym, Encoding, Streamer);
}

const MCExpr*
T8xxELFMCAsmInfo::getExprForFDESymbol(const MCSymbol *Sym,
                                       unsigned Encoding,
                                       MCStreamer &Streamer) const {
  if (Encoding & dwarf::DW_EH_PE_pcrel) {
    MCContext &Ctx = Streamer.getContext();
    return T8xxMCExpr::create(T8xxMCExpr::VK_T8xx_IPTRREL,
                               MCSymbolRefExpr::create(Sym, Ctx), Ctx);
  }
  return MCAsmInfo::getExprForFDESymbol(Sym, Encoding, Streamer);
}
