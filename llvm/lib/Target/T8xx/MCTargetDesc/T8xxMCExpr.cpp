//===-- T8xxMCExpr.cpp - T8xx specific MC expression classes --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of the assembly expression modifiers
// accepted by the T8xx architecture (e.g. "%hi", "%lo", ...).
//
//===----------------------------------------------------------------------===//

#include "T8xxMCExpr.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCObjectStreamer.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"

using namespace llvm;

#define DEBUG_TYPE "t8xxmcexpr"

const T8xxMCExpr*
T8xxMCExpr::create(VariantKind Kind, const MCExpr *Expr,
                      MCContext &Ctx) {
    return new (Ctx) T8xxMCExpr(Kind, Expr);
}

void T8xxMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {

  bool closeParen = printVariantKind(OS, Kind);

  const MCExpr *Expr = getSubExpr();
  Expr->print(OS, MAI);

  if (closeParen)
    OS << ')';
}

bool T8xxMCExpr::printVariantKind(raw_ostream &OS, VariantKind Kind)
{
  switch (Kind) {
  case VK_T8xx_None:     return false;
  case VK_T8xx_IPTRREL:  return false;
  case VK_T8xx_GLOBAL:   return false;
  }
  llvm_unreachable("Unhandled T8xxMCExpr::VariantKind");
}

T8xxMCExpr::VariantKind T8xxMCExpr::parseVariantKind(StringRef name)
{
  return StringSwitch<T8xxMCExpr::VariantKind>(name)
    .Case("iptrrel",   VK_T8xx_IPTRREL)
    .Case("global_pfix",    VK_T8xx_GLOBAL)
    .Case("global",    VK_T8xx_GLOBAL_NPFIX)
    .Default(VK_T8xx_None);
}

T8xx::Fixups T8xxMCExpr::getFixupKind(T8xxMCExpr::VariantKind Kind) {
  printf ("FixupKind %i\n", (int) Kind);

  switch (Kind) {
  default: llvm_unreachable("Unhandled T8xxMCExpr::VariantKind");
  case VK_T8xx_IPTRREL:  return T8xx::fixup_t8xx_jump;
  case VK_T8xx_GLOBAL:   return T8xx::fixup_t8xx_addr;
  case VK_T8xx_GLOBAL_NPFIX:   return T8xx::fixup_t8xx_addr_npfix;

  }
}

bool
T8xxMCExpr::evaluateAsRelocatableImpl(MCValue &Res,
                                       const MCAssembler *Asm,
                                       const MCFixup *Fixup) const {
  return getSubExpr()->evaluateAsRelocatable(Res, Asm, Fixup);
}


void T8xxMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}
