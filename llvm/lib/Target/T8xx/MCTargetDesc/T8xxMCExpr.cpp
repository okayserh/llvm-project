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

#include "MCTargetDesc/T8xxMCAsmInfo.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCObjectStreamer.h"

using namespace llvm;

#define DEBUG_TYPE "t8xxmcexpr"

/*
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

// This method is called from "T8xxAsmPrinter.cpp, printOperand" and can be
// used to introduce annotations into the resulting assembler code
// which indicate special symbols for example.
bool T8xxMCExpr::printVariantKind(raw_ostream &OS, VariantKind Kind)
{
  switch (Kind) {
  case VK_T8xx_None:
    return false;
  case VK_T8xx_IPTRREL:
    OS << "%iptr_jmp(";
    return true;
  case VK_T8xx_SYMREL:
    OS << "%iptr_sym(";
    return true;
  case VK_T8xx_GLOBAL:
    return false;
  case VK_T8xx_GLOBAL_NPFIX:
    return false;
  }
  llvm_unreachable("Unhandled T8xxMCExpr::VariantKind");
}
*/


StringRef T8xx::getSpecifierName(uint16_t S) {
  // clang-format off
  switch (uint16_t(S)) {
  case 0:                      return {};
  case ELF::R_T8XX_32:         return "lon";
  case ELF::R_T8XX_16:         return "sho";
  case ELF::R_T8XX_ADDR:       return "global_pfix";
  case ELF::R_T8XX_JUMP:       return "iptr_jmp";
  case ELF::R_T8XX_ADDR_NPFIX: return "global";
  case ELF::R_T8XX_LDPI_SYM:   return "iptr_sym";
  case ELF::R_T8XX_ADDR_BASE:  return "addr_base";
  case ELF::R_T8XX_ADDR_ADD:   return "addr_add";
  case ELF::R_T8XX_ADDR_SUB:   return "addr_sub";
  case ELF::R_T8XX_ALIGN:      return "align";
  }
  // clang-format on
  llvm_unreachable("Unhandled T8xxMCExpr::Specifier");
}


uint16_t T8xx::parseSpecifier(StringRef name) {
  return StringSwitch<uint16_t>(name)
      .Case("lon",         ELF::R_T8XX_32)
      .Case("sho",         ELF::R_T8XX_16)
      .Case("global_pfix", ELF::R_T8XX_ADDR)
      .Case("iptr_jmp",    ELF::R_T8XX_JUMP)
      .Case("global",      ELF::R_T8XX_ADDR_NPFIX)
      .Case("iptr_sym",    ELF::R_T8XX_LDPI_SYM)
      .Case("addr_base",   ELF::R_T8XX_ADDR_BASE)
      .Case("addr_add",    ELF::R_T8XX_ADDR_ADD)
      .Case("addr_sub",    ELF::R_T8XX_ADDR_SUB)
      .Case("align",       ELF::R_T8XX_ALIGN)
      .Default(0);
}


// This method is used from T8xxAsmParser in method "matchT8xxAsmModifiers"
// to match modifiers before symbols.
/*
T8xxMCExpr::VariantKind T8xxMCExpr::parseVariantKind(StringRef name)
{
  return StringSwitch<T8xxMCExpr::VariantKind>(name)
    .Case("iptr_jmp",   VK_T8xx_IPTRREL)
    .Case("iptr_sym",   VK_T8xx_SYMREL)
    .Case("global",     VK_T8xx_GLOBAL_NPFIX)
    .Case("global_pfix",VK_T8xx_GLOBAL)
    .Default(VK_T8xx_None);
}
*/

/*
T8xx::Fixups T8xxMCExpr::getFixupKind(T8xxMCExpr::VariantKind Kind) {

  LLVM_DEBUG(dbgs() << "FixupKind " << (int) Kind << "\n");

  switch (Kind) {
  default: llvm_unreachable("Unhandled T8xxMCExpr::VariantKind");
  case VK_T8xx_IPTRREL:      return T8xx::fixup_t8xx_jump;
  case VK_T8xx_SYMREL:       return T8xx::fixup_t8xx_pcrel_sym;
  case VK_T8xx_GLOBAL:       return T8xx::fixup_t8xx_addr;
  case VK_T8xx_GLOBAL_NPFIX: return T8xx::fixup_t8xx_addr_npfix;

  }
}
*/
/*
void T8xxMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}
*/
