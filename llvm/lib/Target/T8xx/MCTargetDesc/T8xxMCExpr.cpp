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
