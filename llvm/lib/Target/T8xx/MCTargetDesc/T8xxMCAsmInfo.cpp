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
#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/TargetParser/Triple.h"

using namespace llvm;

void T8xxELFMCAsmInfo::anchor() {}

T8xxELFMCAsmInfo::T8xxELFMCAsmInfo(const Triple &TheTriple) {
  IsLittleEndian = TheTriple.isLittleEndian();

  Data16bitsDirective = "\t.half\t";
  Data32bitsDirective = "\t.word\t";
  // .xword is only supported by V9.
  //  Data64bitsDirective = (isV9) ? "\t.xword\t" : nullptr;
  ZeroDirective = "\t.skip\t";
  CommentString = "//";
  SupportsDebugInformation = true;

  // TODO: Note, the "DwarfCFI" comes presumably from the Sparc "template"
  // This must correspond to the ExceptionHandling defined in "TargetParser/Triple.cpp".
  // Probably the t8xx architecture currently gets "None".

  //  ExceptionsType = ExceptionHandling::DwarfCFI;
  ExceptionsType = ExceptionHandling::None;

  UsesELFSectionDirectiveForBSS = true;
}


void T8xxELFMCAsmInfo::printSpecifierExpr(raw_ostream &OS,
                                           const MCSpecifierExpr &Expr) const {
  StringRef S = T8xx::getSpecifierName(Expr.getSpecifier());
  if (!S.empty())
    OS << '%' << S << '(';
  printExpr(OS, *Expr.getSubExpr());
  if (!S.empty())
    OS << ')';
}


bool T8xxELFMCAsmInfo::evaluateAsRelocatableImpl(const MCSpecifierExpr &Expr, MCValue &Res,
						 const MCAssembler *Asm) const
{
  dbgs() << "EvaluatAsReloc\n";
  return (true);
}
