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

#define DEBUG_TYPE "sparcmcexpr"

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
    .Case("global",    VK_T8xx_GLOBAL)
    .Default(VK_T8xx_None);
}

T8xx::Fixups T8xxMCExpr::getFixupKind(T8xxMCExpr::VariantKind Kind) {
  switch (Kind) {
  default: llvm_unreachable("Unhandled T8xxMCExpr::VariantKind");

  case VK_T8xx_IPTRREL:  return T8xx::fixup_t8xx_jump;
  case VK_T8xx_GLOBAL:   return T8xx::fixup_t8xx_addr;

  }
}

bool
T8xxMCExpr::evaluateAsRelocatableImpl(MCValue &Res,
                                       const MCAsmLayout *Layout,
                                       const MCFixup *Fixup) const {
  return getSubExpr()->evaluateAsRelocatable(Res, Layout, Fixup);
}

static void fixELFSymbolsInTLSFixupsImpl(const MCExpr *Expr, MCAssembler &Asm) {
  switch (Expr->getKind()) {
  case MCExpr::Target:
    llvm_unreachable("Can't handle nested target expr!");
    break;

  case MCExpr::Constant:
    break;

  case MCExpr::Binary: {
    const MCBinaryExpr *BE = cast<MCBinaryExpr>(Expr);
    fixELFSymbolsInTLSFixupsImpl(BE->getLHS(), Asm);
    fixELFSymbolsInTLSFixupsImpl(BE->getRHS(), Asm);
    break;
  }

  case MCExpr::SymbolRef: {
    const MCSymbolRefExpr &SymRef = *cast<MCSymbolRefExpr>(Expr);
    cast<MCSymbolELF>(SymRef.getSymbol()).setType(ELF::STT_TLS);
    break;
  }

  case MCExpr::Unary:
    fixELFSymbolsInTLSFixupsImpl(cast<MCUnaryExpr>(Expr)->getSubExpr(), Asm);
    break;
  }

}

void T8xxMCExpr::fixELFSymbolsInTLSFixups(MCAssembler &Asm) const {
  switch(getKind()) {
  default: return;
    /*
  case VK_T8xx_TLS_GD_CALL:
  case VK_T8xx_TLS_LDM_CALL: {
    // The corresponding relocations reference __tls_get_addr, as they call it,
    // but this is only implicit; we must explicitly add it to our symbol table
    // to bind it for these uses.
    MCSymbol *Symbol = Asm.getContext().getOrCreateSymbol("__tls_get_addr");
    Asm.registerSymbol(*Symbol);
    auto ELFSymbol = cast<MCSymbolELF>(Symbol);
    if (!ELFSymbol->isBindingSet())
      ELFSymbol->setBinding(ELF::STB_GLOBAL);
    [[fallthrough]];
  }
  case VK_T8xx_TLS_LE_HIX22:
  case VK_T8xx_TLS_LE_LOX10: break;
    */
  }
  fixELFSymbolsInTLSFixupsImpl(getSubExpr(), Asm);
}

void T8xxMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}
