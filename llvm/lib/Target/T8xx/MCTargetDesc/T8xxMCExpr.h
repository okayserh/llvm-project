//====- T8xxMCExpr.h - T8xx specific MC expression classes --*- C++ -*-=====//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes T8xx-specific MCExprs, used for modifiers like
// "%hi" or "%lo" etc.,
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SPARC_MCTARGETDESC_SPARCMCEXPR_H
#define LLVM_LIB_TARGET_SPARC_MCTARGETDESC_SPARCMCEXPR_H

#include "T8xxFixupKinds.h"
#include "llvm/MC/MCExpr.h"

namespace llvm {

class StringRef;
class T8xxMCExpr : public MCTargetExpr {
public:
  enum VariantKind {
    VK_T8xx_None,
    VK_T8xx_LO,
    VK_T8xx_HI,
    VK_T8xx_H44,
    VK_T8xx_M44,
    VK_T8xx_L44,
    VK_T8xx_HH,
    VK_T8xx_HM,
    VK_T8xx_LM,
    VK_T8xx_PC22,
    VK_T8xx_PC10,
    VK_T8xx_GOT22,
    VK_T8xx_GOT10,
    VK_T8xx_GOT13,
    VK_T8xx_13,
    VK_T8xx_WPLT30,
    VK_T8xx_WDISP30,
    VK_T8xx_R_DISP32,
    VK_T8xx_TLS_GD_HI22,
    VK_T8xx_TLS_GD_LO10,
    VK_T8xx_TLS_GD_ADD,
    VK_T8xx_TLS_GD_CALL,
    VK_T8xx_TLS_LDM_HI22,
    VK_T8xx_TLS_LDM_LO10,
    VK_T8xx_TLS_LDM_ADD,
    VK_T8xx_TLS_LDM_CALL,
    VK_T8xx_TLS_LDO_HIX22,
    VK_T8xx_TLS_LDO_LOX10,
    VK_T8xx_TLS_LDO_ADD,
    VK_T8xx_TLS_IE_HI22,
    VK_T8xx_TLS_IE_LO10,
    VK_T8xx_TLS_IE_LD,
    VK_T8xx_TLS_IE_LDX,
    VK_T8xx_TLS_IE_ADD,
    VK_T8xx_TLS_LE_HIX22,
    VK_T8xx_TLS_LE_LOX10,
    VK_T8xx_HIX22,
    VK_T8xx_LOX10,
    VK_T8xx_GOTDATA_HIX22,
    VK_T8xx_GOTDATA_LOX10,
    VK_T8xx_GOTDATA_OP,
  };

private:
  const VariantKind Kind;
  const MCExpr *Expr;

  explicit T8xxMCExpr(VariantKind Kind, const MCExpr *Expr)
      : Kind(Kind), Expr(Expr) {}

public:
  /// @name Construction
  /// @{

  static const T8xxMCExpr *create(VariantKind Kind, const MCExpr *Expr,
                                 MCContext &Ctx);
  /// @}
  /// @name Accessors
  /// @{

  /// getOpcode - Get the kind of this expression.
  VariantKind getKind() const { return Kind; }

  /// getSubExpr - Get the child of this expression.
  const MCExpr *getSubExpr() const { return Expr; }

  /// getFixupKind - Get the fixup kind of this expression.
  T8xx::Fixups getFixupKind() const { return getFixupKind(Kind); }

  /// @}
  void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override;
  bool evaluateAsRelocatableImpl(MCValue &Res,
                                 const MCAsmLayout *Layout,
                                 const MCFixup *Fixup) const override;
  void visitUsedExpr(MCStreamer &Streamer) const override;
  MCFragment *findAssociatedFragment() const override {
    return getSubExpr()->findAssociatedFragment();
  }

  void fixELFSymbolsInTLSFixups(MCAssembler &Asm) const override;

  static bool classof(const MCExpr *E) {
    return E->getKind() == MCExpr::Target;
  }

  static bool classof(const T8xxMCExpr *) { return true; }

  static VariantKind parseVariantKind(StringRef name);
  static bool printVariantKind(raw_ostream &OS, VariantKind Kind);
  static T8xx::Fixups getFixupKind(VariantKind Kind);
};

} // end namespace llvm.

#endif
