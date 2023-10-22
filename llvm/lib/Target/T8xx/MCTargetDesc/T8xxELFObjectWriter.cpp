//===-- T8xxELFObjectWriter.cpp - T8xx ELF Writer -----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T8xxFixupKinds.h"
#include "MCTargetDesc/T8xxMCExpr.h"
#include "MCTargetDesc/T8xxMCTargetDesc.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace {
  class T8xxELFObjectWriter : public MCELFObjectTargetWriter {
  public:
    T8xxELFObjectWriter(bool Is64Bit, uint8_t OSABI)
      : MCELFObjectTargetWriter(false, OSABI,
                                ELF::EM_T8XX,
                                /*HasRelocationAddend*/ true) {}

    ~T8xxELFObjectWriter() override = default;

  protected:
    unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                          const MCFixup &Fixup, bool IsPCRel) const override;

    bool needsRelocateWithSymbol(const MCSymbol &Sym,
                                 unsigned Type) const override;

  };
}

unsigned T8xxELFObjectWriter::getRelocType(MCContext &Ctx,
                                            const MCValue &Target,
                                            const MCFixup &Fixup,
                                            bool IsPCRel) const {
  MCFixupKind Kind = Fixup.getKind();
  if (Kind >= FirstLiteralRelocationKind)
    return Kind - FirstLiteralRelocationKind;

  if (const T8xxMCExpr *SExpr = dyn_cast<T8xxMCExpr>(Fixup.getValue())) {
    if (SExpr->getKind() == T8xxMCExpr::VK_T8xx_IPTRREL)
      return ELF::R_T8XX_ADDR;
  }

  if (IsPCRel) {
    switch(Fixup.getTargetKind()) {
    default:
      llvm_unreachable("Unimplemented fixup -> relocation");
    case FK_Data_1:                  return ELF::R_T8XX_ADDR;
    case FK_Data_2:                  return ELF::R_T8XX_ADDR;
    case FK_Data_4:                  return ELF::R_T8XX_ADDR;
    case FK_Data_8:                  return ELF::R_T8XX_ADDR;

    case T8xx::fixup_t8xx_jump: return ELF::R_T8XX_JUMP;
    case T8xx::fixup_t8xx_addr: return ELF::R_T8XX_ADDR;
      /*
    case T8xx::fixup_sparc_call30:  return ELF::R_SPARC_WDISP30;
    case T8xx::fixup_sparc_br22:    return ELF::R_SPARC_WDISP22;
      */
    }
  }

  switch(Fixup.getTargetKind()) {
  default:
    llvm_unreachable("Unimplemented fixup -> relocation");
  case FK_NONE:                  return ELF::R_T8XX_NONE;
  case FK_Data_1:                return ELF::R_T8XX_ADDR;
  case FK_Data_2:                return ((Fixup.getOffset() % 2)
                                         ? ELF::R_T8XX_ADDR
                                         : ELF::R_T8XX_ADDR);
  case FK_Data_4:                return ((Fixup.getOffset() % 4)
                                         ? ELF::R_T8XX_ADDR
                                         : ELF::R_T8XX_ADDR);
  case FK_Data_8:                return ((Fixup.getOffset() % 8)
                                         ? ELF::R_T8XX_ADDR
                                         : ELF::R_T8XX_ADDR);
    /*
  case T8xx::fixup_sparc_13:    return ELF::R_SPARC_13;
  case T8xx::fixup_sparc_hi22:  return ELF::R_SPARC_HI22;
    */
  }

  return ELF::R_T8XX_NONE;
}

bool T8xxELFObjectWriter::needsRelocateWithSymbol(const MCSymbol &Sym,
                                                 unsigned Type) const {
  switch (Type) {
    default:
      return false;

    // All relocations that use a GOT need a symbol, not an offset, as
    // the offset of the symbol within the section is irrelevant to
    // where the GOT entry is. Don't need to list all the TLS entries,
    // as they're all marked as requiring a symbol anyways.
      /*
  case ELF::R_SPARC_GOT10:
    case ELF::R_SPARC_GOT13:
    case ELF::R_SPARC_GOT22:
    case ELF::R_SPARC_GOTDATA_HIX22:
    case ELF::R_SPARC_GOTDATA_LOX10:
    case ELF::R_SPARC_GOTDATA_OP_HIX22:
    case ELF::R_SPARC_GOTDATA_OP_LOX10:
      */
  case ELF::R_T8XX_JUMP:
      return true;
  }
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createT8xxELFObjectWriter(bool Is64Bit, uint8_t OSABI) {
  return std::make_unique<T8xxELFObjectWriter>(Is64Bit, OSABI);
}
