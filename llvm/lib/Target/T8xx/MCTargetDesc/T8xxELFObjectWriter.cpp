//===-- T8xxELFObjectWriter.cpp - T8xx ELF Writer -----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T8xxFixupKinds.h"
#include "MCTargetDesc/T8xxMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
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

    unsigned getRelocType(const MCFixup &, const MCValue &,
                          bool IsPCRel) const override;
    bool needsRelocateWithSymbol(const MCValue & Val,
                                 unsigned Type) const override;

  };
}

unsigned T8xxELFObjectWriter::getRelocType(const MCFixup &Fixup,
                                           const MCValue &Target,
                                           bool IsPCRel) const {
  MCFixupKind Kind = Fixup.getKind();
  auto Spec = Target.getSpecifier();

  if (mc::isRelocation(Kind))
    return Kind;

  if (IsPCRel) {
    switch(Kind) {
    default:
      llvm_unreachable("Unimplemented fixup -> relocation");
      return  ELF::R_T8XX_NONE;
    case FK_Data_1:                  return ELF::R_T8XX_8;
    case FK_Data_2:                  return ELF::R_T8XX_16;
    case FK_Data_4:                  return ELF::R_T8XX_ADDR_NPFIX;
    case FK_Data_8:                  return ELF::R_T8XX_ADDR;

      // TODO: It seem these fixups are only selected when the "IsPCRel" flag is set. However,
      // some of these relocations are not PC relative. Needs to be ordered properly.
      //    case T8xx::fixup_t8xx_pcrel_sym: return ELF::R_T8XX_LDPI_SYM;
    case T8xx::fixup_t8xx_jump: return ELF::R_T8XX_JUMP;
    case T8xx::fixup_t8xx_pcrel_sym: return ELF::R_T8XX_LDPI_SYM;

    }
  }

  switch(Fixup.getKind()) {
  default:
    llvm_unreachable("Unimplemented fixup -> relocation");
  case FK_NONE:                  return ELF::R_T8XX_NONE;
  case FK_Data_1:                return ELF::R_T8XX_8;
  case FK_Data_2:                return ELF::R_T8XX_16;
  case FK_Data_4:                return ELF::R_T8XX_ADDR_NPFIX;
  case FK_Data_8:                return ELF::R_T8XX_ADDR_NPFIX;

  case T8xx::fixup_t8xx_addr:    return ELF::R_T8XX_ADDR;
  case T8xx::fixup_t8xx_addr_npfix: return ELF::R_T8XX_ADDR_NPFIX;

  case T8xx::fixup_t8xx_addr_base:  return ELF::R_T8XX_ADDR_BASE;
  case T8xx::fixup_t8xx_addr_add:   return ELF::R_T8XX_ADDR_ADD;
  case T8xx::fixup_t8xx_addr_sub:   return ELF::R_T8XX_ADDR_SUB;
  case T8xx::fixup_t8xx_align:      return ELF::R_T8XX_ALIGN;
  }

  return ELF::R_T8XX_NONE;
}

bool T8xxELFObjectWriter::needsRelocateWithSymbol(const MCValue &/*Val*/,
                                                 unsigned Type) const {
  if (Type == ELF::R_T8XX_ALIGN)
    return (false);
  else
    return (true);  
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createT8xxELFObjectWriter(bool Is64Bit, uint8_t OSABI) {
  return std::make_unique<T8xxELFObjectWriter>(Is64Bit, OSABI);
}
