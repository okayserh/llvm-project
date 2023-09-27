//===-- T8xxAsmBackend.cpp - T8xx Assembler Backend ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T8xxFixupKinds.h"
#include "MCTargetDesc/T8xxMCTargetDesc.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/EndianStream.h"

using namespace llvm;

static unsigned adjustFixupValue(unsigned Kind, uint64_t Value) {
  switch (Kind) {
  default:
    llvm_unreachable("Unknown fixup kind!");
  case FK_Data_1:
  case FK_Data_2:
  case FK_Data_4:
  case FK_Data_8:
    return Value;

  case T8xx::fixup_t8xx_addr:
  case T8xx::fixup_t8xx_jump:
    return 0;
  }
}

/// getFixupKindNumBytes - The number of bytes the fixup may change.
static unsigned getFixupKindNumBytes(unsigned Kind) {
    switch (Kind) {
  default:
    return 4;
  case FK_Data_1:
    return 1;
  case FK_Data_2:
    return 2;
  case FK_Data_8:
    return 8;
  }
}

namespace {
  class T8xxAsmBackend : public MCAsmBackend {
  protected:
    const Target &TheTarget;
    bool Is64Bit;

  public:
    T8xxAsmBackend(const Target &T)
        : MCAsmBackend(StringRef(T.getName()) == "sparcel" ? support::little
                                                           : support::big),
          TheTarget(T), Is64Bit(StringRef(TheTarget.getName()) == "sparcv9") {}

    unsigned getNumFixupKinds() const override {
      return T8xx::NumTargetFixupKinds;
    }

    std::optional<MCFixupKind> getFixupKind(StringRef Name) const override {
      unsigned Type;
      Type = llvm::StringSwitch<unsigned>(Name)
#define ELF_RELOC(X, Y) .Case(#X, Y)
#include "llvm/BinaryFormat/ELFRelocs/T8xx.def"
#undef ELF_RELOC
                 .Case("BFD_RELOC_NONE", ELF::R_SPARC_NONE)
                 .Case("BFD_RELOC_8", ELF::R_SPARC_8)
                 .Case("BFD_RELOC_16", ELF::R_SPARC_16)
                 .Case("BFD_RELOC_32", ELF::R_SPARC_32)
                 .Case("BFD_RELOC_64", ELF::R_SPARC_64)
                 .Default(-1u);
      if (Type == -1u)
        return std::nullopt;
      return static_cast<MCFixupKind>(FirstLiteralRelocationKind + Type);
    }

    const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override {
      const static MCFixupKindInfo InfosBE[T8xx::NumTargetFixupKinds] = {
        // name                    offset bits  flags
        { "fixup_sparc_call30",     2,     30,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_sparc_br22",      10,     22,  MCFixupKindInfo::FKF_IsPCRel },
      };

      const static MCFixupKindInfo InfosLE[T8xx::NumTargetFixupKinds] = {
        // name                    offset bits  flags
        { "fixup_sparc_call30",     0,     30,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_sparc_br22",       0,     22,  MCFixupKindInfo::FKF_IsPCRel },
      };

      // Fixup kinds from .reloc directive are like R_SPARC_NONE. They do
      // not require any extra processing.
      if (Kind >= FirstLiteralRelocationKind)
        return MCAsmBackend::getFixupKindInfo(FK_NONE);

      if (Kind < FirstTargetFixupKind)
        return MCAsmBackend::getFixupKindInfo(Kind);

      assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
             "Invalid kind!");
      if (Endian == support::little)
        return InfosLE[Kind - FirstTargetFixupKind];

      return InfosBE[Kind - FirstTargetFixupKind];
    }

    bool shouldForceRelocation(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target) override {
      if (Fixup.getKind() >= FirstLiteralRelocationKind)
        return true;
      switch ((T8xx::Fixups)Fixup.getKind()) {
      default:
        return false;
	/*
      case T8xx::fixup_sparc_wplt30:
        if (Target.getSymA()->getSymbol().isTemporary())
          return false;
        [[fallthrough]];
      case T8xx::fixup_sparc_tls_gd_hi22:
      case T8xx::fixup_sparc_tls_gd_lo10:
        return true;
	*/
      }
    }

    /// fixupNeedsRelaxation - Target specific predicate for whether a given
    /// fixup requires the associated instruction to be relaxed.
    bool fixupNeedsRelaxation(const MCFixup &Fixup,
                              uint64_t Value,
                              const MCRelaxableFragment *DF,
                              const MCAsmLayout &Layout) const override {
      // FIXME.
      llvm_unreachable("fixupNeedsRelaxation() unimplemented");
      return false;
    }
    void relaxInstruction(MCInst &Inst,
                          const MCSubtargetInfo &STI) const override {
      // FIXME.
      llvm_unreachable("relaxInstruction() unimplemented");
    }

    bool writeNopData(raw_ostream &OS, uint64_t Count,
                      const MCSubtargetInfo *STI) const override {
      // Cannot emit NOP with size not multiple of 32 bits.
      if (Count % 4 != 0)
        return false;

      uint64_t NumNops = Count / 4;
      for (uint64_t i = 0; i != NumNops; ++i)
        support::endian::write<uint32_t>(OS, 0x01000000, Endian);

      return true;
    }
  };

  class ELFT8xxAsmBackend : public T8xxAsmBackend {
    Triple::OSType OSType;
  public:
    ELFT8xxAsmBackend(const Target &T, Triple::OSType OSType) :
      T8xxAsmBackend(T), OSType(OSType) { }

    void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                    const MCValue &Target, MutableArrayRef<char> Data,
                    uint64_t Value, bool IsResolved,
                    const MCSubtargetInfo *STI) const override {

      if (Fixup.getKind() >= FirstLiteralRelocationKind)
        return;
      Value = adjustFixupValue(Fixup.getKind(), Value);
      if (!Value) return;           // Doesn't change encoding.

      unsigned NumBytes = getFixupKindNumBytes(Fixup.getKind());
      unsigned Offset = Fixup.getOffset();
      // For each byte of the fragment that the fixup touches, mask in the bits
      // from the fixup value. The Value has been "split up" into the
      // appropriate bitfields above.
      for (unsigned i = 0; i != NumBytes; ++i) {
        unsigned Idx = Endian == support::little ? i : (NumBytes - 1) - i;
        Data[Offset + Idx] |= uint8_t((Value >> (i * 8)) & 0xff);
      }
    }

    std::unique_ptr<MCObjectTargetWriter>
    createObjectTargetWriter() const override {
      uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(OSType);
      return createT8xxELFObjectWriter(Is64Bit, OSABI);
    }
  };

} // end anonymous namespace

MCAsmBackend *llvm::createT8xxAsmBackend(const Target &T,
                                          const MCSubtargetInfo &STI,
                                          const MCRegisterInfo &MRI,
                                          const MCTargetOptions &Options) {
  return new ELFT8xxAsmBackend(T, STI.getTargetTriple().getOS());
}
