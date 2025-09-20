//===-- T8xxAsmBackend.cpp - T8xx Assembler Backend ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T8xxFixupKinds.h"
#include "MCTargetDesc/T8xxMCExpr.h"
#include "MCTargetDesc/T8xxMCTargetDesc.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
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
  case T8xx::fixup_t8xx_addr_npfix:
  case T8xx::fixup_t8xx_addr_base:
  case T8xx::fixup_t8xx_addr_add:
  case T8xx::fixup_t8xx_addr_sub:
    return 0;
    break;

  case T8xx::fixup_t8xx_jump:
    return Value - 1;
    break;

  case T8xx::fixup_t8xx_pcrel_sym:
    return Value - 1;
    break;
  }
}

/// getFixupKindNumBytes - The number of bytes the fixup may change.
static unsigned getFixupKindNumBytes(unsigned Kind) {
  switch (Kind) {
  default:
    llvm_unreachable("Unknown fixup kind!");

  case FK_Data_1:
    return 1;

  case FK_Data_2:
    return 2;

  case FK_Data_4:
  case T8xx::fixup_t8xx_addr_npfix:
    return 4;

  case FK_Data_8:
  case T8xx::fixup_t8xx_addr:
  case T8xx::fixup_t8xx_jump:
  case T8xx::fixup_t8xx_pcrel_sym:
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
      : MCAsmBackend(llvm::endianness::little),
          TheTarget(T), Is64Bit(false) {}

    unsigned getNumFixupKinds() const override {
      return T8xx::NumTargetFixupKinds;
    }

    std::optional<MCFixupKind> getFixupKind(StringRef Name) const override {
      unsigned Type;
      Type = llvm::StringSwitch<unsigned>(Name)
#define ELF_RELOC(X, Y) .Case(#X, Y)
#include "llvm/BinaryFormat/ELFRelocs/T8xx.def"
#undef ELF_RELOC
                 .Case("BFD_RELOC_NONE", ELF::R_T8XX_NONE)
                 .Case("BFD_RELOC_16", ELF::R_T8XX_16)
                 .Case("BFD_RELOC_32", ELF::R_T8XX_32)
                 .Default(-1u);
      if (Type == -1u)
        return std::nullopt;
      return static_cast<MCFixupKind>(FirstLiteralRelocationKind + Type);
    }

    const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override {
      const static MCFixupKindInfo Infos[T8xx::NumTargetFixupKinds] = {
        // name                offset bits  flags
        { "fixup_t8xx_addr",    0,      64,  0},
        { "fixup_t8xx_addr_npfix",0,    32,  0},
        { "fixup_t8xx_jump",    0,      64,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym", 0,    64,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_addr_base", 0,    64,  0},
        { "fixup_t8xx_addr_add", 0,    64,  0},
        { "fixup_t8xx_addr_sub", 0,    64,  0},
        { "fixup_t8xx_align",    0,     0,  0},
      };

      // Fixup kinds from .reloc directive are like R_SPARC_NONE. They do
      // not require any extra processing.
      if (Kind >= FirstLiteralRelocationKind)
        return MCAsmBackend::getFixupKindInfo(FK_NONE);

      if (Kind < FirstTargetFixupKind)
        return MCAsmBackend::getFixupKindInfo(Kind);

      assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
             "Invalid kind!");

      return Infos[Kind - FirstTargetFixupKind];
    }

    //
    // Note:
    // This is called from MCAssembler.evaluateFixup. If this target specific
    // method returns "true", the relocation is presumably left for the linker!
    //

    bool shouldForceRelocation(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
			       const MCSubtargetInfo *STI) override {
      if (Fixup.getKind() >= FirstLiteralRelocationKind)
        return true;
      switch ((T8xx::Fixups)Fixup.getKind()) {
      default:
        return false;
      case T8xx::fixup_t8xx_jump:
      case T8xx::fixup_t8xx_addr:
      case T8xx::fixup_t8xx_addr_npfix:
      case T8xx::fixup_t8xx_pcrel_sym:
      case T8xx::fixup_t8xx_addr_base:
      case T8xx::fixup_t8xx_addr_add:
      case T8xx::fixup_t8xx_addr_sub:
      case T8xx::fixup_t8xx_align:
	return true;
      }
    }


    // Relaxation is completely handled in the linker
    bool mayNeedRelaxation(const MCInst &Inst,
			   const MCSubtargetInfo &STI) const override {
      return false;
    }


    /// Target specific predicate for whether a given fixup requires the
    /// associated instruction to be relaxed.
    bool fixupNeedsRelaxationAdvanced(const MCAssembler &Asm,
                                            const MCFixup &Fixup, bool Resolved,
                                            uint64_t Value,
                                            const MCRelaxableFragment *DF,
                                            const bool WasForced) const override
    {
      int64_t Offset = int64_t(Value);
      printf ("fixupAdvanced %li", Offset);
      if (!Resolved)
	printf ("not resolved\n");
      else
	printf ("resolved\n");

      /* This is the original code of the MCAsmBackend class
      if (!Resolved)
	return true;
      */

      // If the fixup cannot be resolved, we cannot do anything with
      // relaxation!?
      if (!Resolved)
	return false;

      return fixupNeedsRelaxation(Fixup, Value);
    }


    bool fixupNeedsRelaxation(const MCFixup &Fixup,
                                    uint64_t Value) const override {
      printf ("Fixup needs relax %i  %lu\n", (int)Fixup.getTargetKind (), Value);
      int64_t Offset = int64_t(Value);

      return false;
    }


    bool writeNopData(raw_ostream &OS, uint64_t Count,
                      const MCSubtargetInfo *STI) const override {
      // Note: Transputer instruction set does not explicity provide
      // a "NOP" instruction. However, the 0x00 will be j 0, which
      // makes a jump to the next instruction, thereby being
      // equivalent to a NOP instruction.
      for (uint64_t i = 0; i != Count; ++i)
        support::endian::write<uint8_t>(OS, 0x00, Endian);

      return true;
    }


    // Linker relaxation may change code size. We have to insert Nops
    // for .align directive when linker relaxation enabled. So then Linker
    // could satisfy alignment by removing Nops.
    // The function return the total Nops Size we need to insert.
    bool shouldInsertExtraNopBytesForCodeAlign(
       const MCAlignFragment &AF, unsigned &Size) {

      unsigned MinNopLen = 1;

      if (AF.getAlignment() <= MinNopLen) {
	return false;
      } else {
	Size = AF.getAlignment().value() - MinNopLen;
	return true;
      }
    }

    // We need to insert R_RISCV_ALIGN relocation type to indicate the
    // position of Nops and the total bytes of the Nops have been inserted
    // when linker relaxation enabled.
    // The function insert fixup_riscv_align fixup which eventually will
    // transfer to R_RISCV_ALIGN relocation type.
    bool shouldInsertFixupForCodeAlign(MCAssembler &Asm,
				       MCAlignFragment &AF) {
      // Calculate total Nops we need to insert. If there are none to insert
      // then simply return.
      unsigned Count;
      if (!shouldInsertExtraNopBytesForCodeAlign(AF, Count) || (Count == 0))
	return false;
      
      MCContext &Ctx = Asm.getContext();
      const MCExpr *Dummy = MCConstantExpr::create(0, Ctx);
      // Create fixup_riscv_align fixup.
      MCFixup Fixup =
	MCFixup::create(0, Dummy, MCFixupKind(T8xx::fixup_t8xx_align), SMLoc());
      
      uint64_t FixedValue = 0;
      MCValue NopBytes = MCValue::get(Count);
      
      Asm.getWriter().recordRelocation(Asm, &AF, Fixup, NopBytes, FixedValue);
      
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
      if (!Value)
	return;           // Doesn't change encoding.

      unsigned NumBytes = getFixupKindNumBytes(Fixup.getKind());
      unsigned Offset = Fixup.getOffset();

      printf ("NumBytes %u  Offset %u\n", NumBytes, Offset);
      if (!IsResolved)
	return;          // If it is not resolved, leave it as is

      // For each byte of the fragment that the fixup touches, mask in the bits
      // from the fixup value. The Value has been "split up" into the
      // appropriate bitfields above.
      for (unsigned i = 0; i != NumBytes; ++i) {
        unsigned Idx = Endian == llvm::endianness::little ? i : (NumBytes - 1) - i;
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
