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
    return 0;
    break;

  case T8xx::fixup_t8xx_jump:
    return Value - 1;
    break;

  case T8xx::fixup_t8xx_jump_p8:
    {
      int64_t Offset = int64_t(Value);
      printf ("J/CJ Kind %u   Value %lu  Offset %li\n", Kind, Value, Offset);

      Offset -= 2;
      uint32_t imm_dec = (Offset < 0 ? (~Offset) : Offset) & 0xFFFFFFFFu;

      Value = Offset < 0 ? 0x0040 : 0;  // From "pfix" to "nfix"
      Value |= (imm_dec & 0xF0) >> 4;
      if (Offset < 0)
	imm_dec = ~imm_dec;
      Value |= (imm_dec & 0xF) << 8;
      printf ("Value %04x\n", Value);

      return Value;
    }

  case T8xx::fixup_t8xx_pcrel_sym:
    return Value - 1;
    break;

  case T8xx::fixup_t8xx_pcrel_sym_p8:
    {
      int64_t Offset = int64_t(Value);
      printf ("LDC Kind %u   Value %lu  Offset %li\n", Kind, Value, Offset);

      Offset -= 2;
      uint32_t imm_dec = (Offset < 0 ? (~Offset) : Offset) & 0xFFFFFFFFu;

      Value = Offset < 0 ? 0x0040 : 0;  // From "pfix" to "nfix"
      Value |= (imm_dec & 0xF0) >> 4;
      if (Offset < 0)
	imm_dec = ~imm_dec;
      Value |= (imm_dec & 0xF) << 8;
      printf ("Value %04x\n", Value);

      return Value;
    }

  }
}

/// getFixupKindNumBytes - The number of bytes the fixup may change.
static unsigned getFixupKindNumBytes(unsigned Kind) {
  switch (Kind) {
  default:
    llvm_unreachable("Unknown fixup kind!");

  case FK_Data_1:
  case T8xx::fixup_t8xx_jump:
  case T8xx::fixup_t8xx_pcrel_sym:
    return 1;

  case FK_Data_2:
  case T8xx::fixup_t8xx_jump_p8:
  case T8xx::fixup_t8xx_pcrel_sym_p8:
    return 2;

  case T8xx::fixup_t8xx_jump_p12:
  case T8xx::fixup_t8xx_pcrel_sym_p12:
    return 3;

  case FK_Data_4:
  case T8xx::fixup_t8xx_jump_p16:
  case T8xx::fixup_t8xx_pcrel_sym_p16:
    return 4;

  case T8xx::fixup_t8xx_jump_p20:
  case T8xx::fixup_t8xx_pcrel_sym_p20:
    return 5;

  case T8xx::fixup_t8xx_jump_p24:
  case T8xx::fixup_t8xx_pcrel_sym_p24:
    return 6;

  case T8xx::fixup_t8xx_jump_p28:
  case T8xx::fixup_t8xx_pcrel_sym_p28:
    return 7;

  case FK_Data_8:
  case T8xx::fixup_t8xx_jump_p32:
  case T8xx::fixup_t8xx_pcrel_sym_p32:
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
        { "fixup_t8xx_addr",    0,      16,  0},
        { "fixup_t8xx_addr_npfix",0,    16,  0},
        { "fixup_t8xx_jump",    0,       8,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_jump_p8", 0,      16,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_jump_p12",0,      24,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_jump_p16",0,      32,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_jump_p20",0,      40,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_jump_p24",0,      48,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_jump_p28",0,      56,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_jump_p32",0,      64,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym", 0,     8,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym_p8",0,  16,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym_p12",0, 24,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym_p16",0, 32,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym_p20",0, 40,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym_p24",0, 48,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym_p28",0, 56,  MCFixupKindInfo::FKF_IsPCRel },
        { "fixup_t8xx_pcrel_sym_p32",0, 64,  MCFixupKindInfo::FKF_IsPCRel },
      };

      printf ("getFixupKindInfo %i\n", (int)Kind);

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
	return true;
      }
    }

    void relaxInstruction(MCInst &Inst,
                          const MCSubtargetInfo &STI) const override {
      printf ("relax Instruction\n");
      unsigned RelaxedOp;

      // Relax jumps to next size
      switch (Inst.getOpcode ())
	{
	case T8xx::CJ:       RelaxedOp = T8xx::CJ_P8;    break;
	case T8xx::CJ_P8:    RelaxedOp = T8xx::CJ_P12;   break;
	case T8xx::CJ_P12:   RelaxedOp = T8xx::CJ_P16;   break;
	case T8xx::CJ_P16:   RelaxedOp = T8xx::CJ_P20;   break;
	case T8xx::CJ_P20:   RelaxedOp = T8xx::CJ_P24;   break;
	case T8xx::CJ_P24:   RelaxedOp = T8xx::CJ_P28;   break;
	case T8xx::CJ_P28:   RelaxedOp = T8xx::CJ_P32;   break;
	case T8xx::JUMP:     RelaxedOp = T8xx::JUMP_P8;  break;
	case T8xx::JUMP_P8:  RelaxedOp = T8xx::JUMP_P12; break;
	case T8xx::JUMP_P12: RelaxedOp = T8xx::JUMP_P16; break;
	case T8xx::JUMP_P16: RelaxedOp = T8xx::JUMP_P20; break;
	case T8xx::JUMP_P20: RelaxedOp = T8xx::JUMP_P24; break;
	case T8xx::JUMP_P24: RelaxedOp = T8xx::JUMP_P28; break;
	case T8xx::JUMP_P28: RelaxedOp = T8xx::JUMP_P32; break;
	case T8xx::LDC:      RelaxedOp = T8xx::LDC_P8;   break;
	}

      // Create new relaxed instruction
      switch (Inst.getOpcode ())
	{
	case T8xx::CJ:
	case T8xx::CJ_P8:
	case T8xx::CJ_P12:
	case T8xx::CJ_P16:
	case T8xx::CJ_P20:
	case T8xx::CJ_P24:
	case T8xx::CJ_P28:
	  {
	    MCInst Res;
	    Res.setOpcode(RelaxedOp);
	    Res.addOperand(MCOperand::createReg(T8xx::AREG));
	    Res.addOperand(Inst.getOperand(1));
	    Inst = std::move(Res);
	    return;
	  }
	  break;

	case T8xx::JUMP:
	case T8xx::JUMP_P8:
	case T8xx::JUMP_P12:
	case T8xx::JUMP_P16:
	case T8xx::JUMP_P20:
	case T8xx::JUMP_P24:
	case T8xx::JUMP_P28:
	  {
	    MCInst Res;
	    Res.setOpcode(RelaxedOp);
	    Res.addOperand(Inst.getOperand(0));
	    Inst = std::move(Res);
	    return;
	  }
	  break;

	case T8xx::LDC:
	case T8xx::LDC_P8:
	  {
	    MCInst Res;
	    Res.setOpcode(RelaxedOp);
	    Res.addOperand(Inst.getOperand(0));
	    Res.addOperand(Inst.getOperand(1));
	    Inst = std::move(Res);
	    return;
	  }
	  break;
	}
    }

    bool mayNeedRelaxation(const MCInst &Inst,
			   const MCSubtargetInfo &STI) const override {
#if 0
      switch (Inst.getOpcode ())
	{
	  /*
	case T8xx::LDC:
	  {
	    if (Inst.getOperand(1).isExpr ())
	      {
		const MCExpr *Expr = Inst.getOperand(1).getExpr ();
		if (const T8xxMCExpr *SExpr = dyn_cast<T8xxMCExpr>(Expr))
		  {
		    printf ("LDC Relax  Expr Kind %i  T8xxMCExpr Kind %i\n",
			    (int) Expr->getKind (),
			    (int) SExpr->getKind ());
		    const MCExpr *SubExpr = SExpr->getSubExpr ();
		    SubExpr->dump ();

		    switch (SExpr->getKind ())
		      {
			// Only instruction pointer relative LDCs need relaxation.
		      case T8xxMCExpr::VK_T8xx_IPTRREL:
			return true;
		      default:
			break;
		      }
		  }
	      }
	    return false;
	  }
	  break;
	  */
	  
	case T8xx::CJ:
	case T8xx::CJ_P8:
	case T8xx::CJ_P12:
	case T8xx::CJ_P16:
	case T8xx::CJ_P20:
	case T8xx::CJ_P24:
	case T8xx::CJ_P28:
	case T8xx::CJ_P32:
	case T8xx::JUMP:
	case T8xx::JUMP_P8:
	case T8xx::JUMP_P12:
	case T8xx::JUMP_P16:
	case T8xx::JUMP_P20:
	case T8xx::JUMP_P24:
	case T8xx::JUMP_P28:
	case T8xx::JUMP_P32:
	  return true;
	  break;
	}
#endif
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

      // TODO: Remove before practical use!
      return (false);
      
      switch (Fixup.getTargetKind()) {
      case T8xx::fixup_t8xx_pcrel_sym:
      case T8xx::fixup_t8xx_jump:
	Offset -= 1;  // Correction for instruction itself
	printf ("Needs relaxation %li\n", Offset);
	if ((Offset < 0) || (Offset > 15))
	  return true;
	break;

      case T8xx::fixup_t8xx_jump_p8:
	Offset -= 2;  // Correction for instruction itself
	printf ("Needs relaxation %li\n", Offset);
	if ((Offset < -255) || (Offset > 255))
	  return true;
	break;

      case T8xx::fixup_t8xx_jump_p12:
	Offset -= 3;  // Correction for instruction itself
	printf ("Needs relaxation %li\n", Offset);
	if ((Offset < -4095) || (Offset > 4095))
	  return true;
	break;

      case T8xx::fixup_t8xx_jump_p16:
	Offset -= 4;  // Correction for instruction itself
	printf ("Needs relaxation %li\n", Offset);
	if ((Offset < -65535) || (Offset > 65535))
	  return true;
	break;

      case T8xx::fixup_t8xx_jump_p20:
	Offset -= 5;  // Correction for instruction itself
	printf ("Needs relaxation %li\n", Offset);
	if ((Offset < -1048575) || (Offset > 1048575))
	  return true;
	break;

      case T8xx::fixup_t8xx_jump_p24:
	Offset -= 6;  // Correction for instruction itself
	printf ("Needs relaxation %li\n", Offset);
	if ((Offset < -16777215) || (Offset > 16777215))
	  return true;
	break;

      case T8xx::fixup_t8xx_jump_p28:
	Offset -= 7;  // Correction for instruction itself
	printf ("Needs relaxation %li\n", Offset);
	if ((Offset < -268435455) || (Offset > 268435455))
	  return true;
	break;
      }
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
