//===-- T8xxAsmBackend.cpp - T8xx Assembler Backend ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "T8xxAsmBackend.h"
#include "T8xxFixupKinds.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/EndianStream.h"

using namespace llvm;

// Temporary workaround for old linkers that do not support ULEB128 relocations,
// which are abused by DWARF v5 DW_LLE_offset_pair/DW_RLE_offset_pair
// implemented in Clang/LLVM.
static cl::opt<bool> ULEB128Reloc(
    "t8xx-uleb128-reloc", cl::init(true), cl::Hidden,
    cl::desc("Emit R_T8XX_SET_ULEB128/E_T8XX_SUB_ULEB128 if appropriate"));


T8xxAsmBackend::T8xxAsmBackend(const MCSubtargetInfo &STI, uint8_t OSABI,
			       const MCTargetOptions &Options)
      : MCAsmBackend(llvm::endianness::little),
	STI(STI), OSABI(OSABI), Is64Bit(false), TargetOptions(Options)
{
}

std::optional<MCFixupKind> T8xxAsmBackend::getFixupKind(StringRef Name) const {
  if (STI.getTargetTriple().isOSBinFormatELF()) {
    unsigned Type;
    Type = llvm::StringSwitch<unsigned>(Name)
#define ELF_RELOC(X, Y) .Case(#X, Y)
#include "llvm/BinaryFormat/ELFRelocs/T8xx.def"
#undef ELF_RELOC
      .Case("BFD_RELOC_NONE", ELF::R_T8XX_NONE)
      .Case("BFD_RELOC_8",  ELF::R_T8XX_8)
      .Case("BFD_RELOC_16", ELF::R_T8XX_16)
      .Case("BFD_RELOC_32", ELF::R_T8XX_32)
      .Default(-1u);
    if (Type == -1u)
      return static_cast<MCFixupKind>(FirstLiteralRelocationKind + Type);
  }
  return std::nullopt;
}

MCFixupKindInfo T8xxAsmBackend::getFixupKindInfo(MCFixupKind Kind) const {
  const static MCFixupKindInfo Infos[] = {
    // This table *must* be in the order that the fixup_* kinds are defined in
    // T8xxFixupKinds.h.
    //
    // name                offset bits  flags
    { "fixup_t8xx_addr",    0,      64,  0},
    { "fixup_t8xx_addr_npfix",0,    32,  0},
    { "fixup_t8xx_jump",    0,      64,  0},
    { "fixup_t8xx_pcrel_sym", 0,    64,  0},
    { "fixup_t8xx_addr_base", 0,    64,  0},
    { "fixup_t8xx_addr_add", 0,    64,  0},
    { "fixup_t8xx_addr_sub", 0,    64,  0},
    { "fixup_t8xx_align",    0,     0,  0},
  };
  static_assert((std::size(Infos)) == T8xx::NumTargetFixupKinds,
		"Not all fixup kinds added to Infos array");

  // Fixup kinds from .reloc directive are like R_SPARC_NONE. They do
  // not require any extra processing.
  dbgs () << "getFixupKindInfo\n";

  if (mc::isRelocation(Kind))
    return {};

  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);

  assert(unsigned(Kind - FirstTargetFixupKind) < T8xx::NumTargetFixupKinds &&
	 "Invalid kind!");
  return Infos[Kind - FirstTargetFixupKind];
}

// Note: This seems the replacement for the prior method
// "shouldInsertFixupForCodeAlign". It is needed for the Transputer.
bool T8xxAsmBackend::relaxAlign(MCFragment &F, unsigned &Size)
{
  unsigned MinNopLen = 1;  // For Transputer the NOP is just a j 0 instruction of 1 byte.
  
  Size = F.getAlignment().value() - MinNopLen;
  auto *Expr = MCConstantExpr::create(Size, getContext());
  MCFixup Fixup =
      MCFixup::create(0, Expr, FirstLiteralRelocationKind + ELF::R_T8XX_ALIGN);
  F.setVarFixups({Fixup});
  F.setLinkerRelaxable();
  return true;
}

bool T8xxAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count,
				  const MCSubtargetInfo *STI) const {
  // Note: Transputer instruction set does not explicity provide
  // a "NOP" instruction. However, the 0x00 will be j 0, which
  // makes a jump to the next instruction, thereby being
  // equivalent to a NOP instruction.
  for (uint64_t i = 0; i != Count; ++i)
    support::endian::write<uint8_t>(OS, 0x00, Endian);

  return true;
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

static bool exprHasSymbolRef (const MCExpr *Expr)
{
  switch (Expr->getKind ())
    {
    case MCExpr::SymbolRef:
      return true;
    case MCExpr::Binary:
      {
	const MCBinaryExpr *ABE = cast<MCBinaryExpr>(Expr);
	return (exprHasSymbolRef (ABE->getLHS()) ||
		exprHasSymbolRef (ABE->getRHS()));
      }
    case MCExpr::Unary:
      {
	const MCUnaryExpr *UE = cast<MCUnaryExpr>(Expr);
	return (exprHasSymbolRef (UE->getSubExpr()));
      }
    default:
      return false;
    }
}


// Note: This is called from MCAssembler in method evaluateFixup.
// If it returns a non NULL pointer, the return value is copied
// into the "IsResolved" flag.
// At the end, the MCAssembler::evaluateFixup calls
// Backend::applyFixup

std::optional<bool> T8xxAsmBackend::evaluateFixup(const MCFragment &, MCFixup &Fixup, MCValue &Target,
						  uint64_t &Value)
{
  if (Fixup.getKind() >= T8xx::fixup_t8xx_addr)
    return (false);
  else
    return {};

  dbgs () << "evaluateFixup " << Fixup.getKind() << "  Value  " << Value << "\n";
  Fixup.getValue ()->dump();

  bool includesSymbol = exprHasSymbolRef (Fixup.getValue ());
  if (includesSymbol)
    dbgs () << "Incl Symbol\n";
  else
    dbgs () << "Incl No Symbol\n";

  switch (Fixup.getValue()->getKind ())
    {
    case MCExpr::Binary:
      {
	dbgs() << "Binary\n";
	const MCBinaryExpr *ABE = cast<MCBinaryExpr>(Fixup.getValue ());
	MCValue LHSValue, RHSValue;

	printf ("LHS Kind = %i   RHS Kind = %i\n",
		(int) ABE->getLHS()->getKind(),
		(int) ABE->getRHS()->getKind());

	if (ABE->getLHS()->getKind()==MCExpr::Binary)
	  {
	    dbgs() << "LHS Binary\n";
	    ABE->getLHS()->dump();

	    const MCBinaryExpr *ABC = cast<MCBinaryExpr>(ABE->getLHS ());

	    if (!ABC->getLHS()->evaluateAsRelocatable(LHSValue, Asm) ||
		!ABC->getRHS()->evaluateAsRelocatable(RHSValue, Asm)) {
	      dbgs() << "LHS Target Expressions\n";
	    }

	    if (ABC->getLHS()->getKind()==MCExpr::SymbolRef)
	      {
		dbgs() << "LHS LHS  \n";
		const MCSymbolRefExpr *SRE = cast<MCSymbolRefExpr>(ABC->getLHS());
		MCSymbol &Sym = const_cast<MCSymbol &>(SRE->getSymbol());
		Sym.dump ();
	      }
	  }

	/*
	if (!ABE->getLHS()->evaluateAsRelocatable(LHSValue, Asm) ||
	    !ABE->getRHS()->evaluateAsRelocatable(RHSValue, Asm)) {
	  dbgs() << "Target Expressions\n";
	}
	*/
	if (LHSValue.getAddSym())
	  dbgs() << "LHS Sym A\n";
	if (LHSValue.getAddSym())
	  dbgs() << "LHS Sym B\n";

	if (LHSValue.isAbsolute())
	  dbgs() << "LHS Absolute\n";
	if (RHSValue.isAbsolute())
	  dbgs() << "RHS Absolute\n";

      }
      break;
    case MCExpr::SymbolRef:
      dbgs() << "SymbolRef\n";
      break;
    case MCExpr::Unary:
      dbgs() << "Unary\n";
      break;
    case MCExpr::Specifier:
      dbgs() << "Specifier\n";
      break;
    case MCExpr::Target:
      dbgs() << "Target\n";
      break;
    }

  return {};
}

bool T8xxAsmBackend::addReloc(const MCFragment &F, const MCFixup &Fixup,
			      const MCValue &Target, uint64_t &FixedValue,
			      bool IsResolved) {
  uint64_t FixedValueA, FixedValueB;

  dbgs() << "addReloc\n";

  if (Target.getAddSym())
    Target.getAddSym()->dump();
  if (Target.getSubSym())
    Target.getSubSym()->dump();

  if (Target.getSubSym()) {
    dbgs () << "Target.getSubSym()\n";

    assert(Target.getSpecifier() == 0 &&
	   "relocatable SymA-SymB cannot have relocation specifier");
    unsigned TA = 0, TB = 0;
    switch (Fixup.getKind()) {
    case llvm::FK_Data_1:
      TA = ELF::R_T8XX_ADD8;
      TB = ELF::R_T8XX_SUB8;
      break;
    case llvm::FK_Data_2:
      TA = ELF::R_T8XX_ADD16;
      TB = ELF::R_T8XX_SUB16;
      break;
    case llvm::FK_Data_4:
      TA = ELF::R_T8XX_ADD32;
      TB = ELF::R_T8XX_SUB32;
      break;
    default:
      llvm_unreachable("unsupported fixup size");
    }
    MCValue A = MCValue::get(Target.getAddSym(), nullptr, Target.getConstant());
    MCValue B = MCValue::get(Target.getSubSym());
    auto FA = MCFixup::create(Fixup.getOffset(), nullptr, TA);
    auto FB = MCFixup::create(Fixup.getOffset(), nullptr, TB);
    Asm->getWriter().recordRelocation(F, FA, A, FixedValueA);
    Asm->getWriter().recordRelocation(F, FB, B, FixedValueB);
    FixedValue = FixedValueA - FixedValueB;
    return false;
  }

  // If linker relaxation is enabled and supported by the current fixup, then we
  // always want to generate a relocation.
  /*
    bool NeedsRelax = Fixup.isLinkerRelaxable() &&
    relaxableFixupNeedsRelocation(Fixup.getKind());
    if (NeedsRelax)
    IsResolved = false;

    if (IsResolved && Fixup.isPCRel())
    IsResolved = isPCRelFixupResolved(Target.getAddSym(), F);
  */
  IsResolved = false;

  if (!IsResolved) {
    // Some Fixups require a VENDOR relocation, record it (directly) before we
    // add the relocation.
    //	maybeAddVendorReloc(F, Fixup);

    dbgs () << "recordReloc\n";
    Asm->getWriter().recordRelocation(F, Fixup, Target, FixedValue);

    /*
      if (NeedsRelax) {
      // Some Fixups get a RELAX relocation, record it (directly) after we add
      // the relocation.
      MCFixup RelaxFixup =
      MCFixup::create(Fixup.getOffset(), nullptr, ELF::R_RISCV_RELAX);
      MCValue RelaxTarget = MCValue::get(nullptr);
      uint64_t RelaxValue;
      Asm->getWriter().recordRelocation(F, RelaxFixup, RelaxTarget, RelaxValue);
      }
    */
  }

  return false;
}

void T8xxAsmBackend::applyFixup(const MCFragment &F, const MCFixup &Fixup,
				const MCValue &Target, uint8_t *Data,
				uint64_t Value, bool IsResolved) {
  // This combination registers the relocation for writing to the object file.
  maybeAddReloc(F, Fixup, Target, Value, IsResolved);
  //addReloc(F, Fixup, Target, Value, IsResolved);

  if (!IsResolved)
    return;   // If it is not resolved, leave it as is

  unsigned NumBytes = getFixupKindNumBytes(Fixup.getKind());
  unsigned Offset = Fixup.getOffset();

  // For each byte of the fragment that the fixup touches, mask in the bits
  // from the fixup value. The Value has been "split up" into the
  // appropriate bitfields above.
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned Idx = Endian == llvm::endianness::little ? i : (NumBytes - 1) - i;
    Data[Idx] |= uint8_t((Value >> (i * 8)) & 0xff);
  }
}

std::unique_ptr<MCObjectTargetWriter>
T8xxAsmBackend::createObjectTargetWriter() const {
  return createT8xxELFObjectWriter(Is64Bit, OSABI);
}

MCAsmBackend *llvm::createT8xxAsmBackend(const Target &T,
					 const MCSubtargetInfo &STI,
					 const MCRegisterInfo &MRI,
					 const MCTargetOptions &Options) {
  const Triple &TT = STI.getTargetTriple();
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(TT.getOS());
  return new T8xxAsmBackend(STI, OSABI, Options);
}
