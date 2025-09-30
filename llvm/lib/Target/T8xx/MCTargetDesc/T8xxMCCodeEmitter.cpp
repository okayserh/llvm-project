//===-- T8xxMCCodeEmitter.cpp - Convert T8xx code to machine code -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the T8xxMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T8xxFixupKinds.h"
#include "T8xxMCExpr.h"
#include "T8xxMCTargetDesc.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/TargetParser/SubtargetFeature.h"
#include <cassert>
#include <cstdint>

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

STATISTIC(MCNumEmitted, "Number of MC instructions emitted");

namespace {

class T8xxMCCodeEmitter : public MCCodeEmitter {
  const MCInstrInfo &MCII;
  MCContext &Ctx;
  bool IsLittleEndian;

public:
  T8xxMCCodeEmitter(const MCInstrInfo &mcii, MCContext &ctx, bool IsLittle)
      : MCII(mcii), Ctx(ctx), IsLittleEndian (IsLittle) {}
  T8xxMCCodeEmitter(const T8xxMCCodeEmitter &) = delete;
  T8xxMCCodeEmitter &operator=(const T8xxMCCodeEmitter &) = delete;
  ~T8xxMCCodeEmitter() override = default;

  void encodeInstruction(const MCInst &MI, SmallVectorImpl<char> &CB,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

  // getBinaryCodeForInstr - TableGen'erated function for getting the
  // binary encoding for an instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  // Taken from MipsMCCodeEmitter.h. Method to treat Expression in a unified
  // way.
  unsigned getExprOpValue(const MCInst &MI,
			  const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

  /// getMachineOpValue - Return binary encoding of operand. If the machine
  /// operand requires relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;
  unsigned getCallTargetOpValue(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  // Taken from ARMMCCodeEmitter
  void EmitByte(unsigned char C, SmallVectorImpl<char> &CB) const {
    support::endian::write<uint8_t>(CB, static_cast<uint8_t>(C),
				    llvm::endianness::big);
  }

  // Note: Instructions need to be emitted in little endian order
  // However, the Transputer is generally big endian!!!

  void EmitConstant(uint64_t Val, unsigned Size, SmallVectorImpl<char> &CB) const {
    // Output the constant in big endian byte order.
    for (unsigned i = 0; i != Size; ++i) {
      unsigned Shift = IsLittleEndian ? i * 8 : (Size - 1 - i) * 8;
      EmitByte((Val >> Shift) & 0xff, CB);
    }
  }


};

} // end anonymous namespace

void T8xxMCCodeEmitter::encodeInstruction(const MCInst &MI,
                                           SmallVectorImpl<char> &CB,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());

  int Size = Desc.getSize ();

  // [OKH] "getBinaryCodeForInstr" is automatically created and calls either the
  // encoder method (getCallTargetOpValue) or the standard method
  // (getMachineOpValue) to combine the operand with the instruction code.
  uint64_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);

  // [OKH] Note: The algorithm in "getBinaryCodeForInstr" works like this:
  // It first takes the "empty" bitpattern for the instruction as base
  // set (in a 64 bit unsigned int). Then the encoder method for the
  // relevant operand is called. The result of the encoder method is
  // "trimmed" (i.e. &= UINT64_C(15) for JUMP/CJ) and the trimmed value
  // is then merged into the initial bitpattern for the instruction
  //
  // Note: Furthermore, it seems like the "Encoders" can do certain things
  // to introduce fixups for example.
  //
  // Note: Currently, the CJ/CALLrel and JUMP instruction use the "getCallTargetOpValue"
  // encoder.
  // The other instructions are the 15 instructions which take an immediate.

  // The required prefix instructions will be generated in this method.
  //  printf ("Encoding %lu   Size %i\n", Bits, Size);
  //  MI.dump ();

  // T8xx immediate functions
  if ((Size == 1) && ((Bits & 0xFF) < 0xF0))
    {
      for (auto MO = MI.begin (); MO != MI.end (); ++MO)
	{
	  if (MO->isImm ())
	    {
	      int64_t imm = MO->getImm ();

	      // Add pfix and nfix as required
	      int i = 7;
	      uint32_t imm_dec = (imm < 0 ? (~imm) : imm) & 0xFFFFFFFFu;
	      uint32_t imm_and = 0xF0000000u;
	      bool enc_beg = false;
	      for (; i > 0; --i)
		{
		  // Determine 4 bits for pfix/nfix command
		  uint32_t imm_res = imm_dec & imm_and;
		  imm_and >>= 4;

		  // If we had some nonzero bits before
		  // continue padding with "pfix" instructions
		  if (enc_beg)
		    support::endian::write<uint8_t>(CB, static_cast<uint8_t> (0x20 | (imm_res >> (4 * i))),
						    llvm::endianness::big);

		  // First nonzero bits discovered
		  if ((imm_res || ((imm < 0) && i == 1)) && !enc_beg)
		    {
		      enc_beg = true;
		      if (imm < 0)
			{
			  support::endian::write<uint8_t>(CB, static_cast<uint8_t> (0x60 | (imm_res >> (4 * i))),
							  llvm::endianness::big);
			  imm_dec = ~imm_dec;
			}
		      else
			support::endian::write<uint8_t>(CB, static_cast<uint8_t> (0x20 | (imm_res >> (4 * i))),
							llvm::endianness::big);
		    }
		}

	      Bits |= imm_dec & 0xF;
	      //	      printf ("Opcode %lu   Imm %li\n", Bits, imm);
	    }

	  if (MO->isExpr ())
	    {
	      // Add 7 "pfix 0" instructions. These will later be adjusted
	      // during relocation with proper values.
	      for (int i = 0; i < 7; ++i)
		EmitByte (0x20, CB);

	      printf ("Opcode %lu, Expression found\n", Bits);
	      MO->getExpr ()->dump ();
	    }
	}
    }

  EmitConstant(Bits, Size, CB);

  ++MCNumEmitted;  // Keep track of the # of mi's emitted.
}


unsigned T8xxMCCodeEmitter::
getExprOpValue(const MCInst &MI,
	       const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
               const MCSubtargetInfo &STI) const {
  int64_t Res;

  if (Expr->evaluateAsAbsolute(Res))
    return Res;

  MCExpr::ExprKind Kind = Expr->getKind();
  if (Kind == MCExpr::Constant) {
    return cast<MCConstantExpr>(Expr)->getValue();
  }

  if (Kind == MCExpr::Binary) {
    const MCBinaryExpr *BinExpr = cast<MCBinaryExpr>(Expr);
    printf ("Encountered binary LHS %i   RHS %i\n",
	    BinExpr->getLHS()->getKind (),
	    BinExpr->getRHS()->getKind ());

    int64_t Res = 0;
    if (BinExpr->getLHS()->evaluateAsAbsolute(Res))
      printf ("LHS Eval %li\n", Res);
    if (BinExpr->getRHS()->evaluateAsAbsolute(Res))
      printf ("RHS Eval %li\n", Res);
    
    BinExpr->getLHS()->dump();
    BinExpr->getRHS()->dump();

    switch (BinExpr->getOpcode ())
      {
      case MCBinaryExpr::Opcode::Add:
	{
	  MCFixupKind Kind = MCFixupKind(T8xx::fixup_t8xx_addr_base);
	  Fixups.push_back(MCFixup::create(0, BinExpr->getLHS(), Kind, MI.getLoc()));
	  Kind = MCFixupKind(T8xx::fixup_t8xx_addr_add);
	  Fixups.push_back(MCFixup::create(0, BinExpr->getRHS(), Kind, MI.getLoc()));
	}
	break;
      case MCBinaryExpr::Opcode::Sub:
	{
	  MCFixupKind Kind = MCFixupKind(T8xx::fixup_t8xx_addr_base);
	  Fixups.push_back(MCFixup::create(0, BinExpr->getLHS(), Kind, MI.getLoc()));
	  Kind = MCFixupKind(T8xx::fixup_t8xx_addr_sub);
	  Fixups.push_back(MCFixup::create(0, BinExpr->getRHS(), Kind, MI.getLoc()));
	}
	break;

      default:
	Ctx.reportError(Expr->getLoc(), "unsupported binary expression");
      }
    

    /*
    unsigned Res =
        getExprOpValue(cast<MCBinaryExpr>(Expr)->getLHS(), Fixups, STI);
    Res += getExprOpValue(cast<MCBinaryExpr>(Expr)->getRHS(), Fixups, STI);
    return Res;
    */

    return 0;
  }

  if (Kind == MCExpr::Target) {
    const T8xxMCExpr *T8xxExpr = cast<T8xxMCExpr>(Expr);
    T8xx::Fixups FixupKind = T8xx::Fixups (0);

    switch (T8xxExpr->getKind ())
      {
      case T8xxMCExpr::VK_T8xx_None:
	printf ("Target Expr: T8xx_None\n");
	break;
      case T8xxMCExpr::VK_T8xx_IPTRREL:
	printf ("Target Expr: T8xx_IPTRREL\n");
	break;
      case T8xxMCExpr::VK_T8xx_SYMREL:
	{
	  FixupKind = T8xx::fixup_t8xx_pcrel_sym;
    	  Fixups.push_back(MCFixup::create(0, T8xxExpr, MCFixupKind(FixupKind), MI.getLoc()));
	  return (0);
	  printf ("Target Expr: T8xx_SYMREL\n");
	}
	break;
      case T8xxMCExpr::VK_T8xx_GLOBAL:
	{
	  FixupKind = T8xx::fixup_t8xx_addr;
    	  Fixups.push_back(MCFixup::create(0, T8xxExpr, MCFixupKind(FixupKind), MI.getLoc()));
	  return (0);
	}
	printf ("Target Expr: T8xx_GLOBAL\n");
	break;
      case T8xxMCExpr::VK_T8xx_GLOBAL_NPFIX:
	printf ("Target Expr: T8xx_GLOBAL_NPFIX\n");
	break;
      }

    // TODO:
    /*
    Mips::Fixups FixupKind = Mips::Fixups(0);
    switch (MipsExpr->getKind()) {
    case MipsMCExpr::MEK_None:
    case MipsMCExpr::MEK_Special:
      llvm_unreachable("Unhandled fixup kind!");
      break;
    case MipsMCExpr::MEK_TPREL_LO:
      FixupKind = isMicroMips(STI) ? Mips::fixup_MICROMIPS_TLS_TPREL_LO16
                                   : Mips::fixup_Mips_TPREL_LO;
      break;
    case MipsMCExpr::MEK_NEG:
      FixupKind =
          isMicroMips(STI) ? Mips::fixup_MICROMIPS_SUB : Mips::fixup_Mips_SUB;
      break;
    }
    Fixups.push_back(MCFixup::create(0, MipsExpr, MCFixupKind(FixupKind)));
    return 0;
    */
    llvm_unreachable("Unhandled expression!");
  }

  if (Kind == MCExpr::SymbolRef)
    {
      const MCSymbolRefExpr *SymRef = cast<MCSymbolRefExpr>(Expr);
      switch (SymRef->getKind ())
	{
	case MCSymbolRefExpr::VariantKind::VK_None:
	  {
	    MCFixupKind Kind = MCFixupKind(T8xx::fixup_t8xx_addr);
	    Fixups.push_back(MCFixup::create(0, Expr, Kind, MI.getLoc()));
	  }
	  break;
	default:
	  Ctx.reportError(Expr->getLoc(), "unhandled symbol type");
	}
    }
  return 0;
}


// Note: These method names are defines by setting the "EncoderMethod"

// Note: The "getMachineOpValue" method is used from some
// autogenerated stuff. (From T8xxGenMCCodeEmitter.inc).
// TODO: Not clear, where the method name is actually defined

// Note: The autogenerated code in "T8xxGenMCCodeEmitter.inc" does not
// work correctly since it does not consider the prefix and postfix
// instructions to setup the immediate value properly.

unsigned T8xxMCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());

  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr());

  return getExprOpValue(MI, MO.getExpr(),Fixups, STI);
}


// Note: These method names are defined by setting the "EncoderMethod"
// (This one is linked to "brtarget", which is used by BRimm2 and Bcc)
// This code just creates a fixup for a jump or conditional jump.
// The fixup is then replaced during relaxation by the Assembler.

unsigned T8xxMCCodeEmitter::
getCallTargetOpValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) return MO.getImm();
  assert(MO.isExpr() && "Unexpected branch target type!");

  const MCExpr *Expr = MO.getExpr();

  MCFixupKind Kind = MCFixupKind(T8xx::fixup_t8xx_jump);
  Fixups.push_back(MCFixup::create(0, Expr, Kind, MI.getLoc()));

  return 0;
}


#include "T8xxGenMCCodeEmitter.inc"

MCCodeEmitter *llvm::createT8xxMCCodeEmitter(const MCInstrInfo &MCII,
                                              MCContext &Ctx) {
  // Endianess to be determined. In "T8xxTargetMachine", big endian "E" is specified
  // little endian would be "e".
  return new T8xxMCCodeEmitter(MCII, Ctx, false);
}
