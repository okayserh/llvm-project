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
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
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

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

  // getBinaryCodeForInstr - TableGen'erated function for getting the
  // binary encoding for an instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
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
  void EmitByte(unsigned char C, raw_ostream &OS) const {
    OS << (char)C;
  }

  void EmitConstant(uint64_t Val, unsigned Size, raw_ostream &OS) const {
    // Output the constant in little endian byte order.
    for (unsigned i = 0; i != Size; ++i) {
      unsigned Shift = IsLittleEndian ? i * 8 : (Size - 1 - i) * 8;
      EmitByte((Val >> Shift) & 0xff, OS);
    }
  }


};

} // end anonymous namespace

void T8xxMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  /*
  uint64_t TSFlags = Desc.TSFlags;
  if ((TSFlags & ARMII::FormMask) == ARMII::Pseudo)
    return;
  */
  
  int Size = Desc.getSize ();  
  uint64_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);

  // T8xx immediate functions
  if ((Size == 1) && ((Bits & 0xF) == 0))
    {
      for (auto MO = MI.begin (); MO != MI.end (); ++MO)
	{
	  if (MO->isImm ())
	    {
	      int64_t imm = MO->getImm ();
	      // Correction for workspace operations
	      // TODO: Check ldnl / stnl
	      // TODO: Maybe move this correction to the eliminateFrameIndex!?
	      if ((MI.getOpcode() == T8xx::STL) ||
		  (MI.getOpcode() == T8xx::LDL) ||
		  (MI.getOpcode() == T8xx::LDLP))
		imm /= 4;

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
		    EmitConstant (0x20 & (imm_res >> (4 * i)), 1, OS);

		  // First nonzero bits discovered
		  if ((imm_res || ((imm < 0) && i == 1)) && !enc_beg)
		    {
		      enc_beg = true;
		      if (imm < 0)
			{
			  EmitConstant (0x60 | (imm_res >> (4 * i)), 1, OS);
			  imm_dec = ~imm_dec;			  
			}
		      else
			  EmitConstant (0x20 | (imm_res >> (4 * i)), 1, OS);			
		    }
		}

	      Bits |= imm_dec & 0xF;
	      printf ("Opcode %u   Imm %i\n", Bits, imm);
	    }

	  if (MO->isExpr ())
	    {
	      /*
	      const MCExpr *Expr = MO->getExpr();
	      if (const T8xxMCExpr *SExpr = dyn_cast<T8xxMCExpr>(Expr)) {
		MCFixupKind Kind = (MCFixupKind)SExpr->getFixupKind();
		Fixups.push_back(MCFixup::create(0, Expr, Kind));
		//		return 0;
	      }
	      */

	      printf ("Opcode %u, Expression found\n", Bits);
	      MO->getExpr ()->dump ();
	    }
	}
    }

  EmitConstant(Bits, Size, OS);

  ++MCNumEmitted;  // Keep track of the # of mi's emitted.
}

unsigned T8xxMCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());

  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr());
  const MCExpr *Expr = MO.getExpr();
  if (const T8xxMCExpr *SExpr = dyn_cast<T8xxMCExpr>(Expr)) {
    MCFixupKind Kind = (MCFixupKind)SExpr->getFixupKind();
    Fixups.push_back(MCFixup::create(0, Expr, Kind));
    return 0;
  }

  int64_t Res;
  if (Expr->evaluateAsAbsolute(Res))
    return Res;

  llvm_unreachable("Unhandled expression!");
  return 0;
}


unsigned T8xxMCCodeEmitter::
getCallTargetOpValue(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);
  const MCExpr *Expr = MO.getExpr();
  const T8xxMCExpr *SExpr = dyn_cast<T8xxMCExpr>(Expr);

  printf ("FIXUP: getCallTargetOpValue\n");

  // Note: Code copied from AVRMCCodeEmitter.cpp.
  // It seems that it differentiates two cases, one is an expression
  // (presumably a symbol) which justifies the insertion of a fixup for
  // later handling, the second being an immediate number, where
  // just some bitshifting happens (in case of the AVR microcontroller).

  if (MO.isExpr()) {
    printf ("FIXUP: Expression\n");
    Fixups.push_back(
		     MCFixup::create(0, MO.getExpr(), MCFixupKind(T8xx::fixup_t8xx_jump), MI.getLoc()));
    return 0;
  }

  assert(MO.isImm());
  printf ("No FIXUP: Immediate\n");

  MCFixupKind Kind = MCFixupKind(SExpr->getFixupKind());
  Fixups.push_back(MCFixup::create(0, Expr, Kind));
  return 0;
}


#include "T8xxGenMCCodeEmitter.inc"

MCCodeEmitter *llvm::createT8xxMCCodeEmitter(const MCInstrInfo &MCII,
                                              MCContext &Ctx) {
  // Endianess to be determined. In "T8xxTargetMachine", little endian "e" is specified
  // big endian would be "E".
  return new T8xxMCCodeEmitter(MCII, Ctx, false);
}
