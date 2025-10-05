//===- T8xxDisassembler.cpp - Disassembler for T8xx -----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the T8xx Disassembler.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T8xxMCTargetDesc.h"
#include "TargetInfo/T8xxTargetInfo.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDecoderOps.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "t8xx-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

/// A disassembler class for T8xx.
class T8xxDisassembler : public MCDisassembler {
public:
  T8xxDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  virtual ~T8xxDisassembler() = default;

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;
};
}

static MCDisassembler *createT8xxDisassembler(const Target &T,
                                               const MCSubtargetInfo &STI,
                                               MCContext &Ctx) {
  return new T8xxDisassembler(STI, Ctx);
}


extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeT8xxDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheT8xxTarget(),
                                         createT8xxDisassembler);
}

static const unsigned IntRegDecoderTable[] = {
  T8xx::AREG,  T8xx::BREG,  T8xx::CREG};

static DecodeStatus DecodeIntRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;
  unsigned Reg = IntRegDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}


static DecodeStatus decodeImm(MCInst &Inst, unsigned Insn, uint64_t Address,
			      const MCDisassembler *Decoder);

static DecodeStatus decodeRegStack(MCInst &Inst, unsigned Insn, uint64_t Address,
				   const MCDisassembler *Decoder);


#include "T8xxGenDisassemblerTables.inc"


static DecodeStatus decodeImm(MCInst &Inst, unsigned Insn, uint64_t Address,
			      const MCDisassembler *Decoder)
{
  unsigned opc = fieldFromInstruction(Insn, 4, 4);
  unsigned imm = fieldFromInstruction(Insn, 0, 4);

  switch (opc)
    {
    case 0x0:  // J
    case 0x9:  // CALL
      Inst.addOperand(MCOperand::createImm(imm));
      break;
    case 0x1: // LDLP
    case 0x7: // LDL
    case 0xD: // STL
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createReg(T8xx::WPTR));
      Inst.addOperand(MCOperand::createImm(imm));
      break;
    case 0x3:  // LDNL
    case 0x5:  // LDNLP
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createImm(imm));
      break;
    case 0x4:  // LDC
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createImm(imm));
      break;
    case 0xC:  // EQC
    case 0x8:  // ADC
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createImm(imm));
      break;
    case 0xA:  // CJ
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createImm(imm));
      break;
    case 0xB: // AJW
      Inst.addOperand(MCOperand::createImm(imm));
      break;
    case 0xE:  // STNL
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createReg(T8xx::AREG));
      Inst.addOperand(MCOperand::createImm(imm));
      break;
      // Note: 0xF is the "OP" instruction, which encodes additional instructions
    }

  return MCDisassembler::Success;
}

static DecodeStatus decodeRegStack(MCInst &Inst, unsigned Insn, uint64_t Address,
				   const MCDisassembler *Decoder)
{
  Inst.addOperand(MCOperand::createReg(T8xx::AREG));
  Inst.addOperand(MCOperand::createReg(T8xx::BREG));
  return MCDisassembler::Success;
}


/// Read four bytes from the ArrayRef and return 32 bit word.
static DecodeStatus readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn,
                                      bool IsLittleEndian) {
  // We want to read exactly 4 Bytes of data.
  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Insn = IsLittleEndian
             ? (Bytes[0] << 0) | (Bytes[1] << 8) | (Bytes[2] << 16) |
                   (Bytes[3] << 24)
             : (Bytes[3] << 0) | (Bytes[2] << 8) | (Bytes[1] << 16) |
                   (Bytes[0] << 24);

  return MCDisassembler::Success;
}

DecodeStatus T8xxDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                               ArrayRef<uint8_t> Bytes,
                                               uint64_t Address,
                                               raw_ostream &CStream) const {
  uint32_t Insn;
  DecodeStatus Result;

  // Collect nfix / pfix instructions
  uint32_t ORegBuf = 0;
  unsigned int i = 0;
  while (((Bytes[i] >> 4) == 0x6) ||
	 ((Bytes[i] >> 4) == 0x2))
    {
      ORegBuf |= (Bytes[i] & 0xF);
      if ((Bytes[i] >> 4) == 0x6)
	ORegBuf = ~ORegBuf;
      ORegBuf <<= 4;
      ++i;
    }

  // Calling the auto-generated decoder function.
  Insn = Bytes[i];
  Address += i;
  Size = i;

  //  printf ("Insn %i  Address %i   Size %i\n", Insn, Address, Size);
  if ((Bytes[i] >> 4) != 0xF)
    {
      // For the direct instructions, first decode just the 8 bit
      // direct instruction. Add the pfix/nfix stuff afterwards
      Result = decodeInstruction(DecoderTableT8xx8, Instr, Insn, Address, this, STI);
      MCInst::iterator mcopit;
      for (mcopit = Instr.begin (); mcopit != Instr.end (); ++mcopit)
	if (mcopit->isImm ())
	  {
	    ORegBuf |= (mcopit->getImm () & 0xF);
	    mcopit->setImm (*((int32_t *)(&ORegBuf)));
	  }      

      // Check for "fpentry" instructions. Those start with an "LDC"
      // followed by "fpentry" (2A FB)
      if ((Instr.getOpcode() == T8xx::LDC) &&
	  (Bytes[i+1] == 0x2A) &&
	  (Bytes[i+2] == 0xFB))
	{
	  if (Instr.getOperand(1).getImm () > 0xF)
	    {
	      Insn = (((uint64_t)Bytes[i-1]) << 24) +
		(((uint64_t)Bytes[i]) << 16) +
		(((uint64_t)Bytes[i+1]) << 8) +
		((uint64_t)Bytes[i+2]);
	      Result = decodeInstruction(DecoderTableT8xx32, Instr, Insn, Address, this, STI);
	    }
	  else
	    {
	      Insn = (((uint64_t)Bytes[i]) << 16) +
		(((uint64_t)Bytes[i+1]) << 8) +
		((uint64_t)Bytes[i+2]);
	      Result = decodeInstruction(DecoderTableT8xx24, Instr, Insn, Address, this, STI);
	    }
	  Size = i + 3;
	}
      else
	Size = i+1;
    }
  else
    {
      if (ORegBuf == 0)
	{
	  Result = decodeInstruction(DecoderTableT8xx8, Instr, Insn, Address, this, STI);
	  Size = i + 1;
	}
      else
	{
	  Insn = (((uint64_t)Bytes[i-1]) << 8) + ((uint64_t)Bytes[i]);
	  Result = decodeInstruction(DecoderTableT8xx16, Instr, Insn, Address, this, STI);
	  if (Result != MCDisassembler::Fail)
	    Size = i+1;
	}
    }

  return Result;
}

static bool tryAddingSymbolicOperand(int64_t Value, bool isBranch,
                                     uint64_t Address, uint64_t Offset,
                                     uint64_t Width, MCInst &MI,
                                     const MCDisassembler *Decoder) {
  return Decoder->tryAddingSymbolicOperand(MI, Value, Address, isBranch, Offset,
                                           Width, /*InstSize=*/4);
}

static DecodeStatus DecodeCall(MCInst &MI, unsigned insn, uint64_t Address,
                               const MCDisassembler *Decoder) {
  unsigned tgt = fieldFromInstruction(insn, 0, 30);
  tgt <<= 2;
  if (!tryAddingSymbolicOperand(tgt+Address, false, Address,
                                0, 30, MI, Decoder))
    MI.addOperand(MCOperand::createImm(tgt));
  return MCDisassembler::Success;
}


static DecodeStatus DecodeWPtrSrcOperand(MCInst &MI, unsigned insn, uint64_t Address,
                               const MCDisassembler *Decoder) {
  // TODO: Needs implementation
  return MCDisassembler::Success;
}
