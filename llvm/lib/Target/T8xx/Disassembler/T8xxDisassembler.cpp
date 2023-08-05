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
  T8xx::R0,  T8xx::R1,  T8xx::R2,  T8xx::R3,
  T8xx::R4,  T8xx::R5,  T8xx::R6,  T8xx::R7,
  T8xx::R8,  T8xx::R9,  T8xx::R10,  T8xx::R11,
  T8xx::R12,  T8xx::R13,  T8xx::R14,  T8xx::R15};

static DecodeStatus DecodeIntRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;
  unsigned Reg = IntRegDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}


#include "T8xxGenDisassemblerTables.inc"

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
  bool isLittleEndian = getContext().getAsmInfo()->isLittleEndian();
  DecodeStatus Result =
      readInstruction32(Bytes, Address, Size, Insn, isLittleEndian);
  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  // Calling the auto-generated decoder function.
  /* TODO: DecoderTable is not created?
  if (STI.getFeatureBits()[T8xx::FeatureV9])
  {
    Result = decodeInstruction(DecoderTableT8xxV932, Instr, Insn, Address, this, STI);
  }
  else
  {
    Result = decodeInstruction(DecoderTableT8xxV832, Instr, Insn, Address, this, STI);
  }
  if (Result != MCDisassembler::Fail)
    return Result;

  Result =
      decodeInstruction(DecoderTableT8xx32, Instr, Insn, Address, this, STI);

  if (Result != MCDisassembler::Fail) {
    Size = 4;
    return Result;
  }
  */
  return MCDisassembler::Fail;
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
