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
  T8::G0,  T8::G1,  T8::G2,  T8::G3,
  T8::G4,  T8::G5,  T8::G6,  T8::G7,
  T8::O0,  T8::O1,  T8::O2,  T8::O3,
  T8::O4,  T8::O5,  T8::O6,  T8::O7,
  T8::L0,  T8::L1,  T8::L2,  T8::L3,
  T8::L4,  T8::L5,  T8::L6,  T8::L7,
  T8::I0,  T8::I1,  T8::I2,  T8::I3,
  T8::I4,  T8::I5,  T8::I6,  T8::I7 };

static const unsigned FPRegDecoderTable[] = {
  T8::F0,   T8::F1,   T8::F2,   T8::F3,
  T8::F4,   T8::F5,   T8::F6,   T8::F7,
  T8::F8,   T8::F9,   T8::F10,  T8::F11,
  T8::F12,  T8::F13,  T8::F14,  T8::F15,
  T8::F16,  T8::F17,  T8::F18,  T8::F19,
  T8::F20,  T8::F21,  T8::F22,  T8::F23,
  T8::F24,  T8::F25,  T8::F26,  T8::F27,
  T8::F28,  T8::F29,  T8::F30,  T8::F31 };

static const unsigned DFPRegDecoderTable[] = {
  T8::D0,   T8::D16,  T8::D1,   T8::D17,
  T8::D2,   T8::D18,  T8::D3,   T8::D19,
  T8::D4,   T8::D20,  T8::D5,   T8::D21,
  T8::D6,   T8::D22,  T8::D7,   T8::D23,
  T8::D8,   T8::D24,  T8::D9,   T8::D25,
  T8::D10,  T8::D26,  T8::D11,  T8::D27,
  T8::D12,  T8::D28,  T8::D13,  T8::D29,
  T8::D14,  T8::D30,  T8::D15,  T8::D31 };

static const unsigned QFPRegDecoderTable[] = {
  T8::Q0,  T8::Q8,   ~0U,  ~0U,
  T8::Q1,  T8::Q9,   ~0U,  ~0U,
  T8::Q2,  T8::Q10,  ~0U,  ~0U,
  T8::Q3,  T8::Q11,  ~0U,  ~0U,
  T8::Q4,  T8::Q12,  ~0U,  ~0U,
  T8::Q5,  T8::Q13,  ~0U,  ~0U,
  T8::Q6,  T8::Q14,  ~0U,  ~0U,
  T8::Q7,  T8::Q15,  ~0U,  ~0U } ;

static const unsigned FCCRegDecoderTable[] = {
  T8::FCC0, T8::FCC1, T8::FCC2, T8::FCC3 };

static const unsigned ASRRegDecoderTable[] = {
  T8::Y,     T8::ASR1,  T8::ASR2,  T8::ASR3,
  T8::ASR4,  T8::ASR5,  T8::ASR6,  T8::ASR7,
  T8::ASR8,  T8::ASR9,  T8::ASR10, T8::ASR11,
  T8::ASR12, T8::ASR13, T8::ASR14, T8::ASR15,
  T8::ASR16, T8::ASR17, T8::ASR18, T8::ASR19,
  T8::ASR20, T8::ASR21, T8::ASR22, T8::ASR23,
  T8::ASR24, T8::ASR25, T8::ASR26, T8::ASR27,
  T8::ASR28, T8::ASR29, T8::ASR30, T8::ASR31};

static const unsigned PRRegDecoderTable[] = {
  T8::TPC, T8::TNPC, T8::TSTATE, T8::TT, T8::TICK, T8::TBA, T8::PSTATE,
  T8::TL, T8::PIL, T8::CWP, T8::CANSAVE, T8::CANRESTORE, T8::CLEANWIN,
  T8::OTHERWIN, T8::WSTATE, T8::PC
};

static const uint16_t IntPairDecoderTable[] = {
  T8::G0_G1, T8::G2_G3, T8::G4_G5, T8::G6_G7,
  T8::O0_O1, T8::O2_O3, T8::O4_O5, T8::O6_O7,
  T8::L0_L1, T8::L2_L3, T8::L4_L5, T8::L6_L7,
  T8::I0_I1, T8::I2_I3, T8::I4_I5, T8::I6_I7,
};

static const unsigned CPRegDecoderTable[] = {
  T8::C0,  T8::C1,  T8::C2,  T8::C3,
  T8::C4,  T8::C5,  T8::C6,  T8::C7,
  T8::C8,  T8::C9,  T8::C10, T8::C11,
  T8::C12, T8::C13, T8::C14, T8::C15,
  T8::C16, T8::C17, T8::C18, T8::C19,
  T8::C20, T8::C21, T8::C22, T8::C23,
  T8::C24, T8::C25, T8::C26, T8::C27,
  T8::C28, T8::C29, T8::C30, T8::C31
};


static const uint16_t CPPairDecoderTable[] = {
  T8::C0_C1,   T8::C2_C3,   T8::C4_C5,   T8::C6_C7,
  T8::C8_C9,   T8::C10_C11, T8::C12_C13, T8::C14_C15,
  T8::C16_C17, T8::C18_C19, T8::C20_C21, T8::C22_C23,
  T8::C24_C25, T8::C26_C27, T8::C28_C29, T8::C30_C31
};

static DecodeStatus DecodeIntRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;
  unsigned Reg = IntRegDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeI64RegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  return DecodeIntRegsRegisterClass(Inst, RegNo, Address, Decoder);
}

// This is used for the type "ptr_rc", which is either IntRegs or I64Regs
// depending on T8xxRegisterInfo::getPointerRegClass.
static DecodeStatus DecodePointerLikeRegClass0(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  return DecodeIntRegsRegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeFPRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                              uint64_t Address,
                                              const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;
  unsigned Reg = FPRegDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeDFPRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;
  unsigned Reg = DFPRegDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeQFPRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;

  unsigned Reg = QFPRegDecoderTable[RegNo];
  if (Reg == ~0U)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus
DecodeCoprocRegsRegisterClass(MCInst &Inst, unsigned RegNo, uint64_t Address,
                              const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;
  unsigned Reg = CPRegDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeFCCRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  if (RegNo > 3)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(FCCRegDecoderTable[RegNo]));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeASRRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(ASRRegDecoderTable[RegNo]));
  return MCDisassembler::Success;
}

static DecodeStatus DecodePRRegsRegisterClass(MCInst &Inst, unsigned RegNo,
                                              uint64_t Address,
                                              const MCDisassembler *Decoder) {
  if (RegNo >= std::size(PRRegDecoderTable))
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(PRRegDecoderTable[RegNo]));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeIntPairRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address,
                                               const MCDisassembler *Decoder) {
  DecodeStatus S = MCDisassembler::Success;

  if (RegNo > 31)
    return MCDisassembler::Fail;

  if ((RegNo & 1))
    S = MCDisassembler::SoftFail;

  unsigned RegisterPair = IntPairDecoderTable[RegNo/2];
  Inst.addOperand(MCOperand::createReg(RegisterPair));
  return S;
}

static DecodeStatus
DecodeCoprocPairRegisterClass(MCInst &Inst, unsigned RegNo, uint64_t Address,
                              const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;

  unsigned RegisterPair = CPPairDecoderTable[RegNo/2];
  Inst.addOperand(MCOperand::createReg(RegisterPair));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeCall(MCInst &Inst, unsigned insn, uint64_t Address,
                               const MCDisassembler *Decoder);
static DecodeStatus DecodeSIMM13(MCInst &Inst, unsigned insn, uint64_t Address,
                                 const MCDisassembler *Decoder);

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

static DecodeStatus DecodeSIMM13(MCInst &MI, unsigned insn, uint64_t Address,
                                 const MCDisassembler *Decoder) {
  unsigned tgt = SignExtend32<13>(fieldFromInstruction(insn, 0, 13));
  MI.addOperand(MCOperand::createImm(tgt));
  return MCDisassembler::Success;
}
