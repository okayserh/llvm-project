
//===-- T8xxInstPrinter.cpp - Convert T8xx MCInst to assembly syntax -----==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an T8xx MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "T8xxInstPrinter.h"
#include "T8xx.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

// The generated AsmMatcher T8xxGenAsmWriter uses "T8xx" as the target
// namespace. But SPARC backend uses "SP" as its namespace.
namespace llvm {
namespace T8xx {
  using namespace T8;
}
}

#define GET_INSTRUCTION_NAME
#define PRINT_ALIAS_INSTR
#include "T8xxGenAsmWriter.inc"

bool T8xxInstPrinter::isV9(const MCSubtargetInfo &STI) const {
  return (STI.getFeatureBits()[T8xx::FeatureV9]) != 0;
}

void T8xxInstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const
{
  OS << '%' << StringRef(getRegisterName(RegNo)).lower();
}

void T8xxInstPrinter::printInst(const MCInst *MI, uint64_t Address,
                                 StringRef Annot, const MCSubtargetInfo &STI,
                                 raw_ostream &O) {
  if (!printAliasInstr(MI, Address, STI, O) &&
      !printT8xxAliasInstr(MI, STI, O))
    printInstruction(MI, Address, STI, O);
  printAnnotation(O, Annot);
}

bool T8xxInstPrinter::printT8xxAliasInstr(const MCInst *MI,
                                            const MCSubtargetInfo &STI,
                                            raw_ostream &O) {
  switch (MI->getOpcode()) {
  default: return false;
  case T8::JMPLrr:
  case T8::JMPLri: {
    if (MI->getNumOperands() != 3)
      return false;
    if (!MI->getOperand(0).isReg())
      return false;
    switch (MI->getOperand(0).getReg()) {
    default: return false;
    case T8::G0: // jmp $addr | ret | retl
      if (MI->getOperand(2).isImm() &&
          MI->getOperand(2).getImm() == 8) {
        switch(MI->getOperand(1).getReg()) {
        default: break;
        case T8::I7: O << "\tret"; return true;
        case T8::O7: O << "\tretl"; return true;
        }
      }
      O << "\tjmp "; printMemOperand(MI, 1, STI, O);
      return true;
    case T8::O7: // call $addr
      O << "\tcall "; printMemOperand(MI, 1, STI, O);
      return true;
    }
  }
  case T8::V9FCMPS:  case T8::V9FCMPD:  case T8::V9FCMPQ:
  case T8::V9FCMPES: case T8::V9FCMPED: case T8::V9FCMPEQ: {
    if (isV9(STI)
        || (MI->getNumOperands() != 3)
        || (!MI->getOperand(0).isReg())
        || (MI->getOperand(0).getReg() != T8::FCC0))
      return false;
    // if V8, skip printing %fcc0.
    switch(MI->getOpcode()) {
    default:
    case T8::V9FCMPS:  O << "\tfcmps "; break;
    case T8::V9FCMPD:  O << "\tfcmpd "; break;
    case T8::V9FCMPQ:  O << "\tfcmpq "; break;
    case T8::V9FCMPES: O << "\tfcmpes "; break;
    case T8::V9FCMPED: O << "\tfcmped "; break;
    case T8::V9FCMPEQ: O << "\tfcmpeq "; break;
    }
    printOperand(MI, 1, STI, O);
    O << ", ";
    printOperand(MI, 2, STI, O);
    return true;
  }
  }
}

void T8xxInstPrinter::printOperand(const MCInst *MI, int opNum,
                                    const MCSubtargetInfo &STI,
                                    raw_ostream &O) {
  const MCOperand &MO = MI->getOperand (opNum);

  if (MO.isReg()) {
    printRegName(O, MO.getReg());
    return ;
  }

  if (MO.isImm()) {
    switch (MI->getOpcode()) {
      default:
        O << (int)MO.getImm();
        return;

      case T8::TICCri: // Fall through
      case T8::TICCrr: // Fall through
      case T8::TRAPri: // Fall through
      case T8::TRAPrr: // Fall through
      case T8::TXCCri: // Fall through
      case T8::TXCCrr: // Fall through
        // Only seven-bit values up to 127.
        O << ((int) MO.getImm() & 0x7f);
        return;
    }
  }

  assert(MO.isExpr() && "Unknown operand kind in printOperand");
  MO.getExpr()->print(O, &MAI);
}

void T8xxInstPrinter::printMemOperand(const MCInst *MI, int opNum,
                                       const MCSubtargetInfo &STI,
                                       raw_ostream &O, const char *Modifier) {
  // If this is an ADD operand, emit it like normal operands.
  if (Modifier && !strcmp(Modifier, "arith")) {
    printOperand(MI, opNum, STI, O);
    O << ", ";
    printOperand(MI, opNum + 1, STI, O);
    return;
  }

  const MCOperand &Op1 = MI->getOperand(opNum);
  const MCOperand &Op2 = MI->getOperand(opNum + 1);

  bool PrintedFirstOperand = false;
  if (Op1.isReg() && Op1.getReg() != T8::G0) {
    printOperand(MI, opNum, STI, O);
    PrintedFirstOperand = true;
  }

  // Skip the second operand iff it adds nothing (literal 0 or %g0) and we've
  // already printed the first one
  const bool SkipSecondOperand =
      PrintedFirstOperand && ((Op2.isReg() && Op2.getReg() == T8::G0) ||
                              (Op2.isImm() && Op2.getImm() == 0));

  if (!SkipSecondOperand) {
    if (PrintedFirstOperand)
      O << '+';
    printOperand(MI, opNum + 1, STI, O);
  }
}

void T8xxInstPrinter::printCCOperand(const MCInst *MI, int opNum,
                                      const MCSubtargetInfo &STI,
                                      raw_ostream &O) {
  int CC = (int)MI->getOperand(opNum).getImm();
  switch (MI->getOpcode()) {
  default: break;
  case T8::FBCOND:
  case T8::FBCONDA:
  case T8::BPFCC:
  case T8::BPFCCA:
  case T8::BPFCCNT:
  case T8::BPFCCANT:
  case T8::MOVFCCrr:  case T8::V9MOVFCCrr:
  case T8::MOVFCCri:  case T8::V9MOVFCCri:
  case T8::FMOVS_FCC: case T8::V9FMOVS_FCC:
  case T8::FMOVD_FCC: case T8::V9FMOVD_FCC:
  case T8::FMOVQ_FCC: case T8::V9FMOVQ_FCC:
    // Make sure CC is a fp conditional flag.
    CC = (CC < SPCC::FCC_BEGIN) ? (CC + SPCC::FCC_BEGIN) : CC;
    break;
  case T8::CBCOND:
  case T8::CBCONDA:
    // Make sure CC is a cp conditional flag.
    CC = (CC < SPCC::CPCC_BEGIN) ? (CC + SPCC::CPCC_BEGIN) : CC;
    break;
  case T8::MOVRri:
  case T8::MOVRrr:
  case T8::FMOVRS:
  case T8::FMOVRD:
  case T8::FMOVRQ:
    // Make sure CC is a register conditional flag.
    CC = (CC < SPCC::REG_BEGIN) ? (CC + SPCC::REG_BEGIN) : CC;
    break;
  }
  O << SPARCCondCodeToString((SPCC::CondCodes)CC);
}

bool T8xxInstPrinter::printGetPCX(const MCInst *MI, unsigned opNum,
                                   const MCSubtargetInfo &STI,
                                   raw_ostream &O) {
  llvm_unreachable("FIXME: Implement T8xxInstPrinter::printGetPCX.");
  return true;
}

void T8xxInstPrinter::printMembarTag(const MCInst *MI, int opNum,
                                      const MCSubtargetInfo &STI,
                                      raw_ostream &O) {
  static const char *const TagNames[] = {
      "#LoadLoad",  "#StoreLoad", "#LoadStore", "#StoreStore",
      "#Lookaside", "#MemIssue",  "#Sync"};

  unsigned Imm = MI->getOperand(opNum).getImm();

  if (Imm > 127) {
    O << Imm;
    return;
  }

  bool First = true;
  for (unsigned i = 0; i < std::size(TagNames); i++) {
    if (Imm & (1 << i)) {
      O << (First ? "" : " | ") << TagNames[i];
      First = false;
    }
  }
}
