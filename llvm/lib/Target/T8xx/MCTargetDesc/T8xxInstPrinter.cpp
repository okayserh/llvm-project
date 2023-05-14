
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
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"


#define GET_INSTRUCTION_NAME
#define PRINT_ALIAS_INSTR
#include "T8xxGenAsmWriter.inc"

void T8xxInstPrinter::printRegName(raw_ostream &OS, MCRegister Reg) const
{
  OS << '%' << StringRef(getRegisterName(Reg)).lower();
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
  }
}

void T8xxInstPrinter::printOperand(const MCInst *MI, int opNum,
                                    const MCSubtargetInfo &STI,
                                    raw_ostream &O) {
  const MCOperand &MO = MI->getOperand (opNum);

  printf ("printOperand\n");
  
  if (MO.isReg()) {
    printRegName(O, MO.getReg());
    return ;
  }

  if (MO.isImm()) {
    O << (int)MO.getImm();
    return;
  }

  assert(MO.isExpr() && "Unknown operand kind in printOperand");
  MO.getExpr()->print(O, &MAI);
}



const char * condCodeToString(ISD::CondCode CC) {
  switch (CC) {
  default:
  case ISD::SETCC_INVALID:
  case ISD::SETFALSE:      //    0 0 0 0       Always false (always folded)
  case ISD::SETFALSE2:     //  1 X 0 0 0       Always false (always folded)
  case ISD::SETOEQ:        //    0 0 0 1       True if ordered and equal
  case ISD::SETOGT:        //    0 0 1 0       True if ordered and greater than
  case ISD::SETOGE:        //    0 0 1 1       True if ordered and greater than or equal
  case ISD::SETOLT:        //    0 1 0 0       True if ordered and less than
  case ISD::SETOLE:        //    0 1 0 1       True if ordered and less than or equal
  case ISD::SETONE:        //    0 1 1 0       True if ordered and operands are unequal
  case ISD::SETO:          //    0 1 1 1       True if ordered (no nans)
  case ISD::SETUO:         //    1 0 0 0       True if unordered: isnan(X) | isnan(Y)
  case ISD::SETUEQ:        //    1 0 0 1       True if unordered or equal
  case ISD::SETUGT:        //    1 0 1 0       True if unordered or greater than
  case ISD::SETUGE:        //    1 0 1 1       True if unordered, greater than, or equal
  case ISD::SETULT:        //    1 1 0 0       True if unordered or less than
  case ISD::SETULE:        //    1 1 0 1       True if unordered, less than, or equal
  case ISD::SETUNE:        //    1 1 1 0       True if unordered or not equal
    llvm_unreachable("Invalid or unsupported condition code");
    return nullptr;
    
  case ISD::SETTRUE:       //    1 1 1 1       Always true (always folded)
  case ISD::SETTRUE2:      //  1 X 1 1 1       Always true (always folded)
    return "";
  
  // Don't care operations: undefined if the input is a nan.
  case ISD::SETEQ:         //  1 X 0 0 1       True if equal
    return "eq";
  case ISD::SETGT:         //  1 X 0 1 0       True if greater than
    return "gt";
  case ISD::SETGE:         //  1 X 0 1 1       True if greater than or equal
    return "ge";
  case ISD::SETLT:         //  1 X 1 0 0       True if less than
    return "lt";
  case ISD::SETLE:         //  1 X 1 0 1       True if less than or equal
    return "le";
  case ISD::SETNE:         //  1 X 1 1 0       True if not equal
    return "ne";
  }
}

void T8xxInstPrinter::printCondCode(const MCInst *MI, int opNum, const MCSubtargetInfo &STI,
                       raw_ostream &OS)
{
  const MCOperand &Op = MI->getOperand(opNum);
  ISD::CondCode CC = (ISD::CondCode)Op.getImm();
  const char *Str = condCodeToString(CC);
  OS << Str;
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
  if (Op1.isReg() && Op1.getReg() != T8xx::R0) {
    printOperand(MI, opNum, STI, O);
    PrintedFirstOperand = true;
  }

  // Skip the second operand iff it adds nothing (literal 0 or %g0) and we've
  // already printed the first one
  const bool SkipSecondOperand =
      PrintedFirstOperand && ((Op2.isReg() && Op2.getReg() == T8xx::R0) ||
                              (Op2.isImm() && Op2.getImm() == 0));

  if (!SkipSecondOperand) {
    if (PrintedFirstOperand)
      O << '+';
    printOperand(MI, opNum + 1, STI, O);
  }
}


// Print a 'memsrc' operand which is a (Register, Offset) pair.
void T8xxInstPrinter::printAddrModeMemSrc(const MCInst *MI, int OpNum,
					  const MCSubtargetInfo &STI,
					  raw_ostream &O) {
  const MCOperand &Op1 = MI->getOperand(OpNum);
  const MCOperand &Op2 = MI->getOperand(OpNum + 1);
  O << "[";
  printRegName(O, Op1.getReg());

  unsigned Offset = Op2.getImm();
  if (Offset) {
    O << ", #" << Offset;
  }
  O << "]";
}


// Print a 'memsrc' operand which is a (Register, Offset) pair.
/*
void T8xxInstPrinter::printAddrModeStack(const MCInst *MI, int OpNum,
					  const MCSubtargetInfo &STI,
					  raw_ostream &O) {
  O << "Stack";
}
*/

