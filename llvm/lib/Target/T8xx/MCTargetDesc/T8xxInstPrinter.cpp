
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

void T8xxInstPrinter::printRegName(raw_ostream &OS, MCRegister Reg)
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
  //  MI->dump ();
  // printf ("Op Num %i No Ops %i\n", opNum, MI->getNumOperands ());

  const MCOperand &MO = MI->getOperand (opNum);

  //  printf ("printOperand\n");
  
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


// Print a 'memsrc' operand which is a (Register, Offset) pair.
void T8xxInstPrinter::printAddrModeMemSrc(const MCInst *MI, int OpNum,
					  const MCSubtargetInfo &STI,
					  raw_ostream &O) {
  //MI->dump ();
  //  printf ("Op Num %i No Ops %i\n", OpNum, MI->getNumOperands ());

  if ((OpNum + 1) < MI->getNumOperands ())
    {
      const MCOperand &Op1 = MI->getOperand(OpNum);
      const MCOperand &Op2 = MI->getOperand(OpNum + 1);
  
      assert((Op1.isReg() && (Op1.getReg() == T8xx::WPTR ||
			      Op1.getReg() == T8xx::AREG ||
			      Op1.getReg() == T8xx::BREG)) && "Unsupported register for MemSrc");
      assert(Op2.isImm() && "Offset is not immediate for MemSrc");

      unsigned Offset = Op2.getImm();
      O << Offset;
    }
  else
    {
      // TODO: Quick fix for disassembler. Not consistent with code when compiling stuff
      O << "x0x";
    }
      
  /*
  O << "[";
  printRegName(O, Op1.getReg());

  unsigned Offset = Op2.getImm();
  if (Offset) {
    O << ", #" << Offset;
  }
  O << "]";
  */
}


