//===-- T8xxMCInstLower.cpp - Convert T8xx MachineInstr to MCInst -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower T8xx MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "T8xx.h"
#include "T8xxAsmPrinter.h"
#include "MCTargetDesc/T8xxMCExpr.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"

#include <iostream>

using namespace llvm;


MCOperand T8xxAsmPrinter::LowerSymbolOperand(const MachineOperand &MO) {

  T8xxMCExpr::VariantKind Kind =
    (T8xxMCExpr::VariantKind)MO.getTargetFlags();
  const MCSymbol *Symbol = nullptr;

  switch(MO.getType()) {
  default: llvm_unreachable("Unknown type in LowerSymbolOperand");
  case MachineOperand::MO_MachineBasicBlock:
    {
      Symbol = MO.getMBB()->getSymbol();

      /* When an assembler ".s" file is generated, real labels are generated as well
	 for ".o" files, no labels are generated.
      printf ("################\n");
      if (Symbol->isTemporary ())
	printf ("Sym is Temp\n");
      std::string temp;
      raw_string_ostream ostemp(temp);
      Symbol->print (ostemp, nullptr);
      std::cout << temp;
    
      printf ("Lower Sym Name %s\n", Symbol->getName().str().c_str());
      */
    }
    break;

  case MachineOperand::MO_GlobalAddress:
    Symbol = getSymbol(MO.getGlobal());
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = GetBlockAddressSymbol(MO.getBlockAddress());
    break;

  case MachineOperand::MO_JumpTableIndex:
    Symbol = GetJTISymbol(MO.getIndex());
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = GetExternalSymbolSymbol(MO.getSymbolName());
    break;

  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = GetCPISymbol(MO.getIndex());
    break;
  }

  // Attempt to create proper symbols
  MCSymbolRefExpr::VariantKind Kind2 = MCSymbolRefExpr::VK_None;
  const MCExpr *Expr = MCSymbolRefExpr::create(Symbol, Kind2, OutContext);
  return MCOperand::createExpr(Expr);

  /*
  const MCSymbolRefExpr *MCSym = MCSymbolRefExpr::create(Symbol,
                                                         OutContext);

  const T8xxMCExpr *expr = T8xxMCExpr::create(Kind, MCSym,
                                                OutContext);

  return MCOperand::createExpr(expr);
  */
}

bool T8xxAsmPrinter::lowerOperand(const MachineOperand &MO,
				  MCOperand &MCOp) {
  switch(MO.getType()) {
  default: llvm_unreachable("unknown operand type"); break;
  case MachineOperand::MO_Register:
    //  Ignore all implicit register operands
    if (MO.isImplicit())
      return false;
    MCOp = MCOperand::createReg(MO.getReg());
    break;
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_BlockAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_ConstantPoolIndex:
    MCOp = LowerSymbolOperand(MO);
    break;
  case MachineOperand::MO_JumpTableIndex:
    {
      // Copied from ARMAsmPrinter::GetSymbolRef
      MCSymbol *Symbol = GetJTISymbol(MO.getIndex());

      // TODO: Clarify whether to use the T8xxMCExpr variant
      // or the MCSymbolRefExpr variant (latter being used for ARM backend)
      /*
      MCSymbolRefExpr::VariantKind SymbolVariant = MCSymbolRefExpr::VK_T8xx_GLOBAL;
      const MCExpr *Expr =
	MCSymbolRefExpr::create(Symbol, SymbolVariant, OutContext);
      */

      T8xxMCExpr::VariantKind SymbolVariant = T8xxMCExpr::VK_T8xx_GLOBAL;
      const MCSymbolRefExpr *MCSym = MCSymbolRefExpr::create(Symbol,
                                                         OutContext);
      const T8xxMCExpr *Expr = T8xxMCExpr::create(SymbolVariant, MCSym,
						  OutContext);
      
      MCOp = MCOperand::createExpr(Expr);
    }
    break;

  case MachineOperand::MO_RegisterMask:
    return false;
  }
  return true;
}

void llvm::LowerT8xxMachineInstrToMCInst(const MachineInstr *MI,
                                          MCInst &OutMI,
                                          T8xxAsmPrinter &AP)
{
  OutMI.setOpcode(MI->getOpcode());

  for (const MachineOperand &MO : MI->operands()) {
    MCOperand MCOp;
    if (AP.lowerOperand(MO, MCOp)) {
      OutMI.addOperand(MCOp);
    }
  }
}
