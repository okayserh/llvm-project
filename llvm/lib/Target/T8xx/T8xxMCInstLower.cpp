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

#include "MCTargetDesc/T8xxBaseInfo.h"
#include "MCTargetDesc/T8xxMCAsmInfo.h"
#include "T8xx.h"
#include "T8xxAsmPrinter.h"
#include "llvm/BinaryFormat/ELF.h"
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

#define DEBUG_TYPE "t8xx-mcinst-lower"

static MCOperand lowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym,
				    const AsmPrinter &AP) {
  MCContext &Ctx = AP.OutContext;
  unsigned TargetFlags = MO.getTargetFlags();
  T8xx::Specifier Kind = T8xx::S_None;

  LLVM_DEBUG(dbgs() << "LowerSymbolOperand TargetFlags " << TargetFlags <<
	     "  Type " << (int)MO.getType() << "\n");

  // Note: The T8xxII:MO_NO_FLAG, MO_GLOBAL, etc. (defined in T8xxBaseInfo.h
  // are relevant for the intermediate assembler representation. In this
  // function, the respective operands are transferred into MCSymbolRef
  // operand. When targetflags are present, an MCSpecifierExpr can be
  // created that is then used in T8xxMCCodeEmitter.cpp to create fixups
  // or do other things.

  // Currently, only three MO_... are used in the T8xx backend
  switch (TargetFlags)
    {
    case T8xxII::MO_GLOBAL:
      Kind = ELF::R_T8XX_ADDR;
      break;
    case T8xxII::MO_IPTRREL:
      Kind = ELF::R_T8XX_JUMP;
      break;
    case T8xxII::MO_PCREL_SYM:
      Kind = ELF::R_T8XX_LDPI_SYM;
      break;
    }

  const MCExpr *Expr = MCSymbolRefExpr::create(Sym, Ctx);

  if (Kind != T8xx::S_None)
    Expr = MCSpecifierExpr::create(Expr, Kind, Ctx);
  return MCOperand::createExpr(Expr);
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
  case MachineOperand::MO_RegisterMask:
    // Regmasks are like implicit defs.
    return false;
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = lowerSymbolOperand(MO, MO.getMBB()->getSymbol(), *this);
    break;
  case MachineOperand::MO_GlobalAddress:
    MCOp = lowerSymbolOperand(MO, getSymbolPreferLocal(*MO.getGlobal()), *this);
    break;
  case MachineOperand::MO_BlockAddress:
    MCOp = lowerSymbolOperand(MO, GetBlockAddressSymbol(MO.getBlockAddress()),
                              *this);
    break;
  case MachineOperand::MO_ExternalSymbol:
    MCOp = lowerSymbolOperand(MO, GetExternalSymbolSymbol(MO.getSymbolName()),
                              *this);
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    MCOp = lowerSymbolOperand(MO, GetCPISymbol(MO.getIndex()), *this);
    break;
  case MachineOperand::MO_JumpTableIndex:
    MCOp = lowerSymbolOperand(MO, GetJTISymbol(MO.getIndex()), *this);
    break;
  case MachineOperand::MO_MCSymbol:
    MCOp = lowerSymbolOperand(MO, MO.getMCSymbol(), *this);
    break;
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
