//===-- T8xxAsmPrinter.cpp - T8xx LLVM assembly writer ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format SPARC assembly language.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T8xxInstPrinter.h"
#include "MCTargetDesc/T8xxMCExpr.h"
#include "MCTargetDesc/T8xxTargetStreamer.h"
#include "T8xx.h"
#include "T8xxInstrInfo.h"
#include "T8xxTargetMachine.h"
#include "TargetInfo/T8xxTargetInfo.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfoImpls.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

namespace {
  class T8xxAsmPrinter : public AsmPrinter {
    T8xxTargetStreamer &getTargetStreamer() {
      return static_cast<T8xxTargetStreamer &>(
          *OutStreamer->getTargetStreamer());
    }
  public:
    explicit T8xxAsmPrinter(TargetMachine &TM,
                             std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)) {}

    StringRef getPassName() const override { return "T8xx Assembly Printer"; }

    void printOperand(const MachineInstr *MI, int opNum, raw_ostream &OS);
    void printMemOperand(const MachineInstr *MI, int opNum, raw_ostream &OS,
                         const char *Modifier = nullptr);

    // Taken from LEG machine (TODO)
    void printAddrModeMemSrc(const MachineInstr *MI, int OpNum,
			     raw_ostream &O);

    void emitFunctionBodyStart() override;
    void emitInstruction(const MachineInstr *MI) override;

    static const char *getRegisterName(unsigned RegNo) {
      return T8xxInstPrinter::getRegisterName(RegNo);
    }

    bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                         const char *ExtraCode, raw_ostream &O) override;
    bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                               const char *ExtraCode, raw_ostream &O) override;

    void LowerGETPCXAndEmitMCInsts(const MachineInstr *MI,
                                   const MCSubtargetInfo &STI);

  };
} // end of anonymous namespace

/* TODO: Check reimplementation

static MCOperand createT8xxMCOperand(T8xxMCExpr::VariantKind Kind,
                                      MCSymbol *Sym, MCContext &OutContext) {
  const MCSymbolRefExpr *MCSym = MCSymbolRefExpr::create(Sym,
                                                         OutContext);
  const T8xxMCExpr *expr = T8xxMCExpr::create(Kind, MCSym, OutContext);
  return MCOperand::createExpr(expr);

}
static MCOperand createPCXCallOP(MCSymbol *Label,
                                 MCContext &OutContext) {
  return createT8xxMCOperand(T8xxMCExpr::VK_T8xx_WDISP30, Label, OutContext);
}

static MCOperand createPCXRelExprOp(T8xxMCExpr::VariantKind Kind,
                                    MCSymbol *GOTLabel, MCSymbol *StartLabel,
                                    MCSymbol *CurLabel,
                                    MCContext &OutContext)
{
  const MCSymbolRefExpr *GOT = MCSymbolRefExpr::create(GOTLabel, OutContext);
  const MCSymbolRefExpr *Start = MCSymbolRefExpr::create(StartLabel,
                                                         OutContext);
  const MCSymbolRefExpr *Cur = MCSymbolRefExpr::create(CurLabel,
                                                       OutContext);

  const MCBinaryExpr *Sub = MCBinaryExpr::createSub(Cur, Start, OutContext);
  const MCBinaryExpr *Add = MCBinaryExpr::createAdd(GOT, Sub, OutContext);
  const T8xxMCExpr *expr = T8xxMCExpr::create(Kind,
                                                Add, OutContext);
  return MCOperand::createExpr(expr);
}

static void EmitCall(MCStreamer &OutStreamer,
                     MCOperand &Callee,
                     const MCSubtargetInfo &STI)
{
  MCInst CallInst;
  CallInst.setOpcode(T8xx::CALL);
  CallInst.addOperand(Callee);
  OutStreamer.emitInstruction(CallInst, STI);
}

static void EmitSETHI(MCStreamer &OutStreamer,
                      MCOperand &Imm, MCOperand &RD,
                      const MCSubtargetInfo &STI)
{
  MCInst SETHIInst;
  SETHIInst.setOpcode(T8xx::SETHIi);
  SETHIInst.addOperand(RD);
  SETHIInst.addOperand(Imm);
  OutStreamer.emitInstruction(SETHIInst, STI);
}

static void EmitBinary(MCStreamer &OutStreamer, unsigned Opcode,
                       MCOperand &RS1, MCOperand &Src2, MCOperand &RD,
                       const MCSubtargetInfo &STI)
{
  MCInst Inst;
  Inst.setOpcode(Opcode);
  Inst.addOperand(RD);
  Inst.addOperand(RS1);
  Inst.addOperand(Src2);
  OutStreamer.emitInstruction(Inst, STI);
}

static void EmitOR(MCStreamer &OutStreamer,
                   MCOperand &RS1, MCOperand &Imm, MCOperand &RD,
                   const MCSubtargetInfo &STI) {
  EmitBinary(OutStreamer, T8xx::ORri, RS1, Imm, RD, STI);
}

static void EmitADD(MCStreamer &OutStreamer,
                    MCOperand &RS1, MCOperand &RS2, MCOperand &RD,
                    const MCSubtargetInfo &STI) {
  EmitBinary(OutStreamer, T8xx::ADDrr, RS1, RS2, RD, STI);
}

static void EmitSHL(MCStreamer &OutStreamer,
                    MCOperand &RS1, MCOperand &Imm, MCOperand &RD,
                    const MCSubtargetInfo &STI) {
  EmitBinary(OutStreamer, T8xx::SLLri, RS1, Imm, RD, STI);
}


static void EmitHiLo(MCStreamer &OutStreamer,  MCSymbol *GOTSym,
                     T8xxMCExpr::VariantKind HiKind,
                     T8xxMCExpr::VariantKind LoKind,
                     MCOperand &RD,
                     MCContext &OutContext,
                     const MCSubtargetInfo &STI) {

  MCOperand hi = createT8xxMCOperand(HiKind, GOTSym, OutContext);
  MCOperand lo = createT8xxMCOperand(LoKind, GOTSym, OutContext);
  EmitSETHI(OutStreamer, hi, RD, STI);
  EmitOR(OutStreamer, RD, lo, RD, STI);
}
*/

void T8xxAsmPrinter::LowerGETPCXAndEmitMCInsts(const MachineInstr *MI,
                                                const MCSubtargetInfo &STI)
{
  MCSymbol *GOTLabel   =
    OutContext.getOrCreateSymbol(Twine("_GLOBAL_OFFSET_TABLE_"));

  /*
  const MachineOperand &MO = MI->getOperand(0);
  assert(MO.getReg() != T8xx::O7 &&
         "%o7 is assigned as destination for getpcx!");
  */

  //  MCOperand MCRegOP = MCOperand::createReg(MO.getReg());

  /*
  if (!isPositionIndependent()) {
    // Just load the address of GOT to MCRegOP.
    switch(TM.getCodeModel()) {
    default:
      llvm_unreachable("Unsupported absolute code model");
    case CodeModel::Small:
      EmitHiLo(*OutStreamer, GOTLabel,
               T8xxMCExpr::VK_T8xx_HI, T8xxMCExpr::VK_T8xx_LO,
               MCRegOP, OutContext, STI);
      break;
    case CodeModel::Medium: {
      EmitHiLo(*OutStreamer, GOTLabel,
               T8xxMCExpr::VK_T8xx_H44, T8xxMCExpr::VK_T8xx_M44,
               MCRegOP, OutContext, STI);
      MCOperand imm = MCOperand::createExpr(MCConstantExpr::create(12,
                                                                   OutContext));
      EmitSHL(*OutStreamer, MCRegOP, imm, MCRegOP, STI);
      MCOperand lo = createT8xxMCOperand(T8xxMCExpr::VK_T8xx_L44,
                                          GOTLabel, OutContext);
      EmitOR(*OutStreamer, MCRegOP, lo, MCRegOP, STI);
      break;
    }
    case CodeModel::Large: {
      EmitHiLo(*OutStreamer, GOTLabel,
               T8xxMCExpr::VK_T8xx_HH, T8xxMCExpr::VK_T8xx_HM,
               MCRegOP, OutContext, STI);
      MCOperand imm = MCOperand::createExpr(MCConstantExpr::create(32,
                                                                   OutContext));
      EmitSHL(*OutStreamer, MCRegOP, imm, MCRegOP, STI);
      // Use register %o7 to load the lower 32 bits.
      MCOperand RegO7 = MCOperand::createReg(T8xx::O7);
      EmitHiLo(*OutStreamer, GOTLabel,
               T8xxMCExpr::VK_T8xx_HI, T8xxMCExpr::VK_T8xx_LO,
               RegO7, OutContext, STI);
      EmitADD(*OutStreamer, MCRegOP, RegO7, MCRegOP, STI);
    }
    }
    return;
  }

  MCSymbol *StartLabel = OutContext.createTempSymbol();
  MCSymbol *EndLabel   = OutContext.createTempSymbol();
  MCSymbol *SethiLabel = OutContext.createTempSymbol();

  MCOperand RegO7   = MCOperand::createReg(T8xx::O7);

  // <StartLabel>:
  //   call <EndLabel>
  // <SethiLabel>:
  //     sethi %hi(_GLOBAL_OFFSET_TABLE_+(<SethiLabel>-<StartLabel>)), <MO>
  // <EndLabel>:
  //   or  <MO>, %lo(_GLOBAL_OFFSET_TABLE_+(<EndLabel>-<StartLabel>))), <MO>
  //   add <MO>, %o7, <MO>

  OutStreamer->emitLabel(StartLabel);
  MCOperand Callee =  createPCXCallOP(EndLabel, OutContext);
  EmitCall(*OutStreamer, Callee, STI);
  OutStreamer->emitLabel(SethiLabel);
  MCOperand hiImm = createPCXRelExprOp(T8xxMCExpr::VK_T8xx_PC22,
                                       GOTLabel, StartLabel, SethiLabel,
                                       OutContext);
  EmitSETHI(*OutStreamer, hiImm, MCRegOP, STI);
  OutStreamer->emitLabel(EndLabel);
  MCOperand loImm = createPCXRelExprOp(T8xxMCExpr::VK_T8xx_PC10,
                                       GOTLabel, StartLabel, EndLabel,
                                       OutContext);
  EmitOR(*OutStreamer, MCRegOP, loImm, MCRegOP, STI);
  EmitADD(*OutStreamer, MCRegOP, RegO7, MCRegOP, STI);
  */
}

void T8xxAsmPrinter::emitInstruction(const MachineInstr *MI) {
  T8xx_MC::verifyInstructionPredicates(MI->getOpcode(),
                                        getSubtargetInfo().getFeatureBits());
  /*
  switch (MI->getOpcode()) {
  default: break;
  case TargetOpcode::DBG_VALUE:
    // FIXME: Debug Value.
    return;
  case T8xx::GETPCX:
    LowerGETPCXAndEmitMCInsts(MI, getSubtargetInfo());
    return;
  }
  */
  MachineBasicBlock::const_instr_iterator I = MI->getIterator();
  MachineBasicBlock::const_instr_iterator E = MI->getParent()->instr_end();
  do {
    MCInst TmpInst;
    LowerT8xxMachineInstrToMCInst(&*I, TmpInst, *this);
    EmitToStreamer(*OutStreamer, TmpInst);
  } while ((++I != E) && I->isInsideBundle()); // Delay slot check.
}

void T8xxAsmPrinter::emitFunctionBodyStart() {
    return;
}



void T8xxAsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                   raw_ostream &O) {
  const DataLayout &DL = getDataLayout();
  const MachineOperand &MO = MI->getOperand (opNum);
  T8xxMCExpr::VariantKind TF = (T8xxMCExpr::VariantKind) MO.getTargetFlags();

  bool CloseParen = T8xxMCExpr::printVariantKind(O, TF);

  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    O << "%" << StringRef(getRegisterName(MO.getReg())).lower();
    break;

  case MachineOperand::MO_Immediate:
    O << MO.getImm();
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MO.getMBB()->getSymbol()->print(O, MAI);
    return;
  case MachineOperand::MO_GlobalAddress:
    PrintSymbolOperand(MO, O);
    break;
  case MachineOperand::MO_BlockAddress:
    O <<  GetBlockAddressSymbol(MO.getBlockAddress())->getName();
    break;
  case MachineOperand::MO_ExternalSymbol:
    O << MO.getSymbolName();
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    O << DL.getPrivateGlobalPrefix() << "CPI" << getFunctionNumber() << "_"
      << MO.getIndex();
    break;
  case MachineOperand::MO_Metadata:
    MO.getMetadata()->printAsOperand(O, MMI->getModule());
    break;
  default:
    llvm_unreachable("<unknown operand type>");
  }
  if (CloseParen) O << ")";
}

void T8xxAsmPrinter::printMemOperand(const MachineInstr *MI, int opNum,
                                      raw_ostream &O, const char *Modifier) {
  printOperand(MI, opNum, O);

  // If this is an ADD operand, emit it like normal operands.
  if (Modifier && !strcmp(Modifier, "arith")) {
    O << ", ";
    printOperand(MI, opNum+1, O);
    return;
  }

  if (MI->getOperand(opNum+1).isReg() &&
      MI->getOperand(opNum+1).getReg() == T8xx::R0)
    return;   // don't print "+%g0"
  if (MI->getOperand(opNum+1).isImm() &&
      MI->getOperand(opNum+1).getImm() == 0)
    return;   // don't print "+0"

  O << "+";
  printOperand(MI, opNum+1, O);
}


// Print a 'memsrc' operand which is a (Register, Offset) pair.
void T8xxAsmPrinter::printAddrModeMemSrc(const MachineInstr *MI, int OpNum,
                                         raw_ostream &O) {
  const MachineOperand &Op1 = MI->getOperand(OpNum);
  const MachineOperand &Op2 = MI->getOperand(OpNum + 1);
  O << "[";
  //  printRegName(O, Op1.getReg());
  //StringRef(getRegisterName(MO.getReg())).lower()
  
  unsigned Offset = Op2.getImm();
  if (Offset) {
    O << ", #" << Offset;
  }
  O << "]";
}



/// PrintAsmOperand - Print out an operand for an inline asm expression.
///
bool T8xxAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                      const char *ExtraCode,
                                      raw_ostream &O) {
  if (ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0) return true; // Unknown modifier.

    switch (ExtraCode[0]) {
    default:
      // See if this is a generic print operand
      return AsmPrinter::PrintAsmOperand(MI, OpNo, ExtraCode, O);
    case 'f':
    case 'r':
     break;
    }
  }

  printOperand(MI, OpNo, O);

  return false;
}

bool T8xxAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                            unsigned OpNo,
                                            const char *ExtraCode,
                                            raw_ostream &O) {
  if (ExtraCode && ExtraCode[0])
    return true;  // Unknown modifier

  O << '[';
  printMemOperand(MI, OpNo, O);
  O << ']';

  return false;
}

// Force static initialization.
extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeT8xxAsmPrinter() {
  RegisterAsmPrinter<T8xxAsmPrinter> X(getTheT8xxTarget());
}
