//===-- T8xxAsmParser.cpp - Parse T8xx assembly to MCInst instructions --===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T8xxMCExpr.h"
#include "MCTargetDesc/T8xxMCTargetDesc.h"
#include "TargetInfo/T8xxTargetInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/TargetParser/Triple.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCAsmParser.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SMLoc.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <memory>

using namespace llvm;

namespace {

class T8xxOperand;

class T8xxAsmParser : public MCTargetAsmParser {
  MCAsmParser &Parser;

  /// @name Auto-generated Match Functions
  /// {

#define GET_ASSEMBLER_HEADER
#include "T8xxGenAsmMatcher.inc"

  /// }

  // public interface of the MCTargetAsmParser.
  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;
  bool parseRegister(MCRegister &Reg, SMLoc &StartLoc, SMLoc &EndLoc) override;
  OperandMatchResultTy tryParseRegister(MCRegister &RegNo, SMLoc &StartLoc,
                                        SMLoc &EndLoc) override;
  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;
  bool ParseDirective(AsmToken DirectiveID) override;

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  // Custom parse functions for T8xx specific operands.
  OperandMatchResultTy parseWPtrOperand(OperandVector &Operands);

  OperandMatchResultTy parseCallTarget(OperandVector &Operands);

  OperandMatchResultTy parseOperand(OperandVector &Operands, StringRef Name);

  OperandMatchResultTy
  parseT8xxAsmOperand(std::unique_ptr<T8xxOperand> &Operand,
                       bool isCall = false);

  OperandMatchResultTy parseBranchModifiers(OperandVector &Operands);

  // Helper function for dealing with %lo / %hi in PIC mode.
  const T8xxMCExpr *adjustPICRelocation(T8xxMCExpr::VariantKind VK,
                                         const MCExpr *subExpr);

  // returns true if Tok is matched to a register and returns register in Reg.
  bool matchRegisterName(const AsmToken &Tok, MCRegister &Reg,
                         unsigned &RegKind);

  //  bool matchT8xxAsmModifiers(const MCExpr *&EVal, SMLoc &EndLoc);
  /*
  bool is64Bit() const {
    return getSTI().getTargetTriple().getArch() == Triple::sparcv9;
  }
  */

  SMLoc getLoc() const { return getParser().getTok().getLoc(); }

public:
  T8xxAsmParser(const MCSubtargetInfo &sti, MCAsmParser &parser,
                const MCInstrInfo &MII,
                const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, sti, MII), Parser(parser) {
    Parser.addAliasForDirective(".half", ".2byte");
    Parser.addAliasForDirective(".uahalf", ".2byte");
    Parser.addAliasForDirective(".word", ".4byte");
    Parser.addAliasForDirective(".uaword", ".4byte");
    //    Parser.addAliasForDirective(".nword", is64Bit() ? ".8byte" : ".4byte");

    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(getSTI().getFeatureBits()));
  }
};

} // end anonymous namespace


namespace {

/// T8xxOperand - Instances of this class represent a parsed T8xx machine
/// instruction.
class T8xxOperand : public MCParsedAsmOperand {
public:
  enum RegisterKind {
    rk_None,
    rk_Int,
    rk_Special,
  };

private:
  enum KindTy {
    k_Token,
    k_Register,
    k_Immediate,
    k_MemoryImm
  } Kind;

  SMLoc StartLoc, EndLoc;

  struct Token {
    const char *Data;
    unsigned Length;
  };

  struct RegOp {
    unsigned RegNum;
    RegisterKind Kind;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  struct MemOp {
    unsigned Base;
    unsigned OffsetReg;
    const MCExpr *Off;
  };

  union {
    struct Token Tok;
    struct RegOp Reg;
    struct ImmOp Imm;
    struct MemOp Mem;
  };

public:
  T8xxOperand(KindTy K) : Kind(K) {}

  bool isToken() const override { return Kind == k_Token; }
  bool isReg() const override { return Kind == k_Register; }
  bool isImm() const override { return Kind == k_Immediate; }
  bool isMem() const override { return isWPtrSrc(); }
  bool isWPtrSrc() const { return Kind == k_MemoryImm; }

  bool isCallTarget() const {
    if (!isImm())
      return false;

    if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Imm.Val))
      return CE->getValue() % 4 == 0;

    return true;
  }

  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return StringRef(Tok.Data, Tok.Length);
  }

  unsigned getReg() const override {
    assert((Kind == k_Register) && "Invalid access!");
    return Reg.RegNum;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate) && "Invalid access!");
    return Imm.Val;
  }

  // TODO: Check what this is needed for
  unsigned getMemBase() const {
    assert((Kind == k_MemoryImm) && "Invalid access!");
    return Mem.Base;
  }

  const MCExpr *getMemOff() const {
    assert((Kind == k_MemoryImm) && "Invalid access!");
    return Mem.Off;
  }


  
  /// getStartLoc - Get the location of the first token of this operand.
  SMLoc getStartLoc() const override {
    return StartLoc;
  }
  /// getEndLoc - Get the location of the last token of this operand.
  SMLoc getEndLoc() const override {
    return EndLoc;
  }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case k_Token:     OS << "Token: " << getToken() << "\n"; break;
    case k_Immediate: OS << "Imm: " << getImm() << "\n"; break;
    }
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addWPtrSrcOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getMemBase()));

    const MCExpr *Expr = getMemOff();
    addExpr(Inst, Expr);
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const{
    // Add as immediate when possible.  Null MCExpr = 0.
    if (!Expr)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addCallTargetOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

  static std::unique_ptr<T8xxOperand> CreateToken(StringRef Str, SMLoc S) {
    auto Op = std::make_unique<T8xxOperand>(k_Token);
    Op->Tok.Data = Str.data();
    Op->Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  static std::unique_ptr<T8xxOperand> CreateReg(unsigned RegNum, unsigned Kind,
                                                 SMLoc S, SMLoc E) {
    auto Op = std::make_unique<T8xxOperand>(k_Register);
    Op->Reg.RegNum = RegNum;
    Op->Reg.Kind   = (T8xxOperand::RegisterKind)Kind;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<T8xxOperand> CreateImm(const MCExpr *Val, SMLoc S,
                                                 SMLoc E) {
    auto Op = std::make_unique<T8xxOperand>(k_Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

};

} // end anonymous namespace


bool T8xxAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                             OperandVector &Operands,
                                             MCStreamer &Out,
                                             uint64_t &ErrorInfo,
                                             bool MatchingInlineAsm) {
  MCInst Inst;
  SmallVector<MCInst, 8> Instructions;
  unsigned MatchResult = MatchInstructionImpl(Operands, Inst, ErrorInfo,
                                              MatchingInlineAsm);

  // Debug ouput
  printf ("MatchAndEmit\n");
  for (auto T = Operands.begin (); T != Operands.end (); ++T)
    (*T)->dump ();

  switch (MatchResult) {
  case Match_Success: {
    switch (Inst.getOpcode()) {
    default:
      Inst.setLoc(IDLoc);
      Instructions.push_back(Inst);
      break;
    }

    for (const MCInst &I : Instructions) {
      Out.emitInstruction(I, getSTI());
    }
    return false;
  }

  case Match_MissingFeature:
    return Error(IDLoc,
                 "instruction requires a CPU feature not currently enabled");

  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0ULL) {
      if (ErrorInfo >= Operands.size())
        return Error(IDLoc, "too few operands for instruction");

      ErrorLoc = ((T8xxOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }

    return Error(ErrorLoc, "invalid operand for instruction");
  }
  case Match_MnemonicFail:
    return Error(IDLoc, "invalid instruction mnemonic");
  }
  llvm_unreachable("Implement any new match types added!");
}


bool T8xxAsmParser::parseRegister(MCRegister &RegNo, SMLoc &StartLoc,
                                   SMLoc &EndLoc) {
  if (tryParseRegister(RegNo, StartLoc, EndLoc) != MatchOperand_Success)
    return Error(StartLoc, "invalid register name");
  return false;
}

OperandMatchResultTy T8xxAsmParser::tryParseRegister(MCRegister &RegNo,
                                                      SMLoc &StartLoc,
                                                      SMLoc &EndLoc) {
  const AsmToken &Tok = Parser.getTok();
  StartLoc = Tok.getLoc();
  EndLoc = Tok.getEndLoc();
  RegNo = 0;
  if (getLexer().getKind() != AsmToken::Percent)
    return MatchOperand_NoMatch;
  Parser.Lex();
  unsigned regKind = T8xxOperand::rk_None;
  /*
  if (matchRegisterName(Tok, RegNo, regKind)) {
    Parser.Lex();
    return MatchOperand_Success;
  }
  */

  getLexer().UnLex(Tok);
  return MatchOperand_NoMatch;
}

static void applyMnemonicAliases(StringRef &Mnemonic,
                                 const FeatureBitset &Features,
                                 unsigned VariantID);

bool T8xxAsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                      StringRef Name, SMLoc NameLoc,
                                      OperandVector &Operands) {

  // First operand in MCInst is instruction mnemonic.
  Operands.push_back(T8xxOperand::CreateToken(Name, NameLoc));

  // apply mnemonic aliases, if any, so that we can parse operands correctly.
  //TODO OKH  applyMnemonicAliases(Name, getAvailableFeatures(), 0);

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    /*
    if (getLexer().is(AsmToken::Comma)) {
      if (parseBranchModifiers(Operands) != MatchOperand_Success) {
        SMLoc Loc = getLexer().getLoc();
        return Error(Loc, "unexpected token");
      }
    }
    */

    if (parseOperand(Operands, Name) != MatchOperand_Success) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token");
    }

    while (getLexer().is(AsmToken::Comma) || getLexer().is(AsmToken::Plus)) {
      if (getLexer().is(AsmToken::Plus)) {
      // Plus tokens are significant in software_traps (p83, sparcv8.pdf). We must capture them.
        Operands.push_back(T8xxOperand::CreateToken("+", Parser.getTok().getLoc()));
      }
      Parser.Lex(); // Eat the comma or plus.
      // Parse and remember the operand.
      if (parseOperand(Operands, Name) != MatchOperand_Success) {
        SMLoc Loc = getLexer().getLoc();
        return Error(Loc, "unexpected token");
      }
    }
  }
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    return Error(Loc, "unexpected token");
  }
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

bool T8xxAsmParser::
ParseDirective(AsmToken DirectiveID)
{
  StringRef IDVal = DirectiveID.getString();

  if (IDVal == ".register") {
    // For now, ignore .register directive.
    Parser.eatToEndOfStatement();
    return false;
  }
  if (IDVal == ".proc") {
    // For compatibility, ignore this directive.
    // (It's supposed to be an "optimization" in the Sun assembler)
    Parser.eatToEndOfStatement();
    return false;
  }

  // Let the MC layer to handle other directives.
  return true;
}


OperandMatchResultTy T8xxAsmParser::parseWPtrOperand(OperandVector &Operands) {
  // TODO: Implement
  return MatchOperand_NoMatch;
}
  

OperandMatchResultTy T8xxAsmParser::parseCallTarget(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Integer:
  case AsmToken::Identifier:
  case AsmToken::Dot:
    break;
  }

  const MCExpr *DestValue;
  if (getParser().parseExpression(DestValue))
    return MatchOperand_NoMatch;

  // TODO: Check for position independence.
  bool IsPic = getContext().getObjectFileInfo()->isPositionIndependent();
  /*
  T8xxMCExpr::VariantKind Kind =
      IsPic ? T8xxMCExpr::VK_T8xx_WPLT30 : T8xxMCExpr::VK_T8xx_WDISP30;
  */
  T8xxMCExpr::VariantKind Kind =T8xxMCExpr::VK_T8xx_IPTRREL;
  

  const MCExpr *DestExpr = T8xxMCExpr::create(Kind, DestValue, getContext());
  Operands.push_back(T8xxOperand::CreateImm(DestExpr, S, E));
  return MatchOperand_Success;
}

OperandMatchResultTy
T8xxAsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {

  OperandMatchResultTy ResTy = MatchOperandParserImpl(Operands, Mnemonic);

  // If there wasn't a custom match, try the generic matcher below. Otherwise,
  // there was a match, but an error occurred, in which case, just return that
  // the operand parsing failed.
  if (ResTy == MatchOperand_Success || ResTy == MatchOperand_ParseFail)
    return ResTy;

  // Note: AsmTokens are defined in include/llvm/MC/MCAsmMacro.h
  
  std::unique_ptr<T8xxOperand> Op;

  ResTy = parseT8xxAsmOperand(Op, (Mnemonic == "call"));
  if (ResTy != MatchOperand_Success || !Op)
    return MatchOperand_ParseFail;

  // Push the parsed operand into the list of operands
  Operands.push_back(std::move(Op));

  return MatchOperand_Success;
}

OperandMatchResultTy
T8xxAsmParser::parseT8xxAsmOperand(std::unique_ptr<T8xxOperand> &Op,
                                     bool isCall) {
  SMLoc S = Parser.getTok().getLoc();
  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  const MCExpr *EVal;

  Op = nullptr;
  switch (getLexer().getKind()) {
  default:  break;

  case AsmToken::Percent:
    {
      Parser.Lex(); // Eat the '%'.
      MCRegister Reg;
      unsigned RegKind;
      if (matchRegisterName(Parser.getTok(), Reg, RegKind)) {
	StringRef name = Parser.getTok().getString();
	Parser.Lex(); // Eat the identifier token.
	E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
	Op = T8xxOperand::CreateReg(Reg, RegKind, S, E);
      }
      /*
      if (matchT8xxAsmModifiers(EVal, E)) {
	E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
	Op = T8xxOperand::CreateImm(EVal, S, E);
      }
      */
    }
    break;

  case AsmToken::Plus:
  case AsmToken::Minus:
  case AsmToken::Integer:
  case AsmToken::LParen:
  case AsmToken::Dot:
  case AsmToken::Identifier:
    if (getParser().parseExpression(EVal, E))
      break;

    int64_t Res;
    if (!EVal->evaluateAsAbsolute(Res)) {
      T8xxMCExpr::VariantKind Kind = T8xxMCExpr::VK_T8xx_IPTRREL;

      if (getContext().getObjectFileInfo()->isPositionIndependent()) {
        if (isCall)
          Kind = T8xxMCExpr::VK_T8xx_IPTRREL;
        else
          Kind = T8xxMCExpr::VK_T8xx_IPTRREL;
      }
      EVal = T8xxMCExpr::create(Kind, EVal, getContext());
    }
    Op = T8xxOperand::CreateImm(EVal, S, E);
    break;
  }
  return (Op) ? MatchOperand_Success : MatchOperand_ParseFail;
}


bool T8xxAsmParser::matchRegisterName(const AsmToken &Tok, MCRegister &RegNo,
                                       unsigned &RegKind) {
  int64_t intVal = 0;
  RegNo = 0;
  RegKind = T8xxOperand::rk_None;
  if (Tok.is(AsmToken::Identifier)) {
    StringRef name = Tok.getString();

    // AREG
    if (name.equals("AREG")) {
      RegNo = T8xx::AREG;
      RegKind = T8xxOperand::rk_Int;
      return true;
    }
    // AREG
    if (name.equals("BREG")) {
      RegNo = T8xx::BREG;
      RegKind = T8xxOperand::rk_Int;
      return true;
    }
    // CREG
    if (name.equals("CREG")) {
      RegNo = T8xx::CREG;
      RegKind = T8xxOperand::rk_Int;
      return true;
    }
    // WPTR
    if (name.equals("WPTR")) {
      RegNo = T8xx::WPTR;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
  }
    
   return false;
}




extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeT8xxAsmParser() {
  RegisterMCAsmParser<T8xxAsmParser> A(getTheT8xxTarget());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "T8xxGenAsmMatcher.inc"

unsigned T8xxAsmParser::validateTargetOperandClass(MCParsedAsmOperand &GOp,
                                                    unsigned Kind) {
  T8xxOperand &Op = (T8xxOperand &)GOp;
  /*
  if (Op.isFloatOrDoubleReg()) {
    switch (Kind) {
    default: break;
    case MCK_DFPRegs:
      if (!Op.isFloatReg() || T8xxOperand::MorphToDoubleReg(Op))
        return MCTargetAsmParser::Match_Success;
      break;
    case MCK_QFPRegs:
      if (T8xxOperand::MorphToQuadReg(Op))
        return MCTargetAsmParser::Match_Success;
      break;
    }
  }
  if (Op.isIntReg() && Kind == MCK_IntPair) {
    if (T8xxOperand::MorphToIntPairReg(Op))
      return MCTargetAsmParser::Match_Success;
  }
  if (Op.isCoprocReg() && Kind == MCK_CoprocPair) {
     if (T8xxOperand::MorphToCoprocPairReg(Op))
       return MCTargetAsmParser::Match_Success;
   }
  */
  return Match_InvalidOperand;
}
