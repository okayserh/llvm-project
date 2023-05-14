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

  enum class TailRelocKind { Load_GOT, Add_TLS, Load_TLS, Call_TLS };

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
  OperandMatchResultTy tryParseRegister(MCRegister &Reg, SMLoc &StartLoc,
                                        SMLoc &EndLoc) override;
  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;
  bool ParseDirective(AsmToken DirectiveID) override;

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  // Custom parse functions for T8xx specific operands.
  OperandMatchResultTy parseMEMOperand(OperandVector &Operands);

  OperandMatchResultTy parseMembarTag(OperandVector &Operands);

  template <TailRelocKind Kind>
  OperandMatchResultTy parseTailRelocSym(OperandVector &Operands);

  template <unsigned N>
  OperandMatchResultTy parseShiftAmtImm(OperandVector &Operands);

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

  bool matchT8xxAsmModifiers(const MCExpr *&EVal, SMLoc &EndLoc);

  bool is64Bit() const {
    return getSTI().getTargetTriple().getArch() == Triple::sparcv9;
  }

  bool expandSET(MCInst &Inst, SMLoc IDLoc,
                 SmallVectorImpl<MCInst> &Instructions);

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
    Parser.addAliasForDirective(".nword", is64Bit() ? ".8byte" : ".4byte");

    // Initialize the set of available features.
    setAvailableFeatures(ComputeAvailableFeatures(getSTI().getFeatureBits()));
  }
};

} // end anonymous namespace

  static const MCPhysReg IntRegs[32] = {
    T8xx::R0, T8xx::R1, T8xx::R2, T8xx::R3,
    T8xx::R4, T8xx::R5, T8xx::R6, T8xx::R7,
    T8xx::R8, T8xx::R9, T8xx::R10, T8xx::R11,
    T8xx::R12, T8xx::R13, T8xx::R14, T8xx::R15 };


namespace {

/// T8xxOperand - Instances of this class represent a parsed T8xx machine
/// instruction.
class T8xxOperand : public MCParsedAsmOperand {
public:
  enum RegisterKind {
    rk_None,
    rk_IntReg,
    rk_IntPairReg,
    rk_FloatReg,
    rk_DoubleReg,
    rk_QuadReg,
    rk_CoprocReg,
    rk_CoprocPairReg,
    rk_Special,
  };

private:
  enum KindTy {
    k_Token,
    k_Register,
    k_Immediate,
    k_MemoryReg,
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
  bool isMem() const override { return isMEMrr() || isMEMri(); }
  bool isMEMrr() const { return Kind == k_MemoryReg; }
  bool isMEMri() const { return Kind == k_MemoryImm; }
  bool isMembarTag() const { return Kind == k_Immediate; }
  bool isTailRelocSym() const { return Kind == k_Immediate; }

  bool isCallTarget() const {
    if (!isImm())
      return false;

    if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Imm.Val))
      return CE->getValue() % 4 == 0;

    return true;
  }

  bool isShiftAmtImm5() const {
    if (!isImm())
      return false;

    if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Imm.Val))
      return isUInt<5>(CE->getValue());

    return false;
  }

  bool isShiftAmtImm6() const {
    if (!isImm())
      return false;

    if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Imm.Val))
      return isUInt<6>(CE->getValue());

    return false;
  }

  bool isIntReg() const {
    return (Kind == k_Register && Reg.Kind == rk_IntReg);
  }

  bool isFloatReg() const {
    return (Kind == k_Register && Reg.Kind == rk_FloatReg);
  }

  bool isFloatOrDoubleReg() const {
    return (Kind == k_Register && (Reg.Kind == rk_FloatReg
                                   || Reg.Kind == rk_DoubleReg));
  }

  bool isCoprocReg() const {
    return (Kind == k_Register && Reg.Kind == rk_CoprocReg);
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

  unsigned getMemBase() const {
    assert((Kind == k_MemoryReg || Kind == k_MemoryImm) && "Invalid access!");
    return Mem.Base;
  }

  unsigned getMemOffsetReg() const {
    assert((Kind == k_MemoryReg) && "Invalid access!");
    return Mem.OffsetReg;
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
    case k_Register:  OS << "Reg: #" << getReg() << "\n"; break;
    case k_Immediate: OS << "Imm: " << getImm() << "\n"; break;
    case k_MemoryReg: OS << "Mem: " << getMemBase() << "+"
                         << getMemOffsetReg() << "\n"; break;
    case k_MemoryImm: assert(getMemOff() != nullptr);
      OS << "Mem: " << getMemBase()
         << "+" << *getMemOff()
         << "\n"; break;
    }
  }

  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addShiftAmtImm5Operands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }
  void addShiftAmtImm6Operands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
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

  void addMEMrrOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getMemBase()));

    assert(getMemOffsetReg() != 0 && "Invalid offset");
    Inst.addOperand(MCOperand::createReg(getMemOffsetReg()));
  }

  void addMEMriOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getMemBase()));

    const MCExpr *Expr = getMemOff();
    addExpr(Inst, Expr);
  }

  void addMembarTagOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addCallTargetOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

  void addTailRelocSymOperands(MCInst &Inst, unsigned N) const {
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


  static std::unique_ptr<T8xxOperand>
  MorphToMEMrr(unsigned Base, std::unique_ptr<T8xxOperand> Op) {
    unsigned offsetReg = Op->getReg();
    Op->Kind = k_MemoryReg;
    Op->Mem.Base = Base;
    Op->Mem.OffsetReg = offsetReg;
    Op->Mem.Off = nullptr;
    return Op;
  }

  static std::unique_ptr<T8xxOperand>
  CreateMEMr(unsigned Base, SMLoc S, SMLoc E) {
    auto Op = std::make_unique<T8xxOperand>(k_MemoryReg);
    Op->Mem.Base = Base;
    Op->Mem.OffsetReg = T8xx::R0;  // always 0
    Op->Mem.Off = nullptr;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static std::unique_ptr<T8xxOperand>
  MorphToMEMri(unsigned Base, std::unique_ptr<T8xxOperand> Op) {
    const MCExpr *Imm  = Op->getImm();
    Op->Kind = k_MemoryImm;
    Op->Mem.Base = Base;
    Op->Mem.OffsetReg = 0;
    Op->Mem.Off = Imm;
    return Op;
  }
};

} // end anonymous namespace

bool T8xxAsmParser::expandSET(MCInst &Inst, SMLoc IDLoc,
                               SmallVectorImpl<MCInst> &Instructions) {
  MCOperand MCRegOp = Inst.getOperand(0);
  MCOperand MCValOp = Inst.getOperand(1);
  assert(MCRegOp.isReg());
  assert(MCValOp.isImm() || MCValOp.isExpr());

  // the imm operand can be either an expression or an immediate.
  bool IsImm = Inst.getOperand(1).isImm();
  int64_t RawImmValue = IsImm ? MCValOp.getImm() : 0;

  // Allow either a signed or unsigned 32-bit immediate.
  if (RawImmValue < -2147483648LL || RawImmValue > 4294967295LL) {
    return Error(IDLoc,
                 "set: argument must be between -2147483648 and 4294967295");
  }

  // If the value was expressed as a large unsigned number, that's ok.
  // We want to see if it "looks like" a small signed number.
  int32_t ImmValue = RawImmValue;
  // For 'set' you can't use 'or' with a negative operand on V9 because
  // that would splat the sign bit across the upper half of the destination
  // register, whereas 'set' is defined to zero the high 32 bits.
  bool IsEffectivelyImm13 =
    IsImm && (-4096 <= ImmValue && ImmValue < 4096);
  const MCExpr *ValExpr;
  if (IsImm)
    ValExpr = MCConstantExpr::create(ImmValue, getContext());
  else
    ValExpr = MCValOp.getExpr();

  MCOperand PrevReg = MCOperand::createReg(T8xx::R0);

  // The low bits require touching in 3 cases:
  // * A non-immediate value will always require both instructions.
  // * An effectively imm13 value needs only an 'or' instruction.
  // * Otherwise, an immediate that is not effectively imm13 requires the
  //   'or' only if bits remain after clearing the 22 bits that 'sethi' set.
  // If the low bits are known zeros, there's nothing to do.
  // In the second case, and only in that case, must we NOT clear
  // bits of the immediate value via the %lo() assembler function.
  // Note also, the 'or' instruction doesn't mind a large value in the case
  // where the operand to 'set' was 0xFFFFFzzz - it does exactly what you mean.
  /*
  if (!IsImm || IsEffectivelyImm13 || (ImmValue & 0x3ff)) {
    MCInst TmpInst;
    const MCExpr *Expr;
    if (IsEffectivelyImm13)
      Expr = ValExpr;
    else
      Expr = adjustPICRelocation(T8xxMCExpr::VK_T8xx_LO, ValExpr);
    TmpInst.setLoc(IDLoc);
    TmpInst.setOpcode(T8::ORri);
    TmpInst.addOperand(MCRegOp);
    TmpInst.addOperand(PrevReg);
    TmpInst.addOperand(MCOperand::createExpr(Expr));
    Instructions.push_back(TmpInst);
  }
  */
  return false;
}

bool T8xxAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                             OperandVector &Operands,
                                             MCStreamer &Out,
                                             uint64_t &ErrorInfo,
                                             bool MatchingInlineAsm) {
  MCInst Inst;
  SmallVector<MCInst, 8> Instructions;
  unsigned MatchResult = MatchInstructionImpl(Operands, Inst, ErrorInfo,
                                              MatchingInlineAsm);
  switch (MatchResult) {
  case Match_Success: {
    switch (Inst.getOpcode()) {
    default:
      Inst.setLoc(IDLoc);
      Instructions.push_back(Inst);
      break;
      /*
    case T8::SET:
      if (expandSET(Inst, IDLoc, Instructions))
        return true;
      break;
      */
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

bool T8xxAsmParser::parseRegister(MCRegister &Reg, SMLoc &StartLoc,
                                   SMLoc &EndLoc) {
  if (tryParseRegister(Reg, StartLoc, EndLoc) != MatchOperand_Success)
    return Error(StartLoc, "invalid register name");
  return false;
}

OperandMatchResultTy T8xxAsmParser::tryParseRegister(MCRegister &Reg,
                                                      SMLoc &StartLoc,
                                                      SMLoc &EndLoc) {
  const AsmToken &Tok = Parser.getTok();
  StartLoc = Tok.getLoc();
  EndLoc = Tok.getEndLoc();
  Reg = 0;
  if (getLexer().getKind() != AsmToken::Percent)
    return MatchOperand_NoMatch;
  Parser.Lex();
  unsigned regKind = T8xxOperand::rk_None;
  if (matchRegisterName(Tok, Reg, regKind)) {
    Parser.Lex();
    return MatchOperand_Success;
  }

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
    if (getLexer().is(AsmToken::Comma)) {
      if (parseBranchModifiers(Operands) != MatchOperand_Success) {
        SMLoc Loc = getLexer().getLoc();
        return Error(Loc, "unexpected token");
      }
    }
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

OperandMatchResultTy
T8xxAsmParser::parseMEMOperand(OperandVector &Operands) {
  SMLoc S, E;

  std::unique_ptr<T8xxOperand> LHS;
  if (parseT8xxAsmOperand(LHS) != MatchOperand_Success)
    return MatchOperand_NoMatch;

  // Single immediate operand
  if (LHS->isImm()) {
    Operands.push_back(T8xxOperand::MorphToMEMri(T8xx::R0, std::move(LHS)));
    return MatchOperand_Success;
  }

  if (!LHS->isIntReg()) {
    Error(LHS->getStartLoc(), "invalid register kind for this operand");
    return MatchOperand_ParseFail;
  }

  AsmToken Tok = getLexer().getTok();
  // The plus token may be followed by a register or an immediate value, the
  // minus one is always interpreted as sign for the immediate value
  if (Tok.is(AsmToken::Plus) || Tok.is(AsmToken::Minus)) {
    (void)Parser.parseOptionalToken(AsmToken::Plus);

    std::unique_ptr<T8xxOperand> RHS;
    if (parseT8xxAsmOperand(RHS) != MatchOperand_Success)
      return MatchOperand_NoMatch;

    if (RHS->isReg() && !RHS->isIntReg()) {
      Error(RHS->getStartLoc(), "invalid register kind for this operand");
      return MatchOperand_ParseFail;
    }

    Operands.push_back(
        RHS->isImm()
            ? T8xxOperand::MorphToMEMri(LHS->getReg(), std::move(RHS))
            : T8xxOperand::MorphToMEMrr(LHS->getReg(), std::move(RHS)));

    return MatchOperand_Success;
  }

  Operands.push_back(T8xxOperand::CreateMEMr(LHS->getReg(), S, E));
  return MatchOperand_Success;
}

template <unsigned N>
OperandMatchResultTy T8xxAsmParser::parseShiftAmtImm(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);

  // This is a register, not an immediate
  if (getLexer().getKind() == AsmToken::Percent)
    return MatchOperand_NoMatch;

  const MCExpr *Expr;
  if (getParser().parseExpression(Expr))
    return MatchOperand_ParseFail;

  const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr);
  if (!CE) {
    Error(S, "constant expression expected");
    return MatchOperand_ParseFail;
  }

  if (!isUInt<N>(CE->getValue())) {
    Error(S, "immediate shift value out of range");
    return MatchOperand_ParseFail;
  }

  Operands.push_back(T8xxOperand::CreateImm(Expr, S, E));
  return MatchOperand_Success;
}

template <T8xxAsmParser::TailRelocKind Kind>
OperandMatchResultTy
T8xxAsmParser::parseTailRelocSym(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);

  auto MatchesKind = [](T8xxMCExpr::VariantKind VK) -> bool {
    switch (Kind) {
    case TailRelocKind::Load_GOT:
      // Non-TLS relocations on ld (or ldx).
      // ld [%rr + %rr], %rr, %rel(sym)
      return VK == T8xxMCExpr::VK_T8xx_GOTDATA_OP;
    case TailRelocKind::Add_TLS:
      // TLS relocations on add.
      // add %rr, %rr, %rr, %rel(sym)
      switch (VK) {
      case T8xxMCExpr::VK_T8xx_TLS_GD_ADD:
      case T8xxMCExpr::VK_T8xx_TLS_IE_ADD:
      case T8xxMCExpr::VK_T8xx_TLS_LDM_ADD:
      case T8xxMCExpr::VK_T8xx_TLS_LDO_ADD:
        return true;
      default:
        return false;
      }
    case TailRelocKind::Load_TLS:
      // TLS relocations on ld (or ldx).
      // ld[x] %addr, %rr, %rel(sym)
      switch (VK) {
      case T8xxMCExpr::VK_T8xx_TLS_IE_LD:
      case T8xxMCExpr::VK_T8xx_TLS_IE_LDX:
        return true;
      default:
        return false;
      }
    case TailRelocKind::Call_TLS:
      // TLS relocations on call.
      // call sym, %rel(sym)
      switch (VK) {
      case T8xxMCExpr::VK_T8xx_TLS_GD_CALL:
      case T8xxMCExpr::VK_T8xx_TLS_LDM_CALL:
        return true;
      default:
        return false;
      }
    }
    llvm_unreachable("Unhandled T8xxAsmParser::TailRelocKind enum");
  };

  if (getLexer().getKind() != AsmToken::Percent) {
    Error(getLoc(), "expected '%' for operand modifier");
    return MatchOperand_ParseFail;
  }

  const AsmToken Tok = Parser.getTok();
  getParser().Lex(); // Eat '%'

  if (getLexer().getKind() != AsmToken::Identifier) {
    Error(getLoc(), "expected valid identifier for operand modifier");
    return MatchOperand_ParseFail;
  }

  StringRef Name = getParser().getTok().getIdentifier();
  T8xxMCExpr::VariantKind VK = T8xxMCExpr::parseVariantKind(Name);
  if (VK == T8xxMCExpr::VK_T8xx_None) {
    Error(getLoc(), "invalid operand modifier");
    return MatchOperand_ParseFail;
  }

  if (!MatchesKind(VK)) {
    // Did not match the specified set of relocation types, put '%' back.
    getLexer().UnLex(Tok);
    return MatchOperand_NoMatch;
  }

  Parser.Lex(); // Eat the identifier.
  if (getLexer().getKind() != AsmToken::LParen) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '('
  const MCExpr *SubExpr;
  if (getParser().parseParenExpression(SubExpr, E)) {
    return MatchOperand_ParseFail;
  }

  const MCExpr *Val = adjustPICRelocation(VK, SubExpr);
  Operands.push_back(T8xxOperand::CreateImm(Val, S, E));
  return MatchOperand_Success;
}

OperandMatchResultTy T8xxAsmParser::parseMembarTag(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();
  const MCExpr *EVal;
  int64_t ImmVal = 0;

  std::unique_ptr<T8xxOperand> Mask;
  if (parseT8xxAsmOperand(Mask) == MatchOperand_Success) {
    if (!Mask->isImm() || !Mask->getImm()->evaluateAsAbsolute(ImmVal) ||
        ImmVal < 0 || ImmVal > 127) {
      Error(S, "invalid membar mask number");
      return MatchOperand_ParseFail;
    }
  }

  while (getLexer().getKind() == AsmToken::Hash) {
    SMLoc TagStart = getLexer().getLoc();
    Parser.Lex(); // Eat the '#'.
    unsigned MaskVal = StringSwitch<unsigned>(Parser.getTok().getString())
      .Case("LoadLoad", 0x1)
      .Case("StoreLoad", 0x2)
      .Case("LoadStore", 0x4)
      .Case("StoreStore", 0x8)
      .Case("Lookaside", 0x10)
      .Case("MemIssue", 0x20)
      .Case("Sync", 0x40)
      .Default(0);

    Parser.Lex(); // Eat the identifier token.

    if (!MaskVal) {
      Error(TagStart, "unknown membar tag");
      return MatchOperand_ParseFail;
    }

    ImmVal |= MaskVal;

    if (getLexer().getKind() == AsmToken::Pipe)
      Parser.Lex(); // Eat the '|'.
  }

  EVal = MCConstantExpr::create(ImmVal, getContext());
  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(T8xxOperand::CreateImm(EVal, S, E));
  return MatchOperand_Success;
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

  bool IsPic = getContext().getObjectFileInfo()->isPositionIndependent();
  T8xxMCExpr::VariantKind Kind =
      IsPic ? T8xxMCExpr::VK_T8xx_WPLT30 : T8xxMCExpr::VK_T8xx_WDISP30;

  const MCExpr *DestExpr = T8xxMCExpr::create(Kind, DestValue, getContext());
  Operands.push_back(T8xxOperand::CreateImm(DestExpr, S, E));
  return MatchOperand_Success;
}

OperandMatchResultTy
T8xxAsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {

  OperandMatchResultTy ResTy; // = MatchOperandParserImpl(Operands, Mnemonic);

  // If there wasn't a custom match, try the generic matcher below. Otherwise,
  // there was a match, but an error occurred, in which case, just return that
  // the operand parsing failed.
  if (ResTy == MatchOperand_Success || ResTy == MatchOperand_ParseFail)
    return ResTy;

  if (getLexer().is(AsmToken::LBrac)) {
    // Memory operand
    Operands.push_back(T8xxOperand::CreateToken("[",
                                                 Parser.getTok().getLoc()));
    Parser.Lex(); // Eat the [

    if (Mnemonic == "cas" || Mnemonic == "casx" || Mnemonic == "casa") {
      SMLoc S = Parser.getTok().getLoc();
      if (getLexer().getKind() != AsmToken::Percent)
        return MatchOperand_NoMatch;
      Parser.Lex(); // eat %

      MCRegister Reg;
      unsigned int RegKind;      
      if (!matchRegisterName(Parser.getTok(), Reg, RegKind))
        return MatchOperand_NoMatch;

      Parser.Lex(); // Eat the identifier token.
      SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer()-1);
      Operands.push_back(T8xxOperand::CreateReg(Reg, RegKind, S, E));
      ResTy = MatchOperand_Success;
    } else {
      ResTy = parseMEMOperand(Operands);
    }

    if (ResTy != MatchOperand_Success)
      return ResTy;

    if (!getLexer().is(AsmToken::RBrac))
      return MatchOperand_ParseFail;

    Operands.push_back(T8xxOperand::CreateToken("]",
                                                 Parser.getTok().getLoc()));
    Parser.Lex(); // Eat the ]

    // Parse an optional address-space identifier after the address.
    if (getLexer().is(AsmToken::Integer)) {
      std::unique_ptr<T8xxOperand> Op;
      ResTy = parseT8xxAsmOperand(Op, false);
      if (ResTy != MatchOperand_Success || !Op)
        return MatchOperand_ParseFail;
      Operands.push_back(std::move(Op));
    }
    return MatchOperand_Success;
  }

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
      if (matchT8xxAsmModifiers(EVal, E)) {
	E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
	Op = T8xxOperand::CreateImm(EVal, S, E);
      }
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
      T8xxMCExpr::VariantKind Kind = T8xxMCExpr::VK_T8xx_13;

      if (getContext().getObjectFileInfo()->isPositionIndependent()) {
        if (isCall)
          Kind = T8xxMCExpr::VK_T8xx_WPLT30;
        else
          Kind = T8xxMCExpr::VK_T8xx_GOT13;
      }
      EVal = T8xxMCExpr::create(Kind, EVal, getContext());
    }
    Op = T8xxOperand::CreateImm(EVal, S, E);
    break;
  }
  return (Op) ? MatchOperand_Success : MatchOperand_ParseFail;
}

OperandMatchResultTy
T8xxAsmParser::parseBranchModifiers(OperandVector &Operands) {
  // parse (,a|,pn|,pt)+

  while (getLexer().is(AsmToken::Comma)) {
    Parser.Lex(); // Eat the comma

    if (!getLexer().is(AsmToken::Identifier))
      return MatchOperand_ParseFail;
    StringRef modName = Parser.getTok().getString();
    if (modName == "a" || modName == "pn" || modName == "pt") {
      Operands.push_back(T8xxOperand::CreateToken(modName,
                                                   Parser.getTok().getLoc()));
      Parser.Lex(); // eat the identifier.
    }
  }
  return MatchOperand_Success;
}

bool T8xxAsmParser::matchRegisterName(const AsmToken &Tok, MCRegister &Reg,
                                       unsigned &RegKind) {
  int64_t intVal = 0;
  Reg = 0;
  RegKind = T8xxOperand::rk_None;
  /*
  if (Tok.is(AsmToken::Identifier)) {
    StringRef name = Tok.getString();

    // %fp
    if (name.equals("fp")) {
      RegNo = T8xx::I6;
      RegKind = T8xxOperand::rk_IntReg;
      return true;
    }
    // %sp
    if (name.equals("sp")) {
      RegNo = T8xx::O6;
      RegKind = T8xxOperand::rk_IntReg;
      return true;
    }

    if (name.equals("y")) {
      RegNo = T8xx::Y;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.substr(0, 3).equals_insensitive("asr") &&
        !name.substr(3).getAsInteger(10, intVal) && intVal > 0 && intVal < 32) {
      RegNo = ASRRegs[intVal];
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    // %fprs is an alias of %asr6.
    if (name.equals("fprs")) {
      RegNo = ASRRegs[6];
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("icc")) {
      RegNo = T8xx::ICC;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("psr")) {
      RegNo = T8xx::PSR;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("fsr")) {
      RegNo = T8xx::FSR;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("fq")) {
      RegNo = T8xx::FQ;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("csr")) {
      RegNo = T8xx::CPSR;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("cq")) {
      RegNo = T8xx::CPQ;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("wim")) {
      RegNo = T8xx::WIM;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("tbr")) {
      RegNo = T8xx::TBR;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    if (name.equals("xcc")) {
      // FIXME:: check 64bit.
      RegNo = T8xx::ICC;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    // %fcc0 - %fcc3
    if (name.substr(0, 3).equals_insensitive("fcc") &&
        !name.substr(3).getAsInteger(10, intVal) && intVal < 4) {
      // FIXME: check 64bit and  handle %fcc1 - %fcc3
      RegNo = T8xx::FCC0 + intVal;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }

    // %g0 - %g7
    if (name.substr(0, 1).equals_insensitive("g") &&
        !name.substr(1).getAsInteger(10, intVal) && intVal < 8) {
      RegNo = IntRegs[intVal];
      RegKind = T8xxOperand::rk_IntReg;
      return true;
    }
    // %o0 - %o7
    if (name.substr(0, 1).equals_insensitive("o") &&
        !name.substr(1).getAsInteger(10, intVal) && intVal < 8) {
      RegNo = IntRegs[8 + intVal];
      RegKind = T8xxOperand::rk_IntReg;
      return true;
    }
    if (name.substr(0, 1).equals_insensitive("l") &&
        !name.substr(1).getAsInteger(10, intVal) && intVal < 8) {
      RegNo = IntRegs[16 + intVal];
      RegKind = T8xxOperand::rk_IntReg;
      return true;
    }
    if (name.substr(0, 1).equals_insensitive("i") &&
        !name.substr(1).getAsInteger(10, intVal) && intVal < 8) {
      RegNo = IntRegs[24 + intVal];
      RegKind = T8xxOperand::rk_IntReg;
      return true;
    }
    // %f0 - %f31
    if (name.substr(0, 1).equals_insensitive("f") &&
        !name.substr(1, 2).getAsInteger(10, intVal) && intVal < 32) {
      RegNo = FloatRegs[intVal];
      RegKind = T8xxOperand::rk_FloatReg;
      return true;
    }
    // %f32 - %f62
    if (name.substr(0, 1).equals_insensitive("f") &&
        !name.substr(1, 2).getAsInteger(10, intVal) && intVal >= 32 &&
        intVal <= 62 && (intVal % 2 == 0)) {
      // FIXME: Check V9
      RegNo = DoubleRegs[intVal/2];
      RegKind = T8xxOperand::rk_DoubleReg;
      return true;
    }

    // %r0 - %r31
    if (name.substr(0, 1).equals_insensitive("r") &&
        !name.substr(1, 2).getAsInteger(10, intVal) && intVal < 31) {
      RegNo = IntRegs[intVal];
      RegKind = T8xxOperand::rk_IntReg;
      return true;
    }

    // %c0 - %c31
    if (name.substr(0, 1).equals_insensitive("c") &&
        !name.substr(1).getAsInteger(10, intVal) && intVal < 32) {
      RegNo = CoprocRegs[intVal];
      RegKind = T8xxOperand::rk_CoprocReg;
      return true;
    }

    if (name.equals("tpc")) {
      RegNo = T8xx::TPC;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("tnpc")) {
      RegNo = T8xx::TNPC;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("tstate")) {
      RegNo = T8xx::TSTATE;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("tt")) {
      RegNo = T8xx::TT;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("tick")) {
      RegNo = T8xx::TICK;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("tba")) {
      RegNo = T8xx::TBA;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("pstate")) {
      RegNo = T8xx::PSTATE;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("tl")) {
      RegNo = T8xx::TL;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("pil")) {
      RegNo = T8xx::PIL;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("cwp")) {
      RegNo = T8xx::CWP;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("cansave")) {
      RegNo = T8xx::CANSAVE;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("canrestore")) {
      RegNo = T8xx::CANRESTORE;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("cleanwin")) {
      RegNo = T8xx::CLEANWIN;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("otherwin")) {
      RegNo = T8xx::OTHERWIN;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("wstate")) {
      RegNo = T8xx::WSTATE;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
    if (name.equals("pc")) {
      RegNo = T8xx::PC;
      RegKind = T8xxOperand::rk_Special;
      return true;
    }
  }
  */
  return false;
}

// Determine if an expression contains a reference to the symbol
// "_GLOBAL_OFFSET_TABLE_".
static bool hasGOTReference(const MCExpr *Expr) {
  switch (Expr->getKind()) {
  case MCExpr::Target:
    if (const T8xxMCExpr *SE = dyn_cast<T8xxMCExpr>(Expr))
      return hasGOTReference(SE->getSubExpr());
    break;

  case MCExpr::Constant:
    break;

  case MCExpr::Binary: {
    const MCBinaryExpr *BE = cast<MCBinaryExpr>(Expr);
    return hasGOTReference(BE->getLHS()) || hasGOTReference(BE->getRHS());
  }

  case MCExpr::SymbolRef: {
    const MCSymbolRefExpr &SymRef = *cast<MCSymbolRefExpr>(Expr);
    return (SymRef.getSymbol().getName() == "_GLOBAL_OFFSET_TABLE_");
  }

  case MCExpr::Unary:
    return hasGOTReference(cast<MCUnaryExpr>(Expr)->getSubExpr());
  }
  return false;
}

const T8xxMCExpr *
T8xxAsmParser::adjustPICRelocation(T8xxMCExpr::VariantKind VK,
                                    const MCExpr *subExpr) {
  // When in PIC mode, "%lo(...)" and "%hi(...)" behave differently.
  // If the expression refers contains _GLOBAL_OFFSET_TABLE, it is
  // actually a %pc10 or %pc22 relocation. Otherwise, they are interpreted
  // as %got10 or %got22 relocation.

  if (getContext().getObjectFileInfo()->isPositionIndependent()) {
    switch(VK) {
    default: break;
    case T8xxMCExpr::VK_T8xx_LO:
      VK = (hasGOTReference(subExpr) ? T8xxMCExpr::VK_T8xx_PC10
                                     : T8xxMCExpr::VK_T8xx_GOT10);
      break;
    case T8xxMCExpr::VK_T8xx_HI:
      VK = (hasGOTReference(subExpr) ? T8xxMCExpr::VK_T8xx_PC22
                                     : T8xxMCExpr::VK_T8xx_GOT22);
      break;
    }
  }

  return T8xxMCExpr::create(VK, subExpr, getContext());
}

bool T8xxAsmParser::matchT8xxAsmModifiers(const MCExpr *&EVal,
                                            SMLoc &EndLoc) {
  AsmToken Tok = Parser.getTok();
  if (!Tok.is(AsmToken::Identifier))
    return false;

  StringRef name = Tok.getString();

  T8xxMCExpr::VariantKind VK = T8xxMCExpr::parseVariantKind(name);
  switch (VK) {
  case T8xxMCExpr::VK_T8xx_None:
    Error(getLoc(), "invalid operand modifier");
    return false;

  case T8xxMCExpr::VK_T8xx_GOTDATA_OP:
  case T8xxMCExpr::VK_T8xx_TLS_GD_ADD:
  case T8xxMCExpr::VK_T8xx_TLS_GD_CALL:
  case T8xxMCExpr::VK_T8xx_TLS_IE_ADD:
  case T8xxMCExpr::VK_T8xx_TLS_IE_LD:
  case T8xxMCExpr::VK_T8xx_TLS_IE_LDX:
  case T8xxMCExpr::VK_T8xx_TLS_LDM_ADD:
  case T8xxMCExpr::VK_T8xx_TLS_LDM_CALL:
  case T8xxMCExpr::VK_T8xx_TLS_LDO_ADD:
    // These are special-cased at tablegen level.
    return false;

  default:
    break;
  }

  Parser.Lex(); // Eat the identifier.
  if (Parser.getTok().getKind() != AsmToken::LParen)
    return false;

  Parser.Lex(); // Eat the LParen token.
  const MCExpr *subExpr;
  if (Parser.parseParenExpression(subExpr, EndLoc))
    return false;

  EVal = adjustPICRelocation(VK, subExpr);
  return true;
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
