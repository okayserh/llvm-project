//===-- T8xxInstrInfo.td - Target Description for T8xx Target -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the T8xx instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

include "T8xxInstrFormats.td"

//===----------------------------------------------------------------------===//
// Feature predicates.
//===----------------------------------------------------------------------===//

// TODO

//===----------------------------------------------------------------------===//
// Instruction Pattern Stuff
//===----------------------------------------------------------------------===//

//  These are target-independent nodes, but have target-specific formats.
def SDT_SPCallSeqStart : SDCallSeqStart<[ SDTCisVT<0, i32>,
                                          SDTCisVT<1, i32> ]>;
def SDT_SPCallSeqEnd   : SDCallSeqEnd<[ SDTCisVT<0, i32>,
                                        SDTCisVT<1, i32> ]>;

def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_SPCallSeqStart,
                           [SDNPHasChain, SDNPOutGlue]>;
def callseq_end   : SDNode<"ISD::CALLSEQ_END",   SDT_SPCallSeqEnd,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;


// SDTCisVT is a type constraint. I.e. has exactly that type.

def SDT_SPCall    : SDTypeProfile<0, -1, [SDTCisVT<0, i32>]>;

// This is the ISD representation of a call. This is later
// matched by the concrete CALL instruction defined below.
def call          : SDNode<"T8xxISD::CALL", SDT_SPCall,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                            SDNPVariadic]>;

def SDT_SPRet     : SDTypeProfile<0, 1, [SDTCisVT<0, i32>]>;

def T8xxRetFlag     : SDNode<"T8xxISD::RET_FLAG", SDTNone,
                           [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

// Custom add for workspace pointer
def SDT_AddWptr     : SDTypeProfile<1, 2, [SDTCisPtrTy<0>, SDTCisPtrTy<1>, SDTCisVT<1, i32>]>;

def T8xxAddWPTR : SDNode<"T8xxISD::ADD_WPTR", SDT_AddWptr, []>;

def load_sym : SDNode<"T8xxISD::LOAD_SYM", SDTIntUnaryOp>;


def MxSDT_EQ : SDTypeProfile<1, 2, [
  /*  Cond */ SDTCisVT<0, i32>,  // Actually a boolean. However on the Transputer those are represented as 32 bit ints
  /*  ARG1 */ SDTCisVT<1, i32>,
  /*  ARG2 */ SDTCisVT<2, i32>
]>;

def MxEQ : SDNode<"T8xxISD::EQ",  MxSDT_EQ>;

/* The transputer conditional jump goes the conditional branch on
  false! Hence we introduce a special code for this one */

def MxSDTBrnotcond : SDTypeProfile<0, 2, [       // brnotcond
  SDTCisInt<0>, SDTCisVT<1, OtherVT>
]>;

def MxBrNotCond     : SDNode<"T8xxISD::BRNCOND"     , MxSDTBrnotcond, [SDNPHasChain]>;


/* Definitions for Conditional Move Pseudo instruction */

def MxSDT_Cmov : SDTypeProfile<1, 3, [
  /*  CCR */ SDTCisVT<1, i32>,
  /*  ARG */ SDTCisSameAs<0, 2>,
  /*  ARG */ SDTCisSameAs<2, 3>
]>;

def MxCmov    : SDNode<"T8xxISD::CMOV",        MxSDT_Cmov>;


/* Example for an operation that yields two results. Might be used for
the long arithmetic operations of the Transputer (i.e. ldiff etc.) */
def MxLongArithmetic : SDTypeProfile<2, 3, [
  /* RES 1 */ SDTCisInt<0>,
  /* RES 2 */ SDTCisInt<1>,
  /*   LHS */ SDTCisSameAs<0, 2>,
  /*   RHS */ SDTCisSameAs<0, 3>,
  /*   CCR */ SDTCisSameAs<1, 4>
]>;

def MxLongDiff : SDNode<"T8xxISD::LDIFF", MxLongArithmetic>;


def MxStackSelect : SDTypeProfile<1, 2, [
  /* RES */ SDTCisInt<0>,
  /*   AREG */ SDTCisSameAs<0, 1>,
  /*   BREG */ SDTCisSameAs<0, 2>,
]>;

def MxSelect : SDNode<"T8xxISD::REV", MxStackSelect>;


//===----------------------------------------------------------------------===//
// Conditional Move Pseudo Instructions
//
// CMOV* - Used to implement the SELECT DAG operation. Expanded after
// instruction selection into a branch sequence.
//===----------------------------------------------------------------------===//

/*
let usesCustomInserter = 1, Uses = [CCR] in
class MxCMove<MxType TYPE>
    : MxPseudo<(outs TYPE.ROp:$dst), (ins TYPE.ROp:$t, TYPE.ROp:$f, i8imm:$cond),
               [(set TYPE.VT:$dst,
                     (TYPE.VT (MxCmov TYPE.VT:$t, TYPE.VT:$f, imm:$cond, CCR)))]>;

def CMOV8d  : MxCMove<MxType8d>;
def CMOV16d : MxCMove<MxType16d>;
def CMOV32r : MxCMove<MxType32r>;
*/


def CMOV32 : T8Inst<(outs OReg:$dst),
    		    (ins OReg:$ccr, OReg:$t, OReg:$f),
                    "",
		    [(set i32:$dst,
		    (i32 (MxCmov i32:$ccr, i32:$t, i32:$f)))]> {
    let usesCustomInserter = 1;
    let isCodeGenOnly = 1;
    let isPseudo = 1;
  }


// TODO: Create a regular pseudo T8xx instruction


// Taken from LEG, matched by ComplexPattern ADDRri

// The parameter MIOperandInfo defines the contribution for the
// MIOperandInfo. It is a register and an immediate.

// This affects "T8xxAsmParser.cpp". Methods with the "Name" suffix
// must be present in that class.
// is<Name>
// add<Name>Operands
//

def T8xxWPtrSrcAsmOperand : AsmOperandClass {
  let Name = "WPtrSrc";
  let ParserMethod = "parseWPtrOperand";
}

def MemSrc : Operand<i32> {
  let PrintMethod = "printOperand";
  let MIOperandInfo = (ops OReg);
//  let EncoderMethod = "getMemSrcValue";
}


def RPtrSrc : Operand<iPTR> {
  let PrintMethod = "printAddrModeMemSrc";
  let MIOperandInfo = (ops OReg, i32imm);
//  let EncoderMethod = "getMemSrcValue";
  let ParserMatchClass = T8xxWPtrSrcAsmOperand;
}

def WPtrSrc : Operand<iPTR> {
  let PrintMethod = "printAddrModeMemSrc";
  let MIOperandInfo = (ops WptrRegs, i32imm);
//  let EncoderMethod = "getMemSrcValue";
  let DecoderMethod = "DecodeWPtrSrcOperand";
  let ParserMatchClass = T8xxWPtrSrcAsmOperand;
}

// Branch targets have OtherVT type.
def brtarget : Operand<OtherVT> {
  let EncoderMethod = "getCallTargetOpValue";
}


/*
* Complex patterns
*/

// This relates to a method in T8xxISelDAGtoDAG.cpp
// It implements a function, where indices relative to the stack pointer
// can be automatically emitted in one statement
// (From RISCVInstrInfo: "Necessary because a frameindex can't be matched
// directly in a pattern.")

// Direct address in AREG
def ADDRr  : ComplexPattern<iPTR, 1, "SelectADDRr", [], []>;

// Word aligned address as offset to WPTR
def ADDRri : ComplexPattern<iPTR, 2, "SelectADDRri", [frameindex], []>;

// Byte aligned address as offset to WPTR
def ADDRri8 : ComplexPattern<iPTR, 2, "SelectADDRrib", [frameindex], []>;

// Word aligned address as offset to AREG
def ADDRrr  : ComplexPattern<iPTR, 2, "SelectADDRrr", [], []>;


// Include Assembler only opcodes
include "T8xxInstrASM.td"


// Some special instructions to move or add immediate values
def ADC  : T8InstImm<0x8,
		(outs OReg:$dst),
       	 	(ins OReg:$src1, i32imm:$imm),
		"adc $imm",
		[(set OReg:$dst, (add OReg:$src1, imm:$imm))]>;

def LDC  : T8InstImm<0x4,
		(outs OReg:$dst),
       	 	(ins i32imm:$imm),
		"ldc $imm",
		[(set OReg:$dst, imm:$imm)]>;

// Taken from SparcInstrInfo.td
// "LEA" forms of add (patterns to make tblgen happy)

// Goes into postprocessing for unaligned addresses, which cannot be directly handled by ldlp
def LDLP   : T8InstImm<0x1,
		(outs OReg:$rd),
      		(ins (WPtrSrc $rs, $imm):$addr),
                "ldlp $addr",
                [(set iPTR:$rd, ADDRri:$addr)]> {
}

def LDLPb   : T8Inst<(outs OReg:$rd),
      		(ins (WPtrSrc $rs, $imm):$addr),
                "ldlp $addr",
                [(set iPTR:$rd, ADDRri8:$addr)]> {
		let isCodeGenOnly = 1;
		let isPseudo = 1;
}

// Special instruction, which is solely used in CallLowering
def AddWptrImm : T8Inst<(outs OReg:$dst),
    		    (ins WptrRegs:$src, i32imm:$val),
                    "ldlp $val",
		    [(set i32:$dst, (T8xxAddWPTR WptrRegs:$src, (i32 imm:$val)))]> {
    let isCodeGenOnly = 1;
    let isPseudo = 1;
  }


class T8BinaryOp4<bits<4> opval, string opstr, SDNode opnode> :
      T8InstOp4<opval,
                (outs OReg:$dst),
       	 	(ins OReg:$src1, OReg:$src2),
		opstr,
		[(set i32:$dst, (opnode i32:$src1, i32:$src2))]>
{
}

class T8BinaryOp8<bits<8> opval, string opstr, SDNode opnode> :
      T8InstOp8<opval,
                (outs OReg:$dst),
       	 	(ins OReg:$src1, OReg:$src2),
		opstr,
		[(set i32:$dst, (opnode i32:$src1, i32:$src2))]>
{
}


def ADD : T8BinaryOp4<0x5, "add", add>;
def SUB : T8BinaryOp4<0xC, "sub", sub>;
def MUL : T8BinaryOp8<0x53, "mul", mul>;
def SHL : T8BinaryOp8<0x41, "shl", shl>;
def SHR : T8BinaryOp8<0x40, "shr", srl>;

// TODO: DIV and REM may need to have switched operands!!!
// Transputer: BREG / AREG
// LLVM:  OP1 / OP2

def DIV : T8BinaryOp8<0x2C, "div", sdiv>;
def REM : T8BinaryOp8<0x1F, "rem", srem>;

def XOR : T8BinaryOp8<0x33, "xor", xor>;  // #53
def OR  : T8BinaryOp8<0x4B, "or", or>;  // #53
def AND : T8BinaryOp8<0x46, "and", and>;  // #53



def : Pat<(udiv (i32 OReg:$lhs), (i32 OReg:$rhs)),
           (LDIV $rhs, $lhs, (LDC 0) )>;


// Dummy instruction to test the code generated
def SRA : T8BinaryOp8<0x23, "sra", sra>;  // #53

/*def : Pat<(sra OReg:$cnt, OReg:$src),
	  (LSHR OReg:$cnt, (XDBLE OReg:$src) )>;
*/

/* Define via patterns */
/*
defm UDIV : T8BinaryOp<0x21, 0xFA, "ldiv", udiv>;   // Pseudo code, ldiv (Section 5.7.2 Compiler writer guide)
defm UREM : T8BinaryOp<0x21, 0xFB, "ldiv", urem>;   // Pseudo code, ldiv -> TODO

defm SRA : T8BinaryOp<0x01, 0x21, "lshr", sra>;  // #53   (Pseudo Code, needs transformation)
defm ROTR : T8BinaryOp<0x01, 0x22, "lshr", rotr>;
defm ROTL : T8BinaryOp<0x01, 0x23, "lshr", rotl>;
*/


// Load i32 from frame index
def LDL : T8InstImm<0x7,
		(outs OReg:$val),
       	 	(ins (WPtrSrc $rs, $imm):$addr),
		"ldl $addr",
		[(set i32:$val, (load ADDRri:$addr))]>;


// Load i32 from global address
def LDNL : T8InstImm<0x3,
		(outs OReg:$val),
       	 	(ins (RPtrSrc $rs, $imm):$addr),
		"ldnl $addr",
		[(set i32:$val, (load ADDRrr:$addr))]>
{
}


/*
defm LDRzi8mem :  T8LoadMem<"ldl $addr \n\tlb \n", zextloadi8>;
defm LDRzi16mem :  T8LoadMem<"ldl $addr \n\tldnl 0 \n\tldc #FFFF \n\tand \n", zextloadi16>;

defm LDRsi8mem :  T8LoadMem<"ldl $addr \n\tlb \n\tldc #128 \n\txword \n", sextloadi8>;
defm LDRsi16mem :  T8LoadMem<"ldl $addr \n\tldc #32768 \n\txword \n", sextloadi16>;
*/

// Load i8 from global address
def LB : T8InstOp4<0x1,
		(outs OReg:$val),
       	 	(ins MemSrc:$addr),
		"lb",
		[(set i32:$val, (extloadi8 ADDRr:$addr))]> {
		}

def : Pat<(extloadi8 ADDRri:$addr),
          (LB (LDLP ADDRri:$addr) )>;

def : Pat<(extloadi8 ADDRri8:$addr),
          (LB (LDLPb ADDRri8:$addr) )>;

def : Pat<(zextloadi8 ADDRr:$addr),
          (LB ADDRr:$addr)>;

def : Pat<(zextloadi8 ADDRri:$addr),
          (LB (LDLP ADDRri:$addr) )>;

def : Pat<(zextloadi8 ADDRri8:$addr),
          (LB (LDLPb ADDRri8:$addr) )>;

def : Pat<(sextloadi8 ADDRr:$addr),
          (XWORD (LB ADDRr:$addr), (LDC 128))>;

def : Pat<(sextloadi8 ADDRri:$addr),
          (XWORD (LB (LDLP ADDRri:$addr) ), (LDC 128))>;

def : Pat<(sextloadi8 ADDRri8:$addr),
          (XWORD (LB (LDLPb ADDRri8:$addr) ), (LDC 128))>;

//defm LDRzi16 : T8Load<"ldl $addr \n", zextloadi16>;
//defm LDRsi16 : T8Load<"ldl $addr \n\tldc #32768 \n\txword \n", sextloadi16>;


// Store i32 onto frame
// Note: previously STRi32regop
def STL : T8InstImm<0xD,
		(outs),
       	 	(ins OReg:$val, (WPtrSrc $rs, $imm):$addr),
		"stl $addr",
		[(store i32:$val, ADDRri:$addr)]> {
		}

// Store i32 to global address
def STNL : T8InstImm<0xE,
		(outs),
       	 	(ins OReg:$val, (RPtrSrc $rs, $imm):$addr),
		"stnl $addr",
		[(store i32:$val, ADDRrr:$addr)]> {
		}

// Store i8 to global address
// Note: previously STRi8regop
def SB : T8InstOp8<0x3B,
		(outs),
       	 	(ins OReg:$val, MemSrc:$addr),
		"sb",
		[(truncstorei8 i32:$val, ADDRr:$addr)]> {
		}


/* Pattern to introduce an LEA in order to get the Framepointer
 into a operand register */
def : Pat<(truncstorei8 i32:$val, ADDRri:$addr),
          (SB i32:$val, (LDLP ADDRri:$addr) )>;

def : Pat<(truncstorei8 i32:$val, ADDRri8:$addr),
          (SB i32:$val, (LDLPb ADDRri8:$addr) )>;

/* These are the core comparisons offered by the Transputer */
/// floating point.  For integer, only the
// SETEQ +
// SETNE +
// SETLT +
// SETLE +
// SETGT +
// SETGE +
// SETULT
// SETULE
// SETUGT
// SETUGE

def EQC   : T8InstImm<0xC,
		(outs OReg:$cc),
		(ins OReg:$lhs, i32imm:$imm),
		"eqc $imm",
		[(set i32:$cc,
		(i32 (setcc i32:$lhs, imm:$imm, SETEQ)))]>;

def GT :      T8InstOp4<0x9,
                (outs OReg:$cc),
       	 	(ins OReg:$lhs, OReg:$rhs),
		"gt",
		[(set i32:$cc,
		(i32 (setcc i32:$lhs, i32:$rhs, SETGT)))]>;

/* Other comparisons will be created by patterns */
def : Pat<(setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETNE),
           (DIFF $rhs, $lhs)>;

def : Pat<(setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETEQ),
           (EQC (DIFF $rhs, $lhs), 0)>;

def : Pat<(setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETLT),
           (GT $rhs, $lhs)>;

def : Pat<(setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETGE),
           (EQC (GT $rhs, $lhs), 0)>;

def : Pat<(setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETLE),
           (EQC (DIFF $lhs, $rhs), 0)>;

def LDIFF :   T8InstOp8<0x4F,
    	      (outs OReg:$dst1, OReg:$dst2),
	      (ins OReg:$src1, OReg:$src2, OReg:$src3),
	      "ldiff",
	      [(set i32:$dst1, i32:$dst2, (MxLongDiff i32:$src1, i32:$src2, i32:$src3))]>;


def REV :     T8InstOp4<0x0,
		(outs OReg:$dst),
		(ins OReg:$src1, OReg:$src2),
		"rev",
		[(set i32:$dst, (MxSelect i32:$src1, i32:$src2))]>;


//===----------------------------------------------------------------------===//
// Return Instructions
//===----------------------------------------------------------------------===//

let isTerminator = 1, isReturn = 1, isBarrier = 1 in {
  def RET : T8xxPseudoInst<(outs), (ins variable_ops),
                    "RET",  [(T8xxRetFlag)]> {
    let isCodeGenOnly = 1;
    let isPseudo = 1;
  }
}


//===----------------------------------------------------------------------===//
// Call Instructions
//===----------------------------------------------------------------------===//

// This is inherently linked to the method "LowerCall", where
// the corresponding call is created in the DAG.

// Working version (passed through register)
//let isCall = 1, Uses = [R15] in {
//  def CALL : T8Inst<(outs), (ins OReg:$addr),
//                      "call $addr",
//                      [(call i32:$addr)]> {
//    let op1 = 0x22;
//  }
//}


// Working version, passed directly
// Note: Currently only one return value in the A register is allowed
// Since the gcall exchanges instruction pointer with the A reg,
// the "rev" is required to put the return value (initially stored in Operand
// register A, then pushed down to B) into the A operand register.
// Then the "copyToReg" after the call will shift it into the
// right destination "Frame-Register"
let isCall = 1 in {
/* Version that tries to determine a instruction pointer relative adressing.
def CALL : T8Inst<(outs), (ins i32imm:$addr),
                      "ldcipr $addr \n\tadc -4 \n\tldpi \n\tgcall \n\trev \n",
                      [(call tglobaladdr:$addr)]> {
    let isCodeGenOnly = 1;
    let isPseudo = 1;
  }
*/

// Version with absolute adressing
def CALL : T8Inst<(outs), (ins i32imm:$addr),
                      "ldc $addr\n\tgcall \n\trev \n",
//                      [(call tglobaladdr:$addr)]> {
                      []> {
    let isCodeGenOnly = 1;
    let isPseudo = 1;
  }
}

// TODO: Needs clean up
def : Pat<(call tglobaladdr:$dst),
          (CALL tglobaladdr:$dst)>;

def : Pat<(call texternalsym:$dst),
          (CALL texternalsym:$dst)>;

def : Pattern<(i32 (load_sym tglobaladdr:$addr)),  [(LDC $addr)]>;

def : Pattern<(i32 (load_sym texternalsym:$addr)),  [(LDC $addr)]>;

//===----------------------------------------------------------------------===//
// Branch Instructions
//===----------------------------------------------------------------------===//


// Compiles at least
def BRimm  : T8Inst<(outs), (ins i32imm:$addr),
                   "j $addr",
                   [(brind tglobaladdr:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let isBranch = 1;
}


// Note j is a relative call

let isTerminator = 1, isBranch = 1 in {
def CJ    : T8InstImm<0xA,
		(outs),
		(ins OReg:$cc, brtarget:$imm),
                "cj $imm",
		[(MxBrNotCond i32:$cc, bb:$imm)]>;

def JUMP  : T8InstImm<0x0,
		   (outs),
		   (ins brtarget:$imm),
                   "j $imm",
                   [(br bb:$imm)]> {
		   let isBarrier = 1;
		   }
}


//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//


/* When these instruction use "Defs" with the WPTR,
+ the eliminateFrameIndex functionality somehow does not work
properly.
// Note: Called from CodeGen/PrologEpilogInserter.cpp
*/

let Uses = [WPTR] in {
def ADJCALLSTACKDOWN : T8xxPseudoInst<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                               "ADJCALLSTACKDOWN $amt1, $amt2",
                               [(callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP : T8xxPseudoInst<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                            "ADJCALLSTACKUP $amt1",
                            [(callseq_end timm:$amt1, timm:$amt2)]>;
}



//===----------------------------------------------------------------------===//
// Floating Point Instructions
//===----------------------------------------------------------------------===//

// Include Assembler only opcodes
include "T8xxInstrArithmetic.td"



include "T8xxInstrAliases.td"
