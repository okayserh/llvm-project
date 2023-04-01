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

def load_sym : SDNode<"T8xxISD::LOAD_SYM", SDTIntUnaryOp>;


// Taken from LEG, matched by ComplexPattern ADDRri
def MemSrc : Operand<i32> {
  let MIOperandInfo = (ops IntRegs, i32imm);
  let PrintMethod = "printAddrModeMemSrc";
  let EncoderMethod = "getMemSrcValue";
}

// Branch targets have OtherVT type.
def brtarget : Operand<OtherVT> {
  let EncoderMethod = "getMemSrcValue";
}

def cc_val : Operand<i32> {
  let PrintMethod = "printCondCode";
}


/* Note: Operands are related to machine instructions!
Pattern matching works on a different level (i.e. types etc.)
*/

/* Attempt to see, if this removes the need for separate
sextload / zextload instructions */

def Mxloadi8         : PatFrag<(ops node:$ptr), (i8  (load node:$ptr))>;

def MxSExtLoadi16i8  : PatFrag<(ops node:$ptr), (i16 (sextloadi8 node:$ptr))>;
def MxSExtLoadi32i8  : PatFrag<(ops node:$ptr), (i32 (sextloadi8 node:$ptr))>;
def MxSExtLoadi32i16 : PatFrag<(ops node:$ptr), (i32 (sextloadi16 node:$ptr))>;

def MxZExtLoadi8i1   : PatFrag<(ops node:$ptr), (i8  (zextloadi1 node:$ptr))>;
def MxZExtLoadi16i1  : PatFrag<(ops node:$ptr), (i16 (zextloadi1 node:$ptr))>;
def MxZExtLoadi32i1  : PatFrag<(ops node:$ptr), (i32 (zextloadi1 node:$ptr))>;
def MxZExtLoadi16i8  : PatFrag<(ops node:$ptr), (i16 (zextloadi8 node:$ptr))>;
def MxZExtLoadi32i8  : PatFrag<(ops node:$ptr), (i32 (zextloadi8 node:$ptr))>;
def MxZExtLoadi32i16 : PatFrag<(ops node:$ptr), (i32 (zextloadi16 node:$ptr))>;

def MxExtLoadi8i1    : PatFrag<(ops node:$ptr), (i8  (extloadi1 node:$ptr))>;
def MxExtLoadi16i1   : PatFrag<(ops node:$ptr), (i16 (extloadi1 node:$ptr))>;
def MxExtLoadi32i1   : PatFrag<(ops node:$ptr), (i32 (extloadi1 node:$ptr))>;
def MxExtLoadi16i8   : PatFrag<(ops node:$ptr), (i16 (extloadi8 node:$ptr))>;
def MxExtLoadi32i8   : PatFrag<(ops node:$ptr), (i32 (extloadi8 node:$ptr))>;
def MxExtLoadi32i16  : PatFrag<(ops node:$ptr), (i32 (extloadi16 node:$ptr))>;

/*
* Complex patterns
*/

// This relates to a method in T8xxISelDAGtoDAG.cpp
// It implements a function, where indices relative to the stack pointer
// can be automatically emitted in one statement
// (From RISCVInstrInfo: "Necessary because a frameindex can't be matched
// directly in a pattern.")
def ADDRri : ComplexPattern<iPTR, 2, "SelectADDRri", [frameindex], []>;


// The real hardware assembler codes. Cannot directly be used in the
// DAG translations, since they affect the register stack, which
// is difficult to combine with the "constructed" registers that
// are located in the workspace.

/* J 0x0 is left out */
def LDLP  : T8InstDirect<0x1, "ldnlp">;
/* PFIX 0x2 is left out */
def LDNL  : T8InstDirect<0x3, "ldnl">;
def LDC   : T8InstDirect<0x4, "ldc">;
def LDNLP : T8InstDirect<0x5, "ldnlp">;
/* NFIX 0x6 is left out */
def LDL   : T8InstDirect<0x7, "ldl">;
/* ADC 0x8 */
/* Call 0x9 */
/* CJ 0xA */
def AJW   : T8InstDirect<0xB, "ajw">;
def EQC   : T8InstDirect<0xC, "eqc">;
def STL   : T8InstDirect<0xD, "stl">;
def STNL  : T8InstDirect<0xE, "stnl">;
/* OPR 0xF */


def DIFF : T8Inst<(outs),
       	 	(ins),
		"diff",
		[]> {
		let isCodeGenOnly = 0;
		let op = 0xF4;
		}

def ADD : T8Inst<(outs),
       	 	(ins),
		"add",
		[]> {
		let isCodeGenOnly = 0;
		let op = 0xF5;
		}

def GT : T8Inst<(outs),
       	 	(ins),
		"gt",
		[]> {
		let isCodeGenOnly = 0;
		let op = 0xF9;
		}


def REV : T8Inst<(outs),
       	 	(ins),
		"rev",
		[]> {
		let isCodeGenOnly = 0;
		let op = 0x13;
		}

def NOT : T8Inst<(outs),
       	 	(ins),
		"not",
		[]> {
		let isCodeGenOnly = 0;
		let op = 0x13;
		}

def GAJW : T8Inst<(outs),
       	 	(ins),
		"gajw",
		[]> {
		let isCodeGenOnly = 0;
		let op = 0x3C;
		}


def ADDimmr  : T8InstImm<0x12,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, i32imm:$src2),
		"adc $src2",
		[(set i32:$dst, (add i32:$src1, imm:$src2))]> {
		let isCodeGenOnly = 0;
		let isPseudo = 1;
		}


multiclass TopStackOp<string opstr, SDNode opnode>{
// op srcreg, imm -> dstreg
// ldl srcreg
// ldc imm
// op
// stl dstreg

def regimmop : T8xxPseudoInst<
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, i32imm:$src2),
		opstr,
		[(set i32:$dst, (opnode i32:$src1, imm:$src2))]> {
		let isCodeGenOnly = 0;
		let isPseudo = 1;
		}

// op srcreg1, srcreg2 -> dstreg
// ldl srcreg1
// ldl srcreg2
// op
// stl dstreg

def regregop : T8xxPseudoInst<
                (outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, IntRegs:$src2),
		opstr,
		[(set i32:$dst, (opnode i32:$src1, i32:$src2))]> {
		let isCodeGenOnly = 0;
		let isPseudo = 1;
		}

def memmemop : T8xxPseudoInst<
                (outs IntRegs:$dst),
                       (ins IntRegs:$src1, MemSrc:$addr2),
		        opstr,
               [(set i32:$dst, (opnode i32:$src1, (load ADDRri:$addr2)))]> {
               let isCodeGenOnly = 0;
               let isPseudo = 1;
               }
}

defm ADD : TopStackOp<"add", add>;  // #05
defm SUB : TopStackOp<"sub", sub>;  // #05
defm MUL : TopStackOp<"mul", mul>;  // #53
defm SHL : TopStackOp<"shl", shl>;  // #53
defm SHR : TopStackOp<"shr", srl>;  // #53

defm XOR : TopStackOp<"xor", xor>;  // #53
defm OR  : TopStackOp<"or", or>;  // #53
defm AND : TopStackOp<"and", and>;  // #53


def CMP : T8xxPseudoInst<(outs), (ins cc_val:$cc, IntRegs:$lhs, IntRegs:$rhs),
                    "cmp$cc $lhs, $rhs", []> {
  let Defs = [CPSR];
}


// Pseudo Instructions

def MOVimmr  : T8xxPseudoInst<(outs IntRegs:$dst),
       	 	(ins i32imm:$src),
		"ldc $src",
		[(set i32:$dst, imm:$src)]> {
		let isCodeGenOnly = 1;
		}

def MOVrr  : T8xxPseudoInst<(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"ldl $src ; stl $dst",
		[/* No pattern */]> {
		let isCodeGenOnly = 1;
		}


// Benutzt von LEGInstrInfo.cpp "loadRegFromStackSlot"
def LDR  : T8Inst<(outs IntRegs:$val), (ins IntRegs:$addr),
                   "ldl $val; ldl $addr ; stnl",
                   [(set i32:$val, (load i32:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x14;
}

def LDRi  : T8Inst<(outs IntRegs:$val), (ins MemSrc:$addr),
                   "LDRi $val; ldl $addr ; stnl",
                   [(set i32:$val, (load ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x14;
}


// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRg  : T8Inst<(outs), (ins IntRegs:$val, IntRegs:$addr),
                   "ldl $val ; ldl $addr ; stnl 0",
                   [(store i32:$val, i32:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STR  : T8Inst<(outs), (ins IntRegs:$val, IntRegs:$addr),
                   "ldl $val ; stnl 0",
                   [(store i32:$val, i32:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRi  : T8Inst<(outs), (ins IntRegs:$val, MemSrc:$addr),
                   "ldl $val ; stl $addr",
                   [(store i32:$val, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}


// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRi8  : T8Inst<(outs), (ins IntRegs:$val, MemSrc:$addr),
                   "ldl $val ; ldlp $addr ; sb",
                   [(truncstorei8 i32:$val, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}


// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRimm8  : T8Inst<(outs), (ins i32imm:$val, MemSrc:$addr),
                   "ldc $val ; ldlp $addr ; sb",
                   [(truncstorei8 imm:$val, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}


// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRri8  : T8Inst<(outs), (ins IntRegs:$val, IntRegs:$addr),
                   "ldl $val ; ldl $addr ; sb",
                   [(truncstorei8 i32:$val, i32:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def LDRri8  : T8Inst<(outs IntRegs:$val), (ins IntRegs:$addr),
                   "ldl $addr ; lb; stl $val",
                   [(set i32:$val, (extloadi8 i32:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def LDRi8  : T8Inst<(outs IntRegs:$val), (ins MemSrc:$addr),
                   "ldc $addr ; lb; stl $val",
                   [(set i32:$val, (extloadi8 ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def LDRzi8  : T8Inst<(outs IntRegs:$val), (ins MemSrc:$addr),
                   "ldc $addr ; lb; stl $val",
                   [(set i32:$val, (zextloadi8 ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
// Note: Irgendwie muss noch die Extension auf ein Word stattfinden
// Vermutlich ldc #0; ldc $addr; lb; xword; stl $val
def LDRsi8  : T8Inst<(outs IntRegs:$val), (ins MemSrc:$addr),
                   "ldc $addr ; lb; stl $val",
                   [(set i32:$val, (sextloadi8 ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}


//===----------------------------------------------------------------------===//
// Return Instructions
//===----------------------------------------------------------------------===//

let isTerminator = 1, isReturn = 1, isBarrier = 1 in {
  def RET : T8Inst<(outs), (ins variable_ops),
                    "ret",  [(T8xxRetFlag)]> {
    let op  = 0x20;
  }
}


//===----------------------------------------------------------------------===//
// Call Instructions
//===----------------------------------------------------------------------===//

// This is inherently linked to the method "LowerCall", where
// the corresponding call is created in the DAG.

// Working version (passed through register)
//let isCall = 1, Uses = [R15] in {
//  def CALL : T8Inst<(outs), (ins IntRegs:$addr),
//                      "call $addr",
//                      [(call i32:$addr)]> {
//    let op = 0x22;
//  }
//}


// Working version, passed directly
//let isCall = 1, Uses = [R15] in {
let isCall = 1 in {
  def CALL : T8Inst<(outs), (ins i32imm:$addr),
                      "ldl %r1 ; ldl %r2 ; ldl %r3; call $addr",
                      [(call tglobaladdr:$addr)]> {
    let op = 0x22;
  }
}

def : Pattern<(i32 (load_sym tglobaladdr:$addr)),  [(MOVimmr $addr)]>;



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
		   let op = 0x15;
}


def BRimm2  : T8Inst<(outs), (ins brtarget:$addr),
                   "j $addr",
                   [(br bb:$addr)]> {
		   let isTerminator = 1;
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let isBranch = 1;
		   let op = 0x15;
}


let isTerminator = 1, isBranch = 1, Uses = [CPSR] in {
  def Bcc : T8Inst<(outs), (ins brtarget:$dst),
                    "cj $dst",  []> {
		let op = 0x33;
  }
}


//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//

// Pseudo instructions.
class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
   : T8Inst<outs, ins, asmstr, pattern> {
  let isCodeGenOnly = 1;
  let isPseudo = 1;
}

def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                               "!ADJCALLSTACKDOWN $amt1, $amt2",
                               [(callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                            "!ADJCALLSTACKUP $amt1",
                            [(callseq_end timm:$amt1, timm:$amt2)]>;

