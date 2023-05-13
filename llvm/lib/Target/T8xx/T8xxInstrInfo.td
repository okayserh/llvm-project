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

// The parameter MIOperandInfo defines the contribution for the
// MIOperandInfo. It is a register and an immediate.

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
def LDLP  : T8InstDirect<0x1, "ldlp">;
/* PFIX 0x2 is left out */
def LDNL  : T8InstDirect<0x3, "ldnl">;
def LDC   : T8InstDirect<0x4, "ldc">;
def LDNLP : T8InstDirect<0x5, "ldnlp">;
/* NFIX 0x6 is left out */
def LDL   : T8InstDirect<0x7, "ldl">;
def ADC   : T8InstDirect<0x8, "adc">;
/* Call 0x9 */
/* CJ 0xA */
def AJW   : T8InstDirect<0xB, "ajw">;
def EQC   : T8InstDirect<0xC, "eqc">;
def STL   : T8InstDirect<0xD, "stl">;
def STNL  : T8InstDirect<0xE, "stnl">;
/* OPR 0xF */

// Instructions with 1 byte length
def REV :     T8InstShort<0xF0, "rev">;
def LB :      T8InstShort<0xF1, "lb">;
def BSUB :    T8InstShort<0xF2, "bsub">;
def ENDP :    T8InstShort<0xF3, "endp">;
def DIFF :    T8InstShort<0xF4, "diff">;
def ADD :     T8InstShort<0xF5, "add">;
def GCALL :   T8InstShort<0xF6, "gcall">;
def IN :      T8InstShort<0xF7, "in">;
def PROD :    T8InstShort<0xF8, "prod">;
def GT :      T8InstShort<0xF9, "gt">;
def WSUB :    T8InstShort<0xFA, "wsub">;
def OUT :     T8InstShort<0xFB, "out">;
def SUB :     T8InstShort<0xFC, "sub">;
def STARTP :  T8InstShort<0xFD, "startp">;
def OUTBYTE : T8InstShort<0xFE, "outbyte">;
def OUTWORD : T8InstShort<0xFF, "outword">;

// Instructions with 2 byte length
def SETERR : T8InstOnePFix<0x21, 0xF0, "seterr">;
def RESETCH : T8InstOnePFix<0x21, 0xF2, "resetch">;
def CSUB0 : T8InstOnePFix<0x21, 0xF3, "csub0">;
def STOPP : T8InstOnePFix<0x21, 0xF5, "stopp">;
def LADD : T8InstOnePFix<0x21, 0xF6, "ladd">;
def STLB : T8InstOnePFix<0x21, 0xF7, "stlb">;
def STHF : T8InstOnePFix<0x21, 0xF8, "sthf">;
def NORM : T8InstOnePFix<0x21, 0xF9, "norm">;
def LDIV : T8InstOnePFix<0x21, 0xFA, "ldiv">;
def LDPI : T8InstOnePFix<0x21, 0xFB, "ldpi">;
def STLF : T8InstOnePFix<0x21, 0xFC, "stlf">;
def XDBLE : T8InstOnePFix<0x21, 0xFD, "xdble">;
def LDPRI : T8InstOnePFix<0x21, 0xFE, "ldpri">;
def REM : T8InstOnePFix<0x21, 0xFF, "rem">;

// TODO: Machine definition vs. return instruction
//def RET : T8InstOnePFix<0x22, 0xF0, "ret">;
def LEND : T8InstOnePFix<0x22, 0xF1, "lend">;
def LDTIMER : T8InstOnePFix<0x22, 0xF2, "ltimer">;
def TESTERR : T8InstOnePFix<0x22, 0xF9, "testerr">;
def TESTPRANAL : T8InstOnePFix<0x22, 0xFA, "testpranal">;
def TIN : T8InstOnePFix<0x22, 0xFB, "tin">;
def DIV : T8InstOnePFix<0x22, 0xFC, "div">;
def DIST : T8InstOnePFix<0x22, 0xFE, "dist">;
def DISC : T8InstOnePFix<0x22, 0xFF, "disc">;

def DISS : T8InstOnePFix<0x23, 0xF0, "diss">;
def LMUL : T8InstOnePFix<0x23, 0xF1, "lmul">;
def NOT : T8InstOnePFix<0x23, 0xF2, "not">;
def XOR : T8InstOnePFix<0x23, 0xF3, "xor">;
def BCNT : T8InstOnePFix<0x23, 0xF4, "bcnt">;
def LSHR : T8InstOnePFix<0x23, 0xF5, "lshr">;
def LSHL : T8InstOnePFix<0x23, 0xF6, "lshl">;
def LSUM : T8InstOnePFix<0x23, 0xF7, "lsum">;
def LSUB : T8InstOnePFix<0x23, 0xF8, "lsub">;
def RUNP : T8InstOnePFix<0x23, 0xF9, "runp">;
def XWORD : T8InstOnePFix<0x23, 0xFA, "xword">;
def SB : T8InstOnePFix<0x23, 0xFB, "sb">;
def GAJW : T8InstOnePFix<0x23, 0xFC, "gajw">;
def SAVEL : T8InstOnePFix<0x23, 0xFD, "savel">;
def SAVEH : T8InstOnePFix<0x23, 0xFE, "saveh">;
def WCNT : T8InstOnePFix<0x23, 0xFF, "wcnt">;

def SHR : T8InstOnePFix<0x24, 0xF0, "shr">;
def SHL : T8InstOnePFix<0x24, 0xF1, "shl">;
def MINT : T8InstOnePFix<0x24, 0xF2, "mint">;
def ALT : T8InstOnePFix<0x24, 0xF3, "alt">;
def ALTWT : T8InstOnePFix<0x24, 0xF4, "altwt">;
def ALTEND : T8InstOnePFix<0x24, 0xF5, "altend">;
def AND : T8InstOnePFix<0x24, 0xF6, "and">;
def ENBT : T8InstOnePFix<0x24, 0xF7, "enbt">;
def ENBC : T8InstOnePFix<0x24, 0xF8, "enbc">;
def ENBS : T8InstOnePFix<0x24, 0xF9, "enbs">;
def MOVE : T8InstOnePFix<0x24, 0xFA, "move">;
def OR : T8InstOnePFix<0x24, 0xFB, "or">;
def CSNGL : T8InstOnePFix<0x24, 0xFC, "csgnl">;
def CCNT1 : T8InstOnePFix<0x24, 0xFD, "ccnt1">;
def TALT : T8InstOnePFix<0x24, 0xFE, "talt">;
def LDIFF : T8InstOnePFix<0x24, 0xFF, "ldiff">;

def STHB : T8InstOnePFix<0x25, 0xF0, "sthb">;
def TALTWT : T8InstOnePFix<0x25, 0xF1, "taltwt">;
def SUM : T8InstOnePFix<0x25, 0xF2, "sum">;
def MUL : T8InstOnePFix<0x25, 0xF3, "mul">;
def STTIMER : T8InstOnePFix<0x25, 0xF4, "sttimer">;
def STOPERR : T8InstOnePFix<0x25, 0xF5, "stoperr">;
def CWORD : T8InstOnePFix<0x25, 0xF6, "cword">;
def CLRHALTERR : T8InstOnePFix<0x25, 0xF7, "clrhalterr">;
def SETHALTERR : T8InstOnePFix<0x25, 0xF8, "sethalterr">;
def TESTHALTERR : T8InstOnePFix<0x25, 0xF9, "testhalterr">;
def DUP : T8InstOnePFix<0x25, 0xFA, "dup">;
def MOVE2DINIT : T8InstOnePFix<0x25, 0xFB, "move2dinit">;
def MOVE2DALL : T8InstOnePFix<0x25, 0xFC, "move2dall">;
def MOVE2DNONZERO : T8InstOnePFix<0x25, 0xFD, "move2dnonzero">;
def MOVE2DZERO : T8InstOnePFix<0x25, 0xFE, "move2dzero">;

def UNPACKSN : T8InstOnePFix<0x26, 0xF3, "unpacksn">;
def POSTNORMSN : T8InstOnePFix<0x26, 0xFC, "postnormsn">;
def ROUNDSN : T8InstOnePFix<0x26, 0xFD, "roundsn">;

def LDINF : T8InstOnePFix<0x27, 0xF1, "ldinf">;
def FMUL : T8InstOnePFix<0x27, 0xF2, "fmul">;
def CFLERR : T8InstOnePFix<0x27, 0xF3, "cflerr">;
def CRCWORD : T8InstOnePFix<0x27, 0xF4, "crcword">;
def CRCBYTE : T8InstOnePFix<0x27, 0xF5, "crcbyte">;
def BITCNT : T8InstOnePFix<0x27, 0xF6, "bitcnt">;
def BITREVWORD : T8InstOnePFix<0x27, 0xF7, "bitrevword">;
def BITREVNBITS : T8InstOnePFix<0x27, 0xF8, "bitrevnbits">;

def WSUBDB : T8InstOnePFix<0x28, 0xF1, "wsubdb">;
def FPLDNLDBI : T8InstOnePFix<0x28, 0xF2, "fpldnldbi">;


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

defm SDIV : TopStackOp<"div", sdiv>; 
defm SREM : TopStackOp<"rem", srem>; 

defm UDIV : TopStackOp<"udiv", udiv>;   // Pseudo code, ldiv (Section 5.7.2 Compiler writer guide)
defm UREM : TopStackOp<"urem", urem>;   // Pseudo code, ldiv

defm SRA : TopStackOp<"sra", sra>;  // #53   (Pseudo Code, needs transformation)
defm ROTR : TopStackOp<"rotr", rotr>;
defm ROTL : TopStackOp<"rotl", rotl>;

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
		"MOVimmr; ldc $src",
		[(set i32:$dst, imm:$src)]> {
		let isCodeGenOnly = 1;
		}

def MOVrr  : T8xxPseudoInst<(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"MOVrr; ldl $src ; stl $dst",
		[/* No pattern */]> {
		let isCodeGenOnly = 1;
		}


// Load instruction
def LDRi  : T8Inst<(outs IntRegs:$val), (ins MemSrc:$addr),
                   "LDRi $val; ldl $addr ; stnl",
                   [(set i32:$val, (load ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x14;
}

// i8 load instructions
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


// i16 load instructions
// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def LDRi16  : T8Inst<(outs IntRegs:$val), (ins MemSrc:$addr),
                   "ldc $addr ; lb; stl $val",
                   [(set i32:$val, (extloadi16 ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def LDRzi16  : T8Inst<(outs IntRegs:$val), (ins MemSrc:$addr),
                   "ldc $addr ; lb; stl $val",
                   [(set i32:$val, (zextloadi16 ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
// Note: Irgendwie muss noch die Extension auf ein Word stattfinden
// Vermutlich ldc #0; ldc $addr; lb; xword; stl $val
def LDRsi16  : T8Inst<(outs IntRegs:$val), (ins MemSrc:$addr),
                   "ldc $addr ; lb; stl $val",
                   [(set i32:$val, (sextloadi16 ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}


// Store instructions

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRi  : T8Inst<(outs), (ins IntRegs:$val, MemSrc:$addr),
                   "ldl $val ; stl $addr",
                   [(store i32:$val, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}


def STRi8  : T8Inst<(outs), (ins IntRegs:$val, MemSrc:$addr),
                   "ldl $val ; ldlp $addr ; sb",
                   [(truncstorei8 i32:$val, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}

def STRi16  : T8Inst<(outs), (ins IntRegs:$val, MemSrc:$addr),
                   "ldl $val ; ldlp $addr ; sw",
                   [(truncstorei16 i32:$val, ADDRri:$addr)]> {
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

/*
def STRimm32  : T8Inst<(outs), (ins i32imm:$val, MemSrc:$addr),
                   "STRimm32; ldc $val ; ldlp $addr ; stnl",
                   [(store imm:$val, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}
*/

// Taken from SparcInstrInfo.td
// "LEA" forms of add (patterns to make tblgen happy)

def LEA_ADDri   : T8Inst<(outs IntRegs:$rd),
      		    (ins MemSrc:$addr),
                     "add $addr, $rd",
                     [(set iPTR:$rd, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x99;
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
//                      "call $addr",
                      "ldc $addr \n\tldpi \n\tgcall \n",
                      [(call tglobaladdr:$addr)]> {
    let isCodeGenOnly = 1;
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
//  let isPseudo = 1;
    let op = 0x66;
}

let Defs = [WPTR], Uses = [WPTR] in {
def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                               "ADJCALLSTACKDOWN $amt1, $amt2",
                               [(callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                            "ADJCALLSTACKUP $amt1",
                            [(callseq_end timm:$amt1, timm:$amt2)]>;
}
