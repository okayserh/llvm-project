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

def WPtrSrc : Operand<iPTR> {
  let PrintMethod = "printAddrModeMemSrc";
  let MIOperandInfo = (ops WptrRegs, i32imm);
//  let EncoderMethod = "getMemSrcValue";
  let DecoderMethod = "DecodeWPtrSrcOperand";
  let ParserMatchClass = T8xxWPtrSrcAsmOperand;
}

// Branch targets have OtherVT type.
def brtarget : Operand<OtherVT> {
  let EncoderMethod = "getMemSrcValue";
}


/*
* Complex patterns
*/

// This relates to a method in T8xxISelDAGtoDAG.cpp
// It implements a function, where indices relative to the stack pointer
// can be automatically emitted in one statement
// (From RISCVInstrInfo: "Necessary because a frameindex can't be matched
// directly in a pattern.")
def ADDRr  : ComplexPattern<iPTR, 1, "SelectADDRr", [], []>;
//def ADDRrr : ComplexPattern<iPTR, 2, "SelectADDRrr", [], []>;
def ADDRri : ComplexPattern<iPTR, 2, "SelectADDRri", [frameindex], []>;

def ADDRri8 : ComplexPattern<iPTR, 2, "SelectADDRrib", [frameindex], []>;

// The real hardware assembler codes. Cannot directly be used in the
// DAG translations, since they affect the register stack, which
// is difficult to combine with the "constructed" registers that
// are located in the workspace.

/* J 0x0 is left out */
//def LDLP  : T8InstImm<0x1, "ldlp">;
/* PFIX 0x2 is left out */
//def LDNL  : T8InstImm<0x3, "ldnl">;
//def LDC   : T8InstImm<0x4, "ldc">;

def LDNLP : T8InstImm<0x5,
		(outs OReg:$dst),
		(ins OReg:$src, i32imm:$src2),
		"ldnlp $src2",
		[]> {
		let imm = 0x0;
		}

/* NFIX 0x6 is left out */
//def LDL   : T8InstImm<0x7, "ldl">;
//def ADC   : T8InstImm<0x8, "adc">;
/* Call 0x9 */
/* CJ 0xA */
/* Check if this makes sense?
def AJW   : T8InstImm<0xB,
		(outs WptrRegs:$dst),
		(ins WptrRegs:$src1, i32imm:$src2),
		"ajw $src2",
		[]> {
		let imm = 0x0;
		}
*/
def AJW   : T8InstImm<0xB,
		(outs),
		(ins i32imm:$src2),
		"ajw $src2",
		[]> {
		let imm = 0x0;
		}

def EQC   : T8InstImm<0xC,
		(outs OReg:$dst),
		(ins OReg:$src1, i32imm:$src2),
		"eqc $src2",
		[]> {
		let imm = 0x0;
		}

def REV :     T8InstOp4<0x0,
		(outs),
		(ins),
		"rev",
		[]>;

def DIFF :    T8InstOp4<0x4,
                (outs OReg:$dst),
       	 	(ins OReg:$src1, OReg:$src2),
		"diff",
		[]>;

def GT :      T8InstOp4<0x9,
                (outs OReg:$dst),
       	 	(ins OReg:$src1, OReg:$src2),
		"gt",
		[]>;

def GCALL :   T8InstOp4<0x6,
    	        (outs),
		(ins MemSrc:$addr),
		"gcall",
		[]>;

def LDPI :    T8InstOp8<0x1B,
		(outs OReg:$dst),
		(ins OReg:$src),
		"ldpi",
		[]>;


def IN :      T8InstOp4<0x7, (outs), (ins OReg:$cnt, MemSrc:$chan, MemSrc:$addr), "in", []>;
def OUT :     T8InstOp4<0xB, (outs), (ins OReg:$cnt, MemSrc:$chan, MemSrc:$addr), "out", []>;

/*

//def STL   : T8InstImm<0xD, "stl">;
//def STNL  : T8InstImm<0xE, "stnl">;
// OPR 0xF

// Instructions with 1 byte length
//def LB :      T8InstOp4<0x1, "lb">;
def BSUB :    T8InstOp4<0x2, "bsub">;
def ENDP :    T8InstOp4<0x3, "endp">;
//def ADD :     T8InstOp4<0x5, "add">;
def PROD :    T8InstOp4<0x8, "prod">;
def WSUB :    T8InstOp4<0xA, "wsub">;
//def SUB :     T8InstOp4<0xC, "sub">;
def STARTP :  T8InstOp4<0xD, "startp">;
def OUTBYTE : T8InstOp4<0xE, "outbyte">;
def OUTWORD : T8InstOp4<0xF, "outword">;

// Instructions with 2 byte length
def SETERR :  T8InstOp8<0x10, "seterr">;
def RESETCH : T8InstOp8<0x12, "resetch">;
def CSUB0 :   T8InstOp8<0x13, "csub0">;
def STOPP :   T8InstOp8<0x15, "stopp">;
def LADD :    T8InstOp8<0x16, "ladd">;
def STLB :    T8InstOp8<0x17, "stlb">;
def STHF :    T8InstOp8<0x18, "sthf">;
def NORM :    T8InstOp8<0x19, "norm">;
def LDIV :    T8InstOp8<0x1A, "ldiv">;
//def LDPI :    T8InstOp8<0x1B, "ldpi">;
def STLF :    T8InstOp8<0x1C, "stlf">;
def XDBLE :   T8InstOp8<0x1D, "xdble">;
def LDPRI :   T8InstOp8<0x1E, "ldpri">;
//def REM :     T8InstOp8<0x1F, "rem">;

// TODO: Machine definition vs. return instruction
//def RET :   T8InstOp8<0x20, "ret">;
def LEND :    T8InstOp8<0x21, "lend">;
def LDTIMER : T8InstOp8<0x22, "ltimer">;
def TESTERR : T8InstOp8<0x29, "testerr">;
def TESTPRANAL : T8InstOp8<0x2A, "testpranal">;
def TIN :     T8InstOp8<0x2B, "tin">;
//def DIV :     T8InstOp8<0x2C, "div">;
def DIST :    T8InstOp8<0x2E, "dist">;
def DISC :    T8InstOp8<0x2F, "disc">;

def DISS :    T8InstOp8<0x30, "diss">;
def LMUL :    T8InstOp8<0x31, "lmul">;
def NOT :     T8InstOp8<0x32, "not">;
//def XOR :     T8InstOp8<0x33, "xor">;
def BCNT :    T8InstOp8<0x34, "bcnt">;
def LSHR :    T8InstOp8<0x35, "lshr">;
def LSHL :    T8InstOp8<0x36, "lshl">;
def LSUM :    T8InstOp8<0x37, "lsum">;
def LSUB :    T8InstOp8<0x38, "lsub">;
def RUNP :    T8InstOp8<0x39, "runp">;
//def XWORD :   T8InstOp8<0x3A, "xword">;
//def SB :      T8InstOp8<0x3B, "sb">;
def GAJW :    T8InstOp8<0x3C, "gajw">;
def SAVEL :   T8InstOp8<0x3D, "savel">;
def SAVEH :   T8InstOp8<0x3E, "saveh">;
def WCNT :    T8InstOp8<0x3F, "wcnt">;

//def SHR :     T8InstOp8<0x40, "shr">;
//def SHL :     T8InstOp8<0x41, "shl">;
def MINT :    T8InstOp8<0x42, "mint">;
def ALT :     T8InstOp8<0x43, "alt">;
def ALTWT :   T8InstOp8<0x44, "altwt">;
def ALTEND :  T8InstOp8<0x45, "altend">;
//def AND :     T8InstOp8<0x46, "and">;
def ENBT :    T8InstOp8<0x47, "enbt">;
def ENBC :    T8InstOp8<0x48, "enbc">;
def ENBS :    T8InstOp8<0x49, "enbs">;
def MOVE :    T8InstOp8<0x4A, "move">;
//def OR :      T8InstOp8<0x4B, "or">;
def CSNGL :   T8InstOp8<0x4C, "csgnl">;
def CCNT1 :   T8InstOp8<0x4D, "ccnt1">;
def TALT :    T8InstOp8<0x4E, "talt">;
def LDIFF :   T8InstOp8<0x4F, "ldiff">;

def STHB :        T8InstOp8<0x50, "sthb">;
def TALTWT :      T8InstOp8<0x51, "taltwt">;
def SUM :         T8InstOp8<0x52, "sum">;
//def MUL :         T8InstOp8<0x53, "mul">;
def STTIMER :     T8InstOp8<0x54, "sttimer">;
def STOPERR :     T8InstOp8<0x55, "stoperr">;
def CWORD :       T8InstOp8<0x56, "cword">;
def CLRHALTERR :  T8InstOp8<0x57, "clrhalterr">;
def SETHALTERR :  T8InstOp8<0x58, "sethalterr">;
def TESTHALTERR : T8InstOp8<0x59, "testhalterr">;
def DUP :         T8InstOp8<0x5A, "dup">;
def MOVE2DINIT :  T8InstOp8<0x5B, "move2dinit">;
def MOVE2DALL :   T8InstOp8<0x5C, "move2dall">;
def MOVE2DNONZERO : T8InstOp8<0x5D, "move2dnonzero">;
def MOVE2DZERO :  T8InstOp8<0x5E, "move2dzero">;

def UNPACKSN :   T8InstOp8<0x63, "unpacksn">;
def POSTNORMSN : T8InstOp8<0x6C, "postnormsn">;
def ROUNDSN :    T8InstOp8<0x6D, "roundsn">;

def LDINF :       T8InstOp8<0x71, "ldinf">;
def FMUL :        T8InstOp8<0x72, "fmul">;
def CFLERR :      T8InstOp8<0x73, "cflerr">;
def CRCWORD :     T8InstOp8<0x74, "crcword">;
def CRCBYTE :     T8InstOp8<0x75, "crcbyte">;
def BITCNT :      T8InstOp8<0x76, "bitcnt">;
def BITREVWORD :  T8InstOp8<0x77, "bitrevword">;
def BITREVNBITS : T8InstOp8<0x78, "bitrevnbits">;

def WSUBDB :     T8InstOp8<0x81, "wsubdb">;
def FPLDNLDBI :  T8InstOp8<0x82, "fpldnldbi">;
def FPCHKERR :   T8InstOp8<0x83, "fpchkerr">;
def FPSTNLDB :   T8InstOp8<0x84, "fpstnldb">;
def FPLDNLSNI :  T8InstOp8<0x86, "fpldnlsni">;
def FPADD :      T8InstOp8<0x87, "fpadd">;
def FPSTNLSN :   T8InstOp8<0x88, "fpstnlsn">;
def FPSUB :      T8InstOp8<0x89, "fpsub">;
def FPLDNLDB :   T8InstOp8<0x8A, "fpldnldb">;
def FPMUL :      T8InstOp8<0x8B, "fpmul">;
def FPDIV :      T8InstOp8<0x8C, "fpdiv">;
def FPLDNLSN :   T8InstOp8<0x8E, "fpldnlsn">;
def FPREMFIRST : T8InstOp8<0x8F, "fpremfirst">;

def FPREMSTEP :   T8InstOp8<0x90, "fpremstep">;
def FPNAN :       T8InstOp8<0x91, "fpnan">;
def FPORDERED :   T8InstOp8<0x92, "fpordered">;
def FPNOTFINITE : T8InstOp8<0x93, "fpnotfinite">;
def FPGT :        T8InstOp8<0x94, "fpgt">;
def FPEQ :        T8InstOp8<0x95, "fpeq">;
def FPI32TOR32 :  T8InstOp8<0x96, "fpi32tor32">;
def FPI32TOR64 :  T8InstOp8<0x98, "fpi32tor64">;
def FPB32TOR64 :  T8InstOp8<0x9A, "fpb32tor64">;
def FPTESTERR :   T8InstOp8<0x9C, "fptesterr">;
def FPRTOI32 :    T8InstOp8<0x9D, "fprtoi32">;
def FPSTNLI32 :   T8InstOp8<0x9E, "fpstnli32">;
def FPLDZEROSN :  T8InstOp8<0x9F, "fpldzerosn">;

def FPLDZERODB :  T8InstOp8<0xA0, "fpldnldbi">;
def FPINT :       T8InstOp8<0xA1, "fpldnldbi">;
def FPDUP :       T8InstOp8<0xA3, "fpldnldbi">;
def FPREV :       T8InstOp8<0xA4, "fpldnldbi">;
def FPLDNLADDDB : T8InstOp8<0xA6, "fpldnldbi">;
def FPLDNLMULDB : T8InstOp8<0xA8, "fpldnldbi">;
def FPLDNLADDSN : T8InstOp8<0xAA, "fpldnldbi">;
def FPENTRY :     T8InstOp8<0xAB, "fpldnldbi">;
def FPLDNLMULSN : T8InstOp8<0xAC, "fpldnldbi">;
*/

/*
def FPUSQRTFIRST : T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPUSQRTSTEP :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPUSQRTLAST :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPURP :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPURM :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPURZ :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPUR32TOR64 :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPUR64TOR32 :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPUR64TOR32 :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
def FPUR64TOR32 :  T8InstOp8<0x2A, 0xF2, "fpldnldbi">;
*/


// Some special instructions to move or add immediate values

def ADC  : T8InstImm<0x8,
		(outs OReg:$dst),
       	 	(ins OReg:$src1, i32imm:$src2),
		"adc $src2",
		[(set OReg:$dst, (add OReg:$src1, imm:$src2))]> {
		let imm = 0x0;
		}

def LDC  : T8InstImm<0x4,
		(outs OReg:$dst),
       	 	(ins i32imm:$src),
		"ldc $src",
		[(set OReg:$dst, imm:$src)]> {
		let imm = 0x0;
		}

// Taken from SparcInstrInfo.td
// "LEA" forms of add (patterns to make tblgen happy)

// Goes into postprocessing for unaligned addresses, which cannot be directly handled by ldlp
def LDLP   : T8InstImm<0x1,
		(outs OReg:$rd),
      		(ins WPtrSrc:$addr),
                "ldlp $addr",
                [(set iPTR:$rd, ADDRri:$addr)]> {
		let imm = 0x0;
}

def LDLPb   : T8Inst<(outs OReg:$rd),
      		(ins WPtrSrc:$addr),
                "ldlp $addr",
                [(set iPTR:$rd, ADDRri8:$addr)]> {
		let isCodeGenOnly = 1;
		let isPseudo = 1;
}

// Special instruction, which is solely used in CallLowering
def AddWptrImm : T8Inst<(outs OReg:$dst),
    		    (ins WptrRegs:$src, i32imm:$val),
                    "ldlp $val",
		    [(set i32:$dst, (T8xxAddWPTR WptrRegs:$src, imm:$val))]> {
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

def DIV : T8BinaryOp8<0x2C, "div", sdiv>;
def REM : T8BinaryOp8<0x1F, "rem", srem>;

def XOR : T8BinaryOp8<0x33, "xor", xor>;  // #53
def OR  : T8BinaryOp8<0x4B, "or", or>;  // #53
def AND : T8BinaryOp8<0x46, "and", and>;  // #53

//def : InstAlias<"add $src1, $src2", (ADDregregop OReg:$src1, OReg:$src2), 0>;


/* Define via patterns */
/*
defm UDIV : T8BinaryOp<0x21, 0xFA, "ldiv", udiv>;   // Pseudo code, ldiv (Section 5.7.2 Compiler writer guide)
defm UREM : T8BinaryOp<0x21, 0xFB, "ldiv", urem>;   // Pseudo code, ldiv -> TODO

defm SRA : T8BinaryOp<0x01, 0x21, "lshr", sra>;  // #53   (Pseudo Code, needs transformation)
defm ROTR : T8BinaryOp<0x01, 0x22, "lshr", rotr>;
defm ROTL : T8BinaryOp<0x01, 0x23, "lshr", rotl>;
*/


def XWORD  : T8InstOp8<0x3A,
		(outs OReg:$dst),
       	 	(ins OReg:$src1, OReg:$src2),
		"xword",
		[]>;


// Load i32 from frame index
def LDL : T8InstImm<0x7,
		(outs OReg:$val),
       	 	(ins WPtrSrc:$addr),
		"ldl $addr",
		[(set i32:$val, (load ADDRri:$addr))]>
{
let imm = 0x0;
}


// Load i32 from global address
// TODO: See if the offset via the immediate can be utilized
def LDNL : T8InstImm<0x3,
		(outs OReg:$val),
       	 	(ins MemSrc:$addr),
		"ldnl 0",
		[(set i32:$val, (load ADDRr:$addr))]>
{
let imm = 0x0;
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

//defm LDRi8 :  T8Load<"lb \n", extloadi8>;
//defm LDRzi8 :  T8Load<"lb \n", zextloadi8>;
//defm LDRsi8 :  T8Load<"lb \n\tldc #128 \n\txword \n", sextloadi8>;

//defm LDRzi16 : T8Load<"ldl $addr \n", zextloadi16>;
//defm LDRsi16 : T8Load<"ldl $addr \n\tldc #32768 \n\txword \n", sextloadi16>;


def : Pat<(zextloadi8 ADDRr:$addr),
          (LB ADDRr:$addr)>;

def : Pat<(zextloadi8 ADDRri:$addr),
          (LB (LDLP ADDRri:$addr) )>;

def : Pat<(zextloadi8 ADDRri8:$addr),
          (LB (LDLPb ADDRri8:$addr) )>;

def : Pat<(sextloadi8 ADDRr:$addr),
          (XWORD (LDC 128), (LB ADDRr:$addr) )>;

def : Pat<(sextloadi8 ADDRri:$addr),
          (XWORD (LDC 128), (LB (LDLP ADDRri:$addr) ) )>;

def : Pat<(sextloadi8 ADDRri8:$addr),
          (XWORD (LDC 128), (LB (LDLPb ADDRri8:$addr) ) )>;


// Store i32 onto frame
// Note: previously STRi32regop
def STL : T8InstImm<0xD,
		(outs),
       	 	(ins OReg:$val, WPtrSrc:$addr),
		"stl $addr",
		[(store i32:$val, ADDRri:$addr)]> {
		let imm = 0x0;
		}

// Store i32 to global address
// TODO: See if immediate offset can be utilized
def STNL : T8InstImm<0xE,
		(outs),
       	 	(ins OReg:$val, MemSrc:$addr),
		"stnl 0",
		[(store i32:$val, ADDRr:$addr)]> {
		let imm = 0x0;
		}

// Store i8 to global address
// Note: previously STRi8regop
def SB : T8InstOp8<0x3B,
		(outs),
       	 	(ins OReg:$val, MemSrc:$addr),
		"sb",
		[(truncstorei8 i32:$val, ADDRr:$addr)]> {
		}


def CMP : T8xxPseudoInst<(outs), (ins i32imm:$cc, OReg:$lhs, OReg:$rhs),
                    "cmp$cc $lhs, $rhs", []> {
  let Defs = [CPSR];
}


/* Pattern to introduce an LEA in order to get the Framepointer
 into a operand register */
def : Pat<(truncstorei8 i32:$val, ADDRri:$addr),
          (SB i32:$val, (LDLP ADDRri:$addr) )>;

def : Pat<(truncstorei8 i32:$val, ADDRri8:$addr),
          (SB i32:$val, (LDLPb ADDRri8:$addr) )>;


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
  def CALL : T8Inst<(outs), (ins i32imm:$addr),
                      "ldcipr $addr \n\tadc -4 \n\tldpi \n\tgcall \n\trev \n",
                      [(call tglobaladdr:$addr)]> {
    let isCodeGenOnly = 1;
    let isPseudo = 1;
  }
}

def : Pattern<(i32 (load_sym tglobaladdr:$addr)),  [(LDC $addr)]>;



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


def BRimm2  : T8InstImm<0x0,
		   (outs),
		   (ins brtarget:$addr),
                   "j $addr",
                   [(br bb:$addr)]> {
		   let isTerminator = 1;
		   let isBranch = 1;
		   let imm = 0x0;
}


let isTerminator = 1, isBranch = 1, Uses = [CPSR] in {
  def Bcc : T8InstImm<0xA,
		(outs), 
		(ins brtarget:$dst),
                "cj $dst",
		[]> {
		let imm = 0x0;
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
