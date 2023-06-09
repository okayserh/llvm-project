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

def MemSrc : Operand<i32> {
  let MIOperandInfo = (ops IntRegs);
  let PrintMethod = "printOperand";
  let EncoderMethod = "getMemSrcValue";
}

def WPtrSrc : Operand<i32> {
  let MIOperandInfo = (ops WptrRegs, i32imm);
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
def ADDRr  : ComplexPattern<iPTR, 1, "SelectADDRr", [], []>;
//def ADDRrr : ComplexPattern<iPTR, 2, "SelectADDRrr", [], []>;
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
def SETERR :  T8InstOnePFix<0x21, 0xF0, "seterr">;
def RESETCH : T8InstOnePFix<0x21, 0xF2, "resetch">;
def CSUB0 :   T8InstOnePFix<0x21, 0xF3, "csub0">;
def STOPP :   T8InstOnePFix<0x21, 0xF5, "stopp">;
def LADD :    T8InstOnePFix<0x21, 0xF6, "ladd">;
def STLB :    T8InstOnePFix<0x21, 0xF7, "stlb">;
def STHF :    T8InstOnePFix<0x21, 0xF8, "sthf">;
def NORM :    T8InstOnePFix<0x21, 0xF9, "norm">;
def LDIV :    T8InstOnePFix<0x21, 0xFA, "ldiv">;
def LDPI :    T8InstOnePFix<0x21, 0xFB, "ldpi">;
def STLF :    T8InstOnePFix<0x21, 0xFC, "stlf">;
def XDBLE :   T8InstOnePFix<0x21, 0xFD, "xdble">;
def LDPRI :   T8InstOnePFix<0x21, 0xFE, "ldpri">;
def REM :     T8InstOnePFix<0x21, 0xFF, "rem">;

// TODO: Machine definition vs. return instruction
//def RET :   T8InstOnePFix<0x22, 0xF0, "ret">;
def LEND :    T8InstOnePFix<0x22, 0xF1, "lend">;
def LDTIMER : T8InstOnePFix<0x22, 0xF2, "ltimer">;
def TESTERR : T8InstOnePFix<0x22, 0xF9, "testerr">;
def TESTPRANAL : T8InstOnePFix<0x22, 0xFA, "testpranal">;
def TIN :     T8InstOnePFix<0x22, 0xFB, "tin">;
def DIV :     T8InstOnePFix<0x22, 0xFC, "div">;
def DIST :    T8InstOnePFix<0x22, 0xFE, "dist">;
def DISC :    T8InstOnePFix<0x22, 0xFF, "disc">;

def DISS :    T8InstOnePFix<0x23, 0xF0, "diss">;
def LMUL :    T8InstOnePFix<0x23, 0xF1, "lmul">;
def NOT :     T8InstOnePFix<0x23, 0xF2, "not">;
def XOR :     T8InstOnePFix<0x23, 0xF3, "xor">;
def BCNT :    T8InstOnePFix<0x23, 0xF4, "bcnt">;
def LSHR :    T8InstOnePFix<0x23, 0xF5, "lshr">;
def LSHL :    T8InstOnePFix<0x23, 0xF6, "lshl">;
def LSUM :    T8InstOnePFix<0x23, 0xF7, "lsum">;
def LSUB :    T8InstOnePFix<0x23, 0xF8, "lsub">;
def RUNP :    T8InstOnePFix<0x23, 0xF9, "runp">;
def XWORD :   T8InstOnePFix<0x23, 0xFA, "xword">;
def SB :      T8InstOnePFix<0x23, 0xFB, "sb">;
def GAJW :    T8InstOnePFix<0x23, 0xFC, "gajw">;
def SAVEL :   T8InstOnePFix<0x23, 0xFD, "savel">;
def SAVEH :   T8InstOnePFix<0x23, 0xFE, "saveh">;
def WCNT :    T8InstOnePFix<0x23, 0xFF, "wcnt">;

def SHR :     T8InstOnePFix<0x24, 0xF0, "shr">;
def SHL :     T8InstOnePFix<0x24, 0xF1, "shl">;
def MINT :    T8InstOnePFix<0x24, 0xF2, "mint">;
def ALT :     T8InstOnePFix<0x24, 0xF3, "alt">;
def ALTWT :   T8InstOnePFix<0x24, 0xF4, "altwt">;
def ALTEND :  T8InstOnePFix<0x24, 0xF5, "altend">;
def AND :     T8InstOnePFix<0x24, 0xF6, "and">;
def ENBT :    T8InstOnePFix<0x24, 0xF7, "enbt">;
def ENBC :    T8InstOnePFix<0x24, 0xF8, "enbc">;
def ENBS :    T8InstOnePFix<0x24, 0xF9, "enbs">;
def MOVE :    T8InstOnePFix<0x24, 0xFA, "move">;
def OR :      T8InstOnePFix<0x24, 0xFB, "or">;
def CSNGL :   T8InstOnePFix<0x24, 0xFC, "csgnl">;
def CCNT1 :   T8InstOnePFix<0x24, 0xFD, "ccnt1">;
def TALT :    T8InstOnePFix<0x24, 0xFE, "talt">;
def LDIFF :   T8InstOnePFix<0x24, 0xFF, "ldiff">;

def STHB :        T8InstOnePFix<0x25, 0xF0, "sthb">;
def TALTWT :      T8InstOnePFix<0x25, 0xF1, "taltwt">;
def SUM :         T8InstOnePFix<0x25, 0xF2, "sum">;
def MUL :         T8InstOnePFix<0x25, 0xF3, "mul">;
def STTIMER :     T8InstOnePFix<0x25, 0xF4, "sttimer">;
def STOPERR :     T8InstOnePFix<0x25, 0xF5, "stoperr">;
def CWORD :       T8InstOnePFix<0x25, 0xF6, "cword">;
def CLRHALTERR :  T8InstOnePFix<0x25, 0xF7, "clrhalterr">;
def SETHALTERR :  T8InstOnePFix<0x25, 0xF8, "sethalterr">;
def TESTHALTERR : T8InstOnePFix<0x25, 0xF9, "testhalterr">;
def DUP :         T8InstOnePFix<0x25, 0xFA, "dup">;
def MOVE2DINIT :  T8InstOnePFix<0x25, 0xFB, "move2dinit">;
def MOVE2DALL :   T8InstOnePFix<0x25, 0xFC, "move2dall">;
def MOVE2DNONZERO : T8InstOnePFix<0x25, 0xFD, "move2dnonzero">;
def MOVE2DZERO :  T8InstOnePFix<0x25, 0xFE, "move2dzero">;

def UNPACKSN :   T8InstOnePFix<0x26, 0xF3, "unpacksn">;
def POSTNORMSN : T8InstOnePFix<0x26, 0xFC, "postnormsn">;
def ROUNDSN :    T8InstOnePFix<0x26, 0xFD, "roundsn">;

def LDINF :       T8InstOnePFix<0x27, 0xF1, "ldinf">;
def FMUL :        T8InstOnePFix<0x27, 0xF2, "fmul">;
def CFLERR :      T8InstOnePFix<0x27, 0xF3, "cflerr">;
def CRCWORD :     T8InstOnePFix<0x27, 0xF4, "crcword">;
def CRCBYTE :     T8InstOnePFix<0x27, 0xF5, "crcbyte">;
def BITCNT :      T8InstOnePFix<0x27, 0xF6, "bitcnt">;
def BITREVWORD :  T8InstOnePFix<0x27, 0xF7, "bitrevword">;
def BITREVNBITS : T8InstOnePFix<0x27, 0xF8, "bitrevnbits">;

def WSUBDB :     T8InstOnePFix<0x28, 0xF1, "wsubdb">;
def FPLDNLDBI :  T8InstOnePFix<0x28, 0xF2, "fpldnldbi">;
def FPCHKERR :   T8InstOnePFix<0x28, 0xF3, "fpchkerr">;
def FPSTNLDB :   T8InstOnePFix<0x28, 0xF4, "fpstnldb">;
def FPLDNLSNI :  T8InstOnePFix<0x28, 0xF6, "fpldnlsni">;
def FPADD :      T8InstOnePFix<0x28, 0xF7, "fpadd">;
def FPSTNLSN :   T8InstOnePFix<0x28, 0xF8, "fpstnlsn">;
def FPSUB :      T8InstOnePFix<0x28, 0xF9, "fpsub">;
def FPLDNLDB :   T8InstOnePFix<0x28, 0xFA, "fpldnldb">;
def FPMUL :      T8InstOnePFix<0x28, 0xFB, "fpmul">;
def FPDIV :      T8InstOnePFix<0x28, 0xFC, "fpdiv">;
def FPLDNLSN :   T8InstOnePFix<0x28, 0xFE, "fpldnlsn">;
def FPREMFIRST : T8InstOnePFix<0x28, 0xFF, "fpremfirst">;

def FPREMSTEP :   T8InstOnePFix<0x29, 0xF0, "fpremstep">;
def FPNAN :       T8InstOnePFix<0x29, 0xF1, "fpnan">;
def FPORDERED :   T8InstOnePFix<0x29, 0xF2, "fpordered">;
def FPNOTFINITE : T8InstOnePFix<0x29, 0xF3, "fpnotfinite">;
def FPGT :        T8InstOnePFix<0x29, 0xF4, "fpgt">;
def FPEQ :        T8InstOnePFix<0x29, 0xF5, "fpeq">;
def FPI32TOR32 :  T8InstOnePFix<0x29, 0xF6, "fpi32tor32">;
def FPI32TOR64 :  T8InstOnePFix<0x29, 0xF8, "fpi32tor64">;
def FPB32TOR64 :  T8InstOnePFix<0x29, 0xFA, "fpb32tor64">;
def FPTESTERR :   T8InstOnePFix<0x29, 0xFC, "fptesterr">;
def FPRTOI32 :    T8InstOnePFix<0x29, 0xFD, "fprtoi32">;
def FPSTNLI32 :   T8InstOnePFix<0x29, 0xFE, "fpstnli32">;
def FPLDZEROSN :  T8InstOnePFix<0x29, 0xFF, "fpldzerosn">;

def FPLDZERODB :  T8InstOnePFix<0x2A, 0xF0, "fpldnldbi">;
def FPINT :       T8InstOnePFix<0x2A, 0xF1, "fpldnldbi">;
def FPDUP :       T8InstOnePFix<0x2A, 0xF3, "fpldnldbi">;
def FPREV :       T8InstOnePFix<0x2A, 0xF4, "fpldnldbi">;
def FPLDNLADDDB : T8InstOnePFix<0x2A, 0xF6, "fpldnldbi">;
def FPLDNLMULDB : T8InstOnePFix<0x2A, 0xF8, "fpldnldbi">;
def FPLDNLADDSN : T8InstOnePFix<0x2A, 0xFA, "fpldnldbi">;
def FPENTRY :     T8InstOnePFix<0x2A, 0xFB, "fpldnldbi">;
def FPLDNLMULSN : T8InstOnePFix<0x2A, 0xFC, "fpldnldbi">;

/*
def FPUSQRTFIRST : T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPUSQRTSTEP :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPUSQRTLAST :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPURP :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPURM :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPURZ :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPUR32TOR64 :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPUR64TOR32 :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPUR64TOR32 :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
def FPUR64TOR32 :  T8InstOnePFix<0x2A, 0xF2, "fpldnldbi">;
*/

// Some special instructions to move or add immediate values

def ADDimmr  : T8InstImm<0x12,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, i32imm:$src2),
		"ldl $src1 \n\tadc $src2 \n\tstl $dst \n",
		[(set IntRegs:$dst, (add IntRegs:$src1, imm:$src2))]> {
		let isCodeGenOnly = 1;
		let op2 = 0x00;
		}

def MOVimmr  : T8xxPseudoInst<(outs IntRegs:$dst),
       	 	(ins i32imm:$src),
		"ldc $src \n\tstl $dst \n",
		[(set i32:$dst, imm:$src)]> {
		let isCodeGenOnly = 1;
		let op2 = 0x00;
		}


// Note: The pseudo assembler code
// will be focused on three types of operand
// - Register
// - Immediate
// - Memory (relativ to register)

multiclass T8BinaryOp<bits<8> pfix1, bits<8> opval, string preop, string opstr1, string opstr2, SDNode opnode>{

def regimmop : T8InstWithOperands<
    	        pfix1, opval,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, i32imm:$src2),
		!strconcat (preop, "ldl $src1 \n", opstr1, "\tldc $src2 \n", opstr2, "\tstl $dst \n"),
		[(set i32:$dst, (opnode i32:$src1, imm:$src2))]>;

def regregop : T8InstWithOperands<
    	        pfix1, opval,
                (outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, IntRegs:$src2),
		!strconcat (preop, "ldl $src1 \n", opstr1, "\tldl $src2 \n", opstr2, "\tstl $dst \n"),
		[(set i32:$dst, (opnode i32:$src1, i32:$src2))]>;


def memmemop : T8InstWithOperands<
    	        pfix1, opval,
                (outs IntRegs:$dst),
                (ins IntRegs:$src1, WPtrSrc:$addr2),
		!strconcat (preop, "ldl $src1 \n", opstr1, "\tldl $addr2 \n", opstr2, "\tstl $dst \n"),
                [(set i32:$dst, (opnode i32:$src1, (load ADDRri:$addr2)))]>;
}

defm ADD : T8BinaryOp<0x00, 0xF5, "", "", "\tadd \n", add>;  // #05
defm SUB : T8BinaryOp<0x00, 0xFC, "", "", "\tsub \n", sub>;  // #05
defm MUL : T8BinaryOp<0x25, 0xF4, "", "", "\tmul \n", mul>;  // #53
defm SHL : T8BinaryOp<0x24, 0xF1, "", "", "\tshl \n", shl>;  // #53
defm SHR : T8BinaryOp<0x24, 0xF0, "", "", "\tshr \n", srl>;  // #53

defm SDIV : T8BinaryOp<0x22, 0xFC, "", "", "\tdiv \n", sdiv>; 
defm SREM : T8BinaryOp<0x21, 0xFF, "", "", "\trem \n", srem>;

defm UDIV : T8BinaryOp<0x21, 0xFA, "ldc 0 \n\t", "", "\tldiv \n", udiv>;   // Pseudo code, ldiv (Section 5.7.2 Compiler writer guide)
defm UREM : T8BinaryOp<0x21, 0xFB, "ldc 0 \n\t", "", "\tldiv \n\trev \n", urem>;   // Pseudo code, ldiv

defm SRA : T8BinaryOp<0x01, 0x21, "", "\txdble \n", "\tlshr \n", sra>;  // #53   (Pseudo Code, needs transformation)
defm ROTR : T8BinaryOp<0x01, 0x22, "", "\tldc 0 \n", "\tlshr \n\tor \n", rotr>;
defm ROTL : T8BinaryOp<0x01, 0x23, "ldc 0 \n\t", "", "\tlshr \t\tor \n", rotl>;

defm XOR : T8BinaryOp<0x01, 0x24, "", "", "\txor \n", xor>;  // #53
defm OR  : T8BinaryOp<0x01, 0x25, "", "", "\tor \n", or>;  // #53
defm AND : T8BinaryOp<0x01, 0x26, "", "", "\tand \n", and>;  // #53

def CMP : T8xxPseudoInst<(outs), (ins cc_val:$cc, IntRegs:$lhs, IntRegs:$rhs),
                    "cmp$cc $lhs, $rhs", []> {
  let Defs = [CPSR];
}


// Load instructions

multiclass T8Load<string opstr, SDPatternOperator opnode>{

def regop : T8xxPseudoInst<
		(outs IntRegs:$val),
       	 	(ins WPtrSrc:$addr),
		!strconcat (opstr, "\tstl $val \n"),
		[(set i32:$val, (opnode ADDRri:$addr))]> {
		let isCodeGenOnly = 1;
		let isPseudo = 1;
		}
}

defm LDRi8 :  T8Load<"ldlp $addr \n\tlb \n", extloadi8>;
defm LDRi16 : T8Load<"ldl $addr \n", extloadi16>;
defm LDRi32 : T8Load<"ldl $addr \n", load>;

defm LDRzi8 :  T8Load<"ldlp $addr \n\tlb \n", zextloadi8>;
defm LDRzi16 : T8Load<"ldl $addr \n", zextloadi16>;

defm LDRsi8 :  T8Load<"ldlp $addr \n\tlb \n\tldc #80 \n\txword \n", sextloadi8>;
defm LDRsi16 : T8Load<"ldl $addr \n\tldc #8000 \n\txword \n", sextloadi16>;


// Load from global addresses

multiclass T8LoadMem<string opstr, SDPatternOperator opnode>{

def regop : T8xxPseudoInst<
		(outs IntRegs:$val),
       	 	(ins MemSrc:$addr),
		!strconcat (opstr, "\tstl $val \n"),
		[(set i32:$val, (opnode ADDRr:$addr))]> {
		let isCodeGenOnly = 1;
		let isPseudo = 1;
		}
}

defm LDRi8mem :  T8LoadMem<"ldl $addr MEM \n\tlb \n", extloadi8>;
defm LDRi16mem : T8LoadMem<"ldl $addr MEM \n\tldnl 0 \n", extloadi16>;
defm LDRi32mem : T8LoadMem<"ldl $addr MEM \n\tldnl 0 \n", load>;

defm LDRzi8mem :  T8LoadMem<"ldl $addr MEM \n\tlb \n", zextloadi8>;
defm LDRzi16mem :  T8LoadMem<"ldl $addr MEM \n\tldnl 0 \n\tldc #FFFF \n\tand \n", zextloadi16>;

defm LDRsi8mem :  T8LoadMem<"ldl $addr MEM \n\tlb \n\tldc #80 \n\txword \n", sextloadi8>;
defm LDRsi16mem :  T8LoadMem<"ldl $addr MEM \n\tldc #8000 \n\txword \n", sextloadi16>;


// Store instructions on frame

multiclass T8Storage<string opstr, SDPatternOperator opnode>{

def regop : T8xxPseudoInst<
		(outs),
       	 	(ins IntRegs:$val, WPtrSrc:$addr),
		!strconcat ("ldl $val \n", opstr),
		[(opnode i32:$val, ADDRri:$addr)]> {
		let isCodeGenOnly = 1;
		let isPseudo = 1;
		}

def immop : T8xxPseudoInst<
		(outs),
       	 	(ins i32imm:$val, WPtrSrc:$addr),
		!strconcat ("ldc $val \n", opstr),
		[(opnode imm:$val, ADDRri:$addr)]> {
		let isCodeGenOnly = 1;
		let isPseudo = 1;
		}
}

defm STRi8 :  T8Storage<"\tldlp $addr \n\tsb \n" , truncstorei8>;
defm STRi16 : T8Storage<"ldc #FFFF \n and \n\tstl $addr \n" ,truncstorei16>;
defm STRi32 : T8Storage<"\tstl $addr \n" ,store>;


// Store instructions on global memory

multiclass T8StorageMem<string opstr, SDPatternOperator opnode>{

def regop : T8xxPseudoInst<
		(outs),
       	 	(ins IntRegs:$val, MemSrc:$addr),
		!strconcat ("ldl $val \n", opstr),
		[(opnode i32:$val, ADDRr:$addr)]> {
		let isCodeGenOnly = 1;
		let isPseudo = 1;
		}

def immop : T8xxPseudoInst<
		(outs),
       	 	(ins i32imm:$val, MemSrc:$addr),
		!strconcat ("ldc $val \n", opstr),
		[(opnode imm:$val, ADDRr:$addr)]> {
		let isCodeGenOnly = 1;
		let isPseudo = 1;
		}
}

defm STRi8mem :  T8StorageMem<"\tldl $addr \n\tsb \n" , truncstorei8>;
defm STRi16mem : T8StorageMem<"\tldc #FFFF \n and \n\tldl $addr \n\tstnl \n" ,truncstorei16>;
defm STRi32mem : T8StorageMem<"\tldl $addr \n\tstnl \n" ,store>;


// Taken from SparcInstrInfo.td
// "LEA" forms of add (patterns to make tblgen happy)

def LEA_ADDri   : T8Inst<(outs IntRegs:$rd),
      		    (ins WPtrSrc:$addr),
                     "ldlp $addr \n\tstl $rd \n",
                     [(set iPTR:$rd, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op1 = 0x99;
}


//===----------------------------------------------------------------------===//
// Return Instructions
//===----------------------------------------------------------------------===//

let isTerminator = 1, isReturn = 1, isBarrier = 1 in {
  def RET : T8Inst<(outs), (ins variable_ops),
                    "ldl %r0 \n\tgcall \n",  [(T8xxRetFlag)]> {
    let op1 = 0x20;
    let op2 = 0x22;
    let isCodeGenOnly = 1;
  }
}


def AddWptrImm : T8Inst<(outs IntRegs:$dst),
    		    (ins WptrRegs:$src, i32imm:$val),
                    "ldlp $val \n\tstl $dst \n",
		    [(set i32:$dst, (T8xxAddWPTR WptrRegs:$src, imm:$val))]> {
    let op1 = 0x20;
    let op2 = 0x23;
    let isCodeGenOnly = 1;
    let isPseudo = 1;
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
                      "ldc $addr \n\tldpi \n\tgcall \n\trev \n",
                      [(call tglobaladdr:$addr)]> {
    let isCodeGenOnly = 1;
    let op1 = 0x22;
    let op2 = 0x25;
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
		   let op1 = 0x15;
		   let op2 = 0x35;
}


def BRimm2  : T8Inst<(outs), (ins brtarget:$addr),
                   "j $addr",
                   [(br bb:$addr)]> {
		   let isTerminator = 1;
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let isBranch = 1;
		   let op1 = 0x15;
		   let op2 = 0x15;
}


let isTerminator = 1, isBranch = 1, Uses = [CPSR] in {
  def Bcc : T8Inst<(outs), (ins brtarget:$dst),
                    "cj $dst",  []> {
		let op1 = 0x33;
		let op2 = 0x55;
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
    let op1 = 0x66;
    let op2 = 0x67;
}


/* When these instruction use "Defs" with the WPTR,
+ the eliminateFrameIndex functionality somehow does not work
properly.
// Note: Called from CodeGen/PrologEpilogInserter.cpp
*/

let Uses = [WPTR] in {
def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                               "ADJCALLSTACKDOWN $amt1, $amt2",
                               [(callseq_start timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                            "ADJCALLSTACKUP $amt1",
                            [(callseq_end timm:$amt1, timm:$amt2)]>;
}
