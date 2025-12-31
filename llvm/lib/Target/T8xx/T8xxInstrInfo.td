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

// Adjust workspace
def SDT_AdjWptr : SDTypeProfile<0, 1, [SDTCisVT<0, i32>]>;
def T8xxAdjWPTR : SDNode<"T8xxISD::AJW", SDT_AdjWptr,
    		[SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def AJW   : T8InstImm<0xB,
		(outs),
		(ins i32imm:$imm),
		"ajw $imm",
		[(T8xxAdjWPTR imm:$imm)]>;

// Type definition for multi byte moves by MOVE instruction. This is crucial for
// handling 16bit values.
def SDT_Move : SDTypeProfile<0, 3,   // 3 inputs (Count, SRC, DST)
[
SDTCisVT<0, i32>, // Count
SDTCisPtrTy<1>,   // DST
SDTCisPtrTy<2>   // SRC
]>; // Input chain

def T8xxMOVE : SDNode<"T8xxISD::MOVE", SDT_Move, [SDNPHasChain, SDNPMayLoad, SDNPMayStore,
    	       			       		  SDNPOptInGlue, SDNPOutGlue]>;


// Note: This translates into a pseudo instruction, which is translated into a
// MOVE, LDL and optionally a EXT/ZEXT/SEXT instruction.
// For the second LDL, the used buffer needs to be kept as fourth parameter

def SDT_MoveLoad : SDTypeProfile<1, 4,   // 1 output, 3 inputs (SRC, DST, Count)
[
SDTCisVT<0, i32>, // Result
SDTCisVT<1, i32>, // Count
SDTCisPtrTy<2>,   // AlignedBuffer
SDTCisPtrTy<3>,   // SRC (Unaligned)
SDTCisPtrTy<4>   // AlignedBuffer 2
]>; // Input chain

def T8xxMoveLoad : SDNode<"T8xxISD::MoveLoad", SDT_MoveLoad, [SDNPHasChain, SDNPMayLoad]>;
def T8xxMoveSEXTLoad : SDNode<"T8xxISD::MoveSEXTLoad", SDT_MoveLoad, [SDNPHasChain, SDNPMayLoad]>;
def T8xxMoveZEXTLoad : SDNode<"T8xxISD::MoveZEXTLoad", SDT_MoveLoad, [SDNPHasChain, SDNPMayLoad]>;


// Custom add for instruction pointer (Note: To avoid defining a new register class
// for the instruction pointer, this is designed as a unary pattern
def SDT_AddIptr     : SDTypeProfile<1, 1, [SDTCisPtrTy<0>, SDTCisPtrTy<1>]>;

def T8xxAddIPTR : SDNode<"T8xxISD::ADD_IPTR", SDT_AddIptr, []>;


// A definition of the store instruction to access the parameter
// stack for function calls.

// First parameter is Argument to be saved,
// Second is the stack offset as simple constant

def SDT_T8xxParmStack : SDTypeProfile<0, 2,
                                       [SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;

def T8xxParmStack : SDNode<"T8xxISD::STL_PARM", SDT_T8xxParmStack,
    		    	   [SDNPHasChain, SDNPInGlue, SDNPOutGlue,
			   SDNPMayStore]>;


def load_sym : SDNode<"T8xxISD::LOAD_SYM", SDTIntUnaryOp>;


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
def MxLongArithmetic : SDTypeProfile<1, 3, [
  /* RES 1 */ SDTCisVT<0, i64>,
  /*   LHS */ SDTCisVT<1, i32>,
  /*   RHS */ SDTCisVT<2, i32>,
  /*   CCR */ SDTCisVT<3, i32>
]>;

def MxLongDiff : SDNode<"T8xxISD::LDIFF", MxLongArithmetic>;


def MxStackSelect : SDTypeProfile<1, 1, [
  /* RES       */ SDTCisVT<0, i32>,
  /* AREG/BREG */ SDTCisVT<1, i64>
]>;

def MxSelect : SDNode<"T8xxISD::REV", MxStackSelect>;


def MxStackJoin : SDTypeProfile<1, 2, [
  /* RES       */ SDTCisVT<0, i64>,
  /* AREG/BREG */ SDTCisVT<1, i32>,
  /* AREG/BREG */ SDTCisVT<2, i32>
]>;

def MxJoin : SDNode<"T8xxISD::JOIN", MxStackJoin>;


// ATOMIC_FENCE (copied from MipsInstrInfo.td)

def SDT_Sync             : SDTypeProfile<0, 1, [SDTCisVT<0, i32>]>;

def T8xxSync : SDNode<"T8xxISD::SYNC", SDT_Sync, [SDNPHasChain,SDNPSideEffect]>;

// Sync, Will be plainly removed
def TxSync : T8Inst<(outs), (ins i32imm:$stype), "sync $stype",
           [(T8xxSync imm:$stype)]>
{
    let isCodeGenOnly = 1;
    let hasSideEffects = 1;
    let isPseudo = 1;
}



//===----------------------------------------------------------------------===//
// Conditional Move Pseudo Instructions
//
// CMOV* - Used to implement the SELECT DAG operation. Expanded after
// instruction selection into a branch sequence.
//===----------------------------------------------------------------------===//


def CMOV32 : T8Inst<(outs OReg:$dst),
    		    (ins OReg:$ccr, OReg:$t, OReg:$f),
                    "",
		    [(set i32:$dst,
		    (i32 (MxCmov i32:$ccr, i32:$t, i32:$f)))]> {
    let usesCustomInserter = 1;
    let isCodeGenOnly = 1;
    let isPseudo = 1;
  }


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
//  let DecoderMethod = "DecodeWPtrSrcOperand";
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
		[(set i32:$dst, (add OReg:$src1, imm:$imm))]>;

// Note: MINT, NOT and the pattern were primarily added to create efficient constants for
// the FCOPYSIGN function. Are much faster than an 8 byte LDC sequence for 0x80000000
// and 0x7FFFFFFF.
def MINT :    T8InstOp8<0x42,
                (outs OReg:$dst),
		(ins),
		"mint",
		[(set i32:$dst, -2147483648)]>;

// Note: There is no binary not operation in the LLVM instruction set.
def NOT : T8InstOp8<0x32, (outs OReg:$dst), (ins OReg:$src), "not", []>;

def : Pat<(i32 2147483647),
      	   (NOT (MINT))>;

def LDC  : T8InstImm<0x4,
		(outs OReg:$dst),
       	 	(ins i32imm:$imm),
		"ldc $imm",
		[(set i32:$dst, imm:$imm)]>;

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


def MoveLoad : T8Inst<
    (outs OReg:$res),
    (ins OReg:$cnt, MemSrc:$src_addr, MemSrc:$buf_addr, (WPtrSrc $rs, $imm):$buf_addr2),
    "move_load",
    [(set i32:$res, (T8xxMoveLoad OReg:$cnt, ADDRr:$buf_addr, ADDRr:$src_addr, ADDRri:$buf_addr2))]>
	 {
	 let hasSideEffects = 1;
	 let isPseudo = 1;
	 }

def : Pat<(T8xxMoveLoad i32:$cnt, ADDRri:$buf_addr, ADDRr:$src_addr, ADDRri:$buf_addr2),
      	   (MoveLoad i32:$cnt, (LDLP ADDRri:$buf_addr), ADDRr:$src_addr, ADDRri:$buf_addr2)>;

def : Pat<(T8xxMoveLoad i32:$cnt, ADDRri:$buf_addr, ADDRri:$src_addr, ADDRri:$buf_addr2),
      	   (MoveLoad i32:$cnt, (LDLP ADDRri:$buf_addr), (LDLP ADDRri:$src_addr), ADDRri:$buf_addr2)>;

def : Pat<(T8xxMoveLoad i32:$cnt, ADDRri:$buf_addr, ADDRri8:$src_addr, ADDRri:$buf_addr2),
      	   (MoveLoad i32:$cnt, (LDLP ADDRri:$buf_addr), (LDLPb ADDRri8:$src_addr), ADDRri:$buf_addr2)>;

def MoveSEXTLoad : T8Inst<
    (outs OReg:$res),
    (ins OReg:$cnt, MemSrc:$src_addr, MemSrc:$buf_addr, (WPtrSrc $rs, $imm):$buf_addr2),
    "move_sext_load",
    [(set i32:$res, (T8xxMoveSEXTLoad OReg:$cnt, ADDRr:$buf_addr, ADDRr:$src_addr, ADDRri:$buf_addr2))]>
	 {
	 let hasSideEffects = 1;
	 let isPseudo = 1;
	 }

def : Pat<(T8xxMoveSEXTLoad i32:$cnt, ADDRri:$buf_addr, ADDRr:$src_addr, ADDRri:$buf_addr2),
      	   (MoveSEXTLoad i32:$cnt, (LDLP ADDRri:$buf_addr), ADDRr:$src_addr, ADDRri:$buf_addr2)>;

def : Pat<(T8xxMoveSEXTLoad i32:$cnt, ADDRri:$buf_addr, ADDRri:$src_addr, ADDRri:$buf_addr2),
      	   (MoveSEXTLoad i32:$cnt, (LDLP ADDRri:$buf_addr), (LDLP ADDRri:$src_addr), ADDRri:$buf_addr2)>;

def : Pat<(T8xxMoveSEXTLoad i32:$cnt, ADDRri:$buf_addr, ADDRri8:$src_addr, ADDRri:$buf_addr2),
      	   (MoveSEXTLoad i32:$cnt, (LDLP ADDRri:$buf_addr), (LDLPb ADDRri8:$src_addr), ADDRri:$buf_addr2)>;


def MoveZEXTLoad : T8Inst<
    (outs OReg:$res),
    (ins OReg:$cnt, MemSrc:$src_addr, MemSrc:$buf_addr, (WPtrSrc $rs, $imm):$buf_addr2),
    "move_sext_load",
    [(set i32:$res, (T8xxMoveZEXTLoad OReg:$cnt, ADDRr:$buf_addr, ADDRr:$src_addr, ADDRri:$buf_addr2))]>
	 {
	 let hasSideEffects = 1;
	 let isPseudo = 1;
	 }

def : Pat<(T8xxMoveZEXTLoad i32:$cnt, ADDRri:$buf_addr, ADDRr:$src_addr, ADDRri:$buf_addr2),
      	   (MoveZEXTLoad i32:$cnt, (LDLP ADDRri:$buf_addr), ADDRr:$src_addr, ADDRri:$buf_addr2)>;

def : Pat<(T8xxMoveZEXTLoad i32:$cnt, ADDRri:$buf_addr, ADDRri:$src_addr, ADDRri:$buf_addr2),
      	   (MoveZEXTLoad i32:$cnt, (LDLP ADDRri:$buf_addr), (LDLP ADDRri:$src_addr), ADDRri:$buf_addr2)>;

def : Pat<(T8xxMoveZEXTLoad i32:$cnt, ADDRri:$buf_addr, ADDRri8:$src_addr, ADDRri:$buf_addr2),
      	   (MoveZEXTLoad i32:$cnt, (LDLP ADDRri:$buf_addr), (LDLPb ADDRri8:$src_addr), ADDRri:$buf_addr2)>;


// Needed for 16 bit operations
def MOVE :    T8InstOp8<0x4A, (outs), (ins OReg:$cnt, MemSrc:$dst_addr, MemSrc:$src_addr), "move",
	 [(T8xxMOVE OReg:$cnt, ADDRr:$dst_addr, ADDRr:$src_addr)]>
	 {
	 let hasSideEffects = 1;
	 }

// 4 Byte aligned Frameindices
def : Pat<(T8xxMOVE i32:$cnt, ADDRri:$dst_addr, ADDRr:$src_addr),
      	   (MOVE i32:$cnt, (LDLP ADDRri:$dst_addr), ADDRr:$src_addr)>;

def : Pat<(T8xxMOVE i32:$cnt, ADDRr:$dst_addr, ADDRri:$src_addr),
      	   (MOVE i32:$cnt, ADDRr:$dst_addr, (LDLP ADDRri:$src_addr))>;

def : Pat<(T8xxMOVE i32:$cnt, ADDRri:$dst_addr, ADDRri:$src_addr),
      	   (MOVE i32:$cnt, (LDLP ADDRri:$dst_addr), (LDLP ADDRri:$src_addr))>;

// 1 Byte aligned Frameindices
def : Pat<(T8xxMOVE i32:$cnt, ADDRri8:$dst_addr, ADDRr:$src_addr),
      	   (MOVE (LDLPb ADDRri8:$dst_addr), ADDRr:$src_addr, i32:$cnt)>;

def : Pat<(T8xxMOVE i32:$cnt, ADDRr:$dst_addr, ADDRri8:$src_addr),
      	   (MOVE i32:$cnt, ADDRr:$dst_addr, (LDLPb ADDRri8:$src_addr))>;

def : Pat<(T8xxMOVE i32:$cnt, ADDRri8:$dst_addr, ADDRri8:$src_addr),
      	   (MOVE i32:$cnt, (LDLPb ADDRri:$dst_addr), (LDLPb ADDRri:$src_addr))>;


// Special instruction, which is solely used in CallLowering
def AddWptrImm : T8Inst<(outs OReg:$dst),
    		    (ins WptrRegs:$src, i32imm:$val),
                    "ldlp $val",
		    [(set i32:$dst, (T8xxAddWPTR WptrRegs:$src, (i32 imm:$val)))]> {
    let isCodeGenOnly = 1;
    let isPseudo = 1;
  }

// Instruction used to get addresses relative to the instruction pointer
// for position independent code.
def LDPI :    T8InstOp8<0x1B,
		(outs OReg:$dst),
		(ins OReg:$src),
		"ldpi",
		[(set i32:$dst, (T8xxAddIPTR i32:$src))]>;

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

def : Pat<(i32 (sra (i32 OReg:$src), (i32 OReg:$cnt))),
	  (LSHR (XDBLE OReg:$src), OReg:$cnt)>;

/* Define via patterns */
/*
defm UDIV : T8BinaryOp<0x21, 0xFA, "ldiv", udiv>;   // Pseudo code, ldiv (Section 5.7.2 Compiler writer guide)
defm UREM : T8BinaryOp<0x21, 0xFB, "ldiv", urem>;   // Pseudo code, ldiv -> TODO

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

// Load i8 from global address
def LB : T8InstOp4<0x1,
		(outs OReg:$val),
       	 	(ins MemSrc:$addr),
		"lb",
		[(set i32:$val, (extloadi8 ADDRr:$addr))]> {
		}

def : Pat<(i32 (extloadi8 ADDRri:$addr)),
          (LB (LDLP ADDRri:$addr) )>;

def : Pat<(i32 (extloadi8 ADDRri8:$addr)),
          (LB (LDLPb ADDRri8:$addr) )>;

def : Pat<(i32 (zextloadi8 ADDRr:$addr)),
          (LB ADDRr:$addr)>;

def : Pat<(i32 (zextloadi8 ADDRri:$addr)),
          (LB (LDLP ADDRri:$addr) )>;

def : Pat<(i32 (zextloadi8 ADDRri8:$addr)),
          (LB (LDLPb ADDRri8:$addr) )>;

def : Pat<(i32 (sextloadi8 ADDRr:$addr)),
          (XWORD (LB ADDRr:$addr), (LDC 128))>;

def : Pat<(i32 (sextloadi8 ADDRri:$addr)),
          (XWORD (LB (LDLP ADDRri:$addr) ), (LDC 128))>;

def : Pat<(i32 (sextloadi8 ADDRri8:$addr)),
          (XWORD (LB (LDLPb ADDRri8:$addr) ), (LDC 128))>;

// "Inreg" extensions
def : Pat<(sext_inreg (i32 OReg:$in), i16),
          (XWORD $in, (LDC 32768))>;
def : Pat<(sext_inreg (i32 OReg:$in), i8),
          (XWORD $in, (LDC 128))>;

def : Pat<(i32 (zext (i16 OReg:$in))),
          (AND $in, (LDC 65535))>;
/*
def : Pat<(i32 (zext (i8 OReg:$in))),
          (AND $in, (LDC 255))>;
*/

// Store i32 onto frame
// Note: previously STRi32regop
def STL : T8InstImm<0xD,
		(outs),
       	 	(ins OReg:$val, (WPtrSrc $rs, $imm):$addr),
		"stl $addr",
		[(store i32:$val, ADDRri:$addr)]> {
		}

// TODO: The address pattern allows for 8 bit addresses in order to
// match the DAG and get the test16.c to compile. However, this will not
// work in the general case, since the STL requires a proper 4 Byte aligned
// address to work.
/*
def : Pat<(truncstorei16 i32:$val, ADDRri8:$addr),
          (STL (AND i32:$val, (LDC 65535) ), ADDRri:$addr )>;
	  */

// This pattern is used for the treatment of call parameters
// They are positioned relative to the WPTR but the framepointer
// does support this kind of operation.
def : Pat<(T8xxParmStack i32:$val, imm:$imm),
          (STL i32:$val, WPTR, imm:$imm )>;

// Store i32 to global address
def STNL : T8InstImm<0xE,
		(outs),
       	 	(ins OReg:$val, (RPtrSrc $rs, $imm):$addr),
		"stnl $addr",
		[(store i32:$val, ADDRrr:$addr)]> {
		}

// Store i8 to global address
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
// SETULT +
// SETULE +
// SETUGT +
// SETUGE +
// are used (include/CodeGen/ISDOpcodes.h)

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

// Note: The double EQC is needed since DIFF yields a result != 0
// for true. However, certain operations expect 0 for false and 1 for true.
// TODO: Could be optimized in conjunction with a CJ instruction.
def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETNE)),
           (EQC (EQC (DIFF $rhs, $lhs), 0), 0)>;

def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETEQ)),
           (EQC (DIFF $rhs, $lhs), 0)>;

def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETLT)),
           (GT $rhs, $lhs)>;

def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETGE)),
           (EQC (GT $rhs, $lhs), 0)>;

def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETLE)),
           (EQC (GT $lhs, $rhs), 0)>;

def LDIFF :   T8InstOp8<0x4F,
    	      (outs LReg:$dst),
	      (ins OReg:$src1, OReg:$src2, OReg:$src3),
	      "ldiff",
	      [(set i64:$dst, (MxLongDiff i32:$src1, i32:$src2, i32:$src3))]>;


// Moves the "high word" of an LReg to the destination register
def REV :     T8InstOp4<0x0,
		(outs OReg:$dst),
		(ins LReg:$src),
		"rev",
		[(set i32:$dst, (MxSelect i64:$src))]>;

// Move the "low word" of an LReg to the destination register
// Since the low word is already in the destination register on the operand stack,
// this is a pseudo instruction
def SELLOW :  T8Inst<(outs OReg:$dst),
	             (ins LReg:$src),
	             "sellow",
		     [(set i32:$dst, (MxSelect i64:$src))]>
{
    let isPseudo = 1;
}

def JOIN :  T8Inst<(outs LReg:$dst),
	           (ins OReg:$src1, OReg:$src2),
	           "join",
		   [(set i64:$dst, (MxJoin i32:$src1, i32:$src2))]>
{
    let isPseudo = 1;
}


// Unsigned comparisons are a bit more complex
def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETUGT)),
           (EQC (REV (LDIFF $rhs, $lhs, (LDC 1))), 0)>;   // ab in reference

def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETULT)),
           (EQC (REV (LDIFF $lhs, $rhs, (LDC 1))), 0)>;   // ba in reference

// Greater Equal is NOT Less Than
def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETUGE)),
           (REV (LDIFF $lhs, $rhs, (LDC 1)))>;  // ba in reference

def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETULE)),
           (REV (LDIFF $rhs, $lhs, (LDC 1)))>;      // ab in reference

// Note: Same as SETEQ
def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETUEQ)),
           (EQC (DIFF $lhs, $rhs), 0)>;
// Note: Same as SETNE
def : Pat<(i32 (setcc (i32 OReg:$lhs), (i32 OReg:$rhs), SETUNE)),
           (DIFF $lhs, $rhs)>;


// Unsigned arithmetic operations

def LDIV :    T8InstOp8<0x1A, (outs LReg:$dst), (ins OReg:$src1, OReg:$src2, OReg:$src3), "ldiv", []>;

def : Pat<(udiv (i32 OReg:$lhs), (i32 OReg:$rhs)),
           (SELLOW (LDIV $rhs, $lhs, (LDC 0)) )>;

def : Pat<(urem (i32 OReg:$lhs), (i32 OReg:$rhs)),
           (REV (LDIV $rhs, $lhs, (LDC 0)) )>;


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

def : Pat<(call OReg:$target),
           (CALL $target)>;


def : Pattern<(i32 (load_sym tglobaladdr:$addr)),  [(LDC $addr)]>;
def : Pattern<(i32 (load_sym tglobaltlsaddr:$addr)),  [(LDC $addr)]>;

def : Pattern<(i32 (load_sym texternalsym:$addr)),  [(LDC $addr)]>;

// TODO: Test for constant pool
def : Pattern<(i32 (load_sym tconstpool:$addr)),  [(LDC $addr)]>;

def : Pattern<(i32 (load_sym tjumptable:$addr)),  [(LDC $addr)]>;
def : Pattern<(i32 (load_sym tblockaddress:$addr)),  [(LDC $addr)]>;

//===----------------------------------------------------------------------===//
// Branch Instructions
//===----------------------------------------------------------------------===//

let isTerminator = 1, isBranch = 1 in {
def CJ    : T8InstImm<0xA,
		(outs),
		(ins OReg:$cc, brtarget:$imm),
                "cj $imm",
		[(MxBrNotCond i32:$cc, bb:$imm)]>;
}

let isTerminator = 1, isBranch = 1, isBarrier = 1 in {
def JUMP  : T8InstImm<0x0,
		   (outs),
		   (ins brtarget:$imm),
                   "j $imm",
                   [(br bb:$imm)]>;
}

// Note: Defines a pseudo instruction, which is replaced by a GCALL in T8xxInstrInfo.cpp
// during Pseudo Code replacement.
def BRIND  : T8Inst<(outs),
		   (ins OReg:$target),
                   "brind",
                   [(brind i32:$target)]>
{
   let isTerminator = 1;
   let isBranch = 1;
   let isIndirectBranch = 1;
   let isBarrier = 1;
   let isCodeGenOnly = 1;
   let isPseudo = 1;
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
// Atomic Instructions
// Note: Structurally copied from the MipsInstrInfo.td
// Since the T8xx is single threaded, the atomic instructions do not
// make much sense for this backend. As long as no T8xx task switching
// instructions are used within an instruction sequence, the regular instructions
// should be "atomic".
//===----------------------------------------------------------------------===//

// Atomic instructions with 2 source operands (ATOMIC_SWAP & ATOMIC_LOAD_*).
class Atomic2Ops<PatFrag Op> :
  T8Inst<(outs OReg:$dst), (ins OReg:$ptr, OReg:$incr), "",
           [(set i32:$dst, (Op iPTR:$ptr, i32:$incr))]> {
  let hasNoSchedulingInfo = 1;
  let isCodeGenOnly = 1;    // Note: prevents an error message that the token is not found
  let isPseudo = 1;
}

class Atomic2OpsPostRA :
  T8Inst<(outs OReg:$dst),
         (ins OReg:$ptr, OReg:$incr), "",
	 []> {
  let isCodeGenOnly = 1;    // Note: prevents an error message that the token is not found
  let isPseudo = 1;
  let mayLoad = 1;
  let mayStore = 1;
}

// Atomic Compare & Swap.
// Atomic compare and swap is lowered into two stages. The first stage happens
// during ISelLowering, which produces the PostRA version of this instruction.
class AtomicCmpSwap<PatFrag Op> :
  T8Inst<(outs OReg:$dst), (ins OReg:$ptr, OReg:$cmp, OReg:$swap), "",
           [(set i32:$dst, (Op iPTR:$ptr, i32:$cmp, i32:$swap))]> {
  let hasNoSchedulingInfo = 1;
  let isCodeGenOnly = 1;    // Note: prevents an error message that the token is not found
  let isPseudo = 1;
}

class AtomicCmpSwapPostRA :
  T8Inst<(outs OReg:$dst), (ins OReg:$ptr, OReg:$cmp, OReg:$swap), "", []> {
  let isCodeGenOnly = 1;    // Note: prevents an error message that the token is not found
  let isPseudo = 1;
  let mayLoad = 1;
  let mayStore = 1;
}


let usesCustomInserter = 1 in {
  def ATOMIC_LOAD_ADD_I32  : Atomic2Ops<atomic_load_add_i32>;
  def ATOMIC_LOAD_SUB_I32  : Atomic2Ops<atomic_load_sub_i32>;
  def ATOMIC_LOAD_AND_I32  : Atomic2Ops<atomic_load_and_i32>;
  def ATOMIC_LOAD_OR_I32   : Atomic2Ops<atomic_load_or_i32>;
  def ATOMIC_LOAD_XOR_I32  : Atomic2Ops<atomic_load_xor_i32>;
  def ATOMIC_LOAD_NAND_I32 : Atomic2Ops<atomic_load_nand_i32>;

  def ATOMIC_SWAP_I32      : Atomic2Ops<atomic_swap_i32>;

  def ATOMIC_CMP_SWAP_I32  : AtomicCmpSwap<atomic_cmp_swap_i32>;

  def ATOMIC_LOAD_MIN_I32  : Atomic2Ops<atomic_load_min_i32>;
  def ATOMIC_LOAD_MAX_I32  : Atomic2Ops<atomic_load_max_i32>;
  def ATOMIC_LOAD_UMIN_I32 : Atomic2Ops<atomic_load_umin_i32>;
  def ATOMIC_LOAD_UMAX_I32 : Atomic2Ops<atomic_load_umax_i32>;
}

def ATOMIC_LOAD_ADD_I32_POSTRA  : Atomic2OpsPostRA;
def ATOMIC_LOAD_SUB_I32_POSTRA  : Atomic2OpsPostRA;
def ATOMIC_LOAD_AND_I32_POSTRA  : Atomic2OpsPostRA;
def ATOMIC_LOAD_OR_I32_POSTRA   : Atomic2OpsPostRA;
def ATOMIC_LOAD_XOR_I32_POSTRA  : Atomic2OpsPostRA;
def ATOMIC_LOAD_NAND_I32_POSTRA : Atomic2OpsPostRA;

def ATOMIC_SWAP_I32_POSTRA : Atomic2OpsPostRA;

def ATOMIC_CMP_SWAP_I32_POSTRA : AtomicCmpSwapPostRA;

def ATOMIC_LOAD_MIN_I32_POSTRA  : Atomic2OpsPostRA;
def ATOMIC_LOAD_MAX_I32_POSTRA  : Atomic2OpsPostRA;
def ATOMIC_LOAD_UMIN_I32_POSTRA : Atomic2OpsPostRA;
def ATOMIC_LOAD_UMAX_I32_POSTRA : Atomic2OpsPostRA;


//===----------------------------------------------------------------------===//
// Floating Point Instructions
//===----------------------------------------------------------------------===//

// Include Assembler only opcodes
include "T8xxInstrArithmetic.td"

include "T8xxInstrAliases.td"
