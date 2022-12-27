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

def SDT_SPCall    : SDTypeProfile<0, -1, [SDTCisVT<0, i32>]>;
def call          : SDNode<"T8xxISD::CALL", SDT_SPCall,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                            SDNPVariadic]>;

def tailcall      : SDNode<"T8xxSD::TAIL_CALL", SDT_SPCall,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                            SDNPVariadic]>;

def SDT_SPRet     : SDTypeProfile<0, 1, [SDTCisVT<0, i32>]>;
//def retflag       : SDNode<"T8xxISD::RET_FLAG", SDT_SPRet,
//                           [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
//def retflag       : SDNode<"T8xxISD::RET_FLAG", SDTNone,
//                           [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def T8xxRetFlag     : SDNode<"T8xxISD::RET_FLAG", SDTNone,
                           [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;


// Signed Operand
def imm32
: Operand<i32> {
}

// Address operand
def mem : Operand<iPTR> {
let PrintMethod = "printMemOperand";
//let MIOperandInfo = (ops GPROut, simm16);
let EncoderMethod = "getMemEncoding";
}

def memsrc : Operand<i32> {
  let MIOperandInfo = (ops IntRegs, i32imm);
  let PrintMethod = "printAddrModeMemSrc";
  let EncoderMethod = "getMemSrcValue";
}

def ADDRrr : ComplexPattern<iPTR, 2, "SelectADDRrr", [], []>;

// Pseudo Instructions

def MOVimmr  : T8InstImm<0x10,
		(outs IntRegs:$dst),
       	 	(ins imm32:$src),
		"ldc $src ; stl $dst",
		[(set i32:$dst, imm:$src)]> {
		let isCodeGenOnly = 0;
		}

def MOVrr  : T8InstImm<0x11,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"ldl $src ; stl $dst",
		[/* No pattern */]> {
		let isCodeGenOnly = 0;
		}

def ADDimmr  : T8InstImm<0x10,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, imm32:$src2),
		"ldl $src1 ; adc $src2 ; stl $dst",
		[(set i32:$dst, (add i32:$src1, imm:$src2))]> {
		let isCodeGenOnly = 0;
		}

def ADDrr : T8Inst<(outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, IntRegs:$src2),
		"ldl $src1 ; ldl $src2 ; add ; stl $dst",
		[(set i32:$dst, (add i32:$src1, i32:$src2))]> {
		let isCodeGenOnly = 0;
		let op = 0x12;
		}

// Benutzt von LEGInstrInfo.cpp "loadRegFromStackSlot"
def LDR  : T8Inst<(outs IntRegs:$val), (ins memsrc:$addr),
                   "ldl $addr ; tstl $val",
                   [(set i32:$val, (load ADDRrr:$addr))]> {
		   let isCodeGenOnly = 0;
		   let op = 0x13;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STR  : T8Inst<(outs), (ins IntRegs:$val, memsrc:$addr),
                   "ldl $val ; tstl $addr",
                   [(store i32:$val, ADDRrr:$addr)]> {
		   let isCodeGenOnly = 0;
		   let op = 0x14;
}


//def MOVmemr  : T8InstImm<0xF,
//		(outs),
//       	 	(ins MEMrr:$addr, IntRegs:$src),
//		"ldl %src; ldc $addr; stnl",
//		[(store i32:$src, ADDRrr:$addr)]> {
//		let isCodeGenOnly = 0;
//		}


//===----------------------------------------------------------------------===//
// Return Instructions
//===----------------------------------------------------------------------===//

let isTerminator = 1, isReturn = 1, isBarrier = 1, Uses = [R15] in {
  def RET : T8Inst<(outs), (ins variable_ops),
                    "bx lr",  [(T8xxRetFlag)]> {
    let op  = 12;
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

