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

def SDT_SPRet     : SDTypeProfile<0, 1, [SDTCisVT<0, i32>]>;

def T8xxRetFlag     : SDNode<"T8xxISD::RET_FLAG", SDTNone,
                           [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

def load_sym : SDNode<"T8xxISD::LOAD_SYM", SDTIntUnaryOp>;


def memsrc : Operand<i32> {
  let MIOperandInfo = (ops IntRegs, i32imm);
  let PrintMethod = "printAddrModeMemSrc";
  let EncoderMethod = "getMemSrcValue";
}



// This relates to a method in T8xxISelDAGtoDAG.cpp
// It implements a function, where indices relative to the stack pointer
// can be automatically emitted in one statement
def ADDRrr : ComplexPattern<iPTR, 2, "SelectADDRrr", [], []>;


// The real hardware assembler codes. Cannot directly be used in the
// DAG translations, since they affect the register stack, which
// is difficult to combine with the "constructed" registers that
// are located in the workspace.

def stl : T8InstImm<0xD,
		(outs IntRegs:$dst),
       	 	(ins),
		"stl $dst",
		[]> {
		let isCodeGenOnly = 0;
		}

def ldl : T8InstImm<0x7,
		(outs),
       	 	(ins IntRegs:$src),
		"ldl $src",
		[]> {
		let isCodeGenOnly = 0;
		}

def stnl : T8InstImm<0xE,
		(outs),
       	 	(ins i32imm:$src),
		"stnl $src",
		[]> {
		let isCodeGenOnly = 0;
		}

def ldnl : T8InstImm<0x3,
		(outs),
       	 	(ins i32imm:$src),
		"ldnl $src",
		[]> {
		let isCodeGenOnly = 0;
		}

def ldc : T8InstImm<0x4,
		(outs),
       	 	(ins i32imm:$src),
		"ldc $src",
		[]> {
		let isCodeGenOnly = 0;
		}

def ADD : T8Inst<(outs),
       	 	(ins),
		"add",
		[]> {
		let isCodeGenOnly = 0;
		let op = 0x13;
		}

def ADDimmr  : T8InstImm<0x12,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src1, i32imm:$src2),
		"ldl $src1 ; adc $src2 ; stl $dst",
		[(set i32:$dst, (add i32:$src1, imm:$src2))]> {
		let isCodeGenOnly = 0;
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
		!strconcat ("ldl $src1 ; ldc $src2 ;", opstr, " ; stl $dst"),
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
//		!strconcat ("ldl $src1 ; ldl $src2 ;", opstr, " ; stl $dst"),
		opstr,
		[(set i32:$dst, (opnode i32:$src1, i32:$src2))]> {
		let isCodeGenOnly = 0;
		let isPseudo = 1;
		}

}

defm ADD : TopStackOp<"add", add>;  // #05
defm SUB : TopStackOp<"sub", sub>;  // #05
defm MUL : TopStackOp<"mul", mul>;  // #53


// Pseudo Instructions

def MOVimmr  : T8xxPseudoInst<(outs IntRegs:$dst),
       	 	(ins i32imm:$src),
		"ldc $src ; stl $dst",
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
def LDR  : T8Inst<(outs IntRegs:$val), (ins memsrc:$addr),
                   "ldc $addr ; ldnl ; stl $val",
                   [(set i32:$val, (load ADDRrr:$addr))]> {
		   let isCodeGenOnly = 1;
		   let op = 0x14;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STR  : T8Inst<(outs), (ins IntRegs:$val, memsrc:$addr),
                   "ldl $val ; ldc $addr ; stnl 0",
                   [(store i32:$val, ADDRrr:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
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

let isTerminator = 1, isReturn = 1, isBarrier = 1, Uses = [R14] in {
  def RET : T8Inst<(outs), (ins variable_ops),
                    "ret",  [(T8xxRetFlag)]> {
    let op  = 0x20;
  }
}


//===----------------------------------------------------------------------===//
// Call Instructions
//===----------------------------------------------------------------------===//

let isCall = 1, Uses = [R15] in {
  def CALL : T8Inst<(outs), (ins IntRegs:$addr),
                      "call $addr",
                      [(call i32:$addr)]> {
    let op = 0x22;
  }
}


//let isCall = 1, Uses = [R15] in {
//  def CALL : T8Inst<(outs), (ins i32imm:$addr),
//                      "call $addr",
//                      [(call globaladdr:$addr)]> {
//    let op = 0x22;
//  }
//}

def : Pattern<(i32 (load_sym tglobaladdr:$addr)),  [(MOVimmr $addr)]>;


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

