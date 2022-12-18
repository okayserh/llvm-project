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
def call          : SDNode<"SPISD::CALL", SDT_SPCall,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                            SDNPVariadic]>;

def tailcall      : SDNode<"SPISD::TAIL_CALL", SDT_SPCall,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                            SDNPVariadic]>;

def SDT_SPRet     : SDTypeProfile<0, 1, [SDTCisVT<0, i32>]>;
def retflag       : SDNode<"SPISD::RET_FLAG", SDT_SPRet,
                           [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;


// Benutzt von LEGInstrInfo.cpp "copyPhysReg"
def jump : T8InstImm<0x0,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"j $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def ldlp : T8InstImm<0x1,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"ldlp $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def pfix : T8InstImm<0x2,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"pfix $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def ldnl : T8InstImm<0x3,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"ldnl $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def ldc : T8InstImm<0x4,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"ldc $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def ldnlp : T8InstImm<0x5,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"ldnlp $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def nfix : T8InstImm<0x6,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"nfix $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def ldl : T8InstImm<0x7,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"ldl $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def adc : T8InstImm<0x8,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"adc $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def cal : T8InstImm<0x9,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"cal $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def cj : T8InstImm<0xA,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"cj $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def ajw : T8InstImm<0xB,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"ajw $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def eqc : T8InstImm<0xC,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"eqc $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def stl : T8InstImm<0xD,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"stl $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def stnl : T8InstImm<0xE,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"stnl $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
		}

def opr : T8InstImm<0xF,
		(outs IntRegs:$dst),
       	 	(ins IntRegs:$src),
		"opr $src",
		[(set i32:$dst, i32:$src)]> {
		let isCodeGenOnly = 0;
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

