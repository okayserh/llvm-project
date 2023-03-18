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


def memsrc : Operand<i32> {
  let MIOperandInfo = (ops IntRegs, i32imm);
  let PrintMethod = "printAddrModeMemSrc";
  let EncoderMethod = "getMemSrcValue";
}


//def stacksrc : Operand<i32> {
//  let MIOperandInfo = (ops WPRegs, i32imm);
//  let PrintMethod = "printAddrModeMemSrc";
//  let EncoderMethod = "getMemSrcValue";
//}



// This relates to a method in T8xxISelDAGtoDAG.cpp
// It implements a function, where indices relative to the stack pointer
// can be automatically emitted in one statement
def ADDRri : ComplexPattern<iPTR, 2, "SelectADDRri", [frameindex], []>;


// The real hardware assembler codes. Cannot directly be used in the
// DAG translations, since they affect the register stack, which
// is difficult to combine with the "constructed" registers that
// are located in the workspace.

def STL : T8InstImm<0xD,
		(outs IntRegs:$dst),
       	 	(ins),
		"stl $dst",
		[]> {
		let isCodeGenOnly = 0;
		}

def STL_stack : T8InstImm<0xD,
		(outs),
       	 	(ins i32imm:$dst),
		"stl_s $dst",
		[]> {
		let isCodeGenOnly = 0;
		}


def LDL : T8InstImm<0x7,
		(outs),
       	 	(ins IntRegs:$src),
		"ldl $src",
		[]> {
		let isCodeGenOnly = 0;
		}

def LDL_stack : T8InstImm<0x7,
		(outs),
       	 	(ins i32imm:$src),
		"ldl_s $src",
		[]> {
		let isCodeGenOnly = 0;
		}

def STNL : T8InstImm<0xE,
		(outs),
       	 	(ins i32imm:$src),
		"stnl $src",
		[]> {
		let isCodeGenOnly = 0;
		}

def LDNL : T8InstImm<0x3,
		(outs),
       	 	(ins i32imm:$src),
		"ldnl $src",
		[]> {
		let isCodeGenOnly = 0;
		}

def LDNLP : T8InstImm<0x3,
		(outs IntRegs:$dst),
       	 	(ins i32imm:$offs),
		"ldnlp $offs ; stl $dst",
		[]> {
		let isCodeGenOnly = 0;
		}

def LDLP : T8InstImm<0x3,
		(outs IntRegs:$dst),
       	 	(ins i32imm:$offs),
		"ldlp $offs ; stl $dst",
		[]> {
		let isCodeGenOnly = 0;
		}

def LDC : T8InstImm<0x4,
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

def AJW : T8Inst<(outs),
       	 	(ins i32imm:$src),
		"ajw $src",
		[]> {
		let isCodeGenOnly = 0;
		let op = 0x13;
		}


def REV : T8Inst<(outs),
       	 	(ins),
		"rev",
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

//def memmemop : T8xxPseudoInst<
//                (outs IntRegs:$dst),
//                       (ins memsrc:$addr1, memsrc:$addr2),
//             opstr,
//               [(set i32:$dst, (opnode (load ADDRrr:$addr1), (load ADDRrr:$addr2)))]> {
//               let isCodeGenOnly = 0;
//               let isPseudo = 1;
//               }
}

defm ADD : TopStackOp<"add", add>;  // #05
defm SUB : TopStackOp<"sub", sub>;  // #05
defm MUL : TopStackOp<"mul", mul>;  // #53
defm SHL : TopStackOp<"shl", shl>;  // #53
defm SHR : TopStackOp<"shr", srl>;  // #53

defm XOR : TopStackOp<"xor", xor>;  // #53
defm OR : TopStackOp<"or", or>;  // #53
defm AND : TopStackOp<"and", and>;  // #53


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
//                   [(set i32:$val, (load ADDRrr:$addr))]> {
                   [(set i32:$val, (load i32:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x14;
}

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRg  : T8Inst<(outs), (ins IntRegs:$val, IntRegs:$addr),
                   "ldl $val ; ldl $addr ; stnl 0",
//                   [(store i32:$val, ADDRrr:$addr)]> {
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
def STRi  : T8Inst<(outs), (ins IntRegs:$val, memsrc:$addr),
                   "ldl $val ; stl $addr",
                   [(store i32:$val, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}



// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRi8  : T8Inst<(outs), (ins IntRegs:$val, memsrc:$addr),
                   "ldl $val ; ldlp $addr ; sb",
                   [(truncstorei8 i32:$val, ADDRri:$addr)]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}


// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
def STRimm8  : T8Inst<(outs), (ins i32imm:$val, memsrc:$addr),
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
/*
def LDRri8  : T8Inst<(outs IntRegs:$val), (ins IntRegs:$addr),
                   "ldl $addr ; lb; stl $val",
                   [(set i32:$val, (extloadi8 i32:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}
*/

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
/*
def LDRi8  : T8Inst<(outs IntRegs:$val), (ins memsrc:$addr),
                   "ldc $addr ; lb; stl $val",
                   [(set i32:$val, (extloadi8 ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}
*/

// Benutzt von LEGInstrInfo.cpp "storeRegToStackSlot"
// Note: Irgendwie muss noch die Extension auf ein Word stattfinden
// Vermutlich ldc #0; ldc $addr; lb; xword; stl $val

/*
def LDRsi8  : T8Inst<(outs IntRegs:$val), (ins memsrc:$addr),
                   "ldc $addr ; lb; stl $val",
                   [(set i32:$val, (sextloadi8 ADDRri:$addr))]> {
		   let isCodeGenOnly = 1;
		   let isPseudo = 1;
		   let op = 0x15;
}
*/




//===----------------------------------------------------------------------===//
// Return Instructions
//===----------------------------------------------------------------------===//

//let isTerminator = 1, isReturn = 1, isBarrier = 1, Uses = [R15] in {
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

