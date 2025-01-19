//===-- T8xxInstrArithmethic.td - Target Description for T8xx Target -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the T8xx FP instructions in TableGen format.
//
//===----------------------------------------------------------------------===//


/*
Floating point instructions
*/


class T8BinaryFPOp8<bits<8> opval, string opstr, SDNode opnode> :
      T8InstOp8<opval,
                (outs FPReg:$dst),
       	 	(ins FPReg:$src1, FPReg:$src2),
		opstr,
		[(set f32:$dst, (opnode f32:$src1, f32:$src2))]>
{
}

// Arithmethic
def FPADD :      T8BinaryFPOp8<0x87, "fpadd", fadd>;
def FPSUB :      T8BinaryFPOp8<0x89, "fpsub", fsub>;
def FPMUL :      T8BinaryFPOp8<0x8B, "fpmul", fmul>;
def FPDIV :      T8BinaryFPOp8<0x8C, "fpdiv", fdiv>;



// Store f32 to global address
def FPSTNLSN : T8InstOp8<0x88,
		(outs),
       	 	(ins FPReg:$val, (RPtrSrc $rs, $imm):$addr),
		"fpstnlsn $addr",
		[(store f32:$val, ADDRrr:$addr)]> {
		}

// Load f32 from global address
def FPLDNLSN : T8InstOp8<0x8E,
		(outs FPReg:$val),
       	 	(ins (RPtrSrc $rs, $imm):$addr),
		"fpldnlsn $addr",
		[(set f32:$val, (load ADDRrr:$addr))]>
{
}

/*

def FPREMFIRST : T8InstOp8<0x8F, "fpremfirst">;
def FPREMSTEP :   T8InstOp8<0x90, "fpremstep">;

def FPUABS :  	   T8InstFPU4<0xB, "fpuabs">;

// Comparison
def FPGT :        T8InstOp8<0x94, "fpgt">;
def FPEQ :        T8InstOp8<0x95, "fpeq">;

// Load/Store
def FPLDNLDB :   T8InstOp8<0x8A, "fpldnldb">;
def FPLDNLSN :   T8InstOp8<0x8E, "fpldnlsn">;

def FPSTNLDB :   T8InstOp8<0x84, "fpstnldb">;
def FPSTNLSN :   T8InstOp8<0x88, "fpstnlsn">;

def FPLDNLDBI :  T8InstOp8<0x82, "fpldnldbi">;
def FPLDNLSNI :  T8InstOp8<0x86, "fpldnlsni">;

// Type conversion
def FPUR32TOR64 :  T8InstFPU4<0x7, "fpur32tor64">;
def FPUR64TOR32 :  T8InstFPU4<0x8, "fpur64tor32">;



def FPNAN :       T8InstOp8<0x91, "fpnan">;
def FPORDERED :   T8InstOp8<0x92, "fpordered">;
def FPNOTFINITE : T8InstOp8<0x93, "fpnotfinite">;
def FPI32TOR32 :  T8InstOp8<0x96, "fpi32tor32">;
def FPI32TOR64 :  T8InstOp8<0x98, "fpi32tor64">;
def FPB32TOR64 :  T8InstOp8<0x9A, "fpb32tor64">;
def FPTESTERR :   T8InstOp8<0x9C, "fptesterr">;
def FPRTOI32 :    T8InstOp8<0x9D, "fprtoi32">;
def FPSTNLI32 :   T8InstOp8<0x9E, "fpstnli32">;
def FPLDZEROSN :  T8InstOp8<0x9F, "fpldzerosn">;

def FPLDZERODB :  T8InstOp8<0xA0, "fpldzerodb">;
def FPINT :       T8InstOp8<0xA1, "fpint">;
def FPDUP :       T8InstOp8<0xA3, "fpdup">;
def FPREV :       T8InstOp8<0xA4, "fprev">;
def FPLDNLADDDB : T8InstOp8<0xA6, "fpldnladddb">;
def FPLDNLMULDB : T8InstOp8<0xA8, "fpldnlmuldb">;
def FPLDNLADDSN : T8InstOp8<0xAA, "fpldnladdsn">;
def FPENTRY :     T8InstOp8<0xAB, "fpentry">;
def FPLDNLMULSN : T8InstOp8<0xAC, "fpldnlmulsn">;
*/

/* These are in fact combinations of ldc <opcode> and fpentry
def FPUSQRTFIRST : T8InstFPU4<0x1, "fpusqrtfirst">;
def FPUSQRTSTEP :  T8InstFPU4<0x2, "fpusqrtstep">;
def FPUSQRTLAST :  T8InstFPU4<0x3, "fpusqrtlast">;
def FPURP :  	   T8InstFPU4<0x4, "fpurp">;
def FPURM :  	   T8InstFPU4<0x5, "fpurm">;
def FPURZ :  	   T8InstFPU4<0x6, "fpurz">;
def FPUEXPDEC32 :  T8InstFPU4<0x9, "fpuexpdec32">;
def FPUEXPINC32 :  T8InstFPU4<0xA, "fpuexpinc32">;
def FPUNOROUND :   T8InstFPU4<0xD, "fpunoround">;

def FPUCHKI32 :    T8InstFPU4<0xE, "fpuchki32">;
def FPUCHKI64 :    T8InstFPU4<0xF, "fpuchki64">;

def FPUDIVBY2 :    T8InstFPU8<0x11, "fpudivby2">;
def FPUMULBY2 :    T8InstFPU8<0x12, "fpumulby2">;
def FPURN :        T8InstFPU8<0x22, "fpurn">;

def FPCHKERR :   T8InstOp8<0x83, "fpchkerr">;
def FPUSETERR :    T8InstFPU8<0x23, "fpuseterr">;
def FPUCLRERR :    T8InstFPU8<0x9C, "fpuclrerr">;
*/
