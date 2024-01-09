//===-- T8xxInstrASM.td - Target Description for T8xx Target -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the T8xx instructions that are only used for the
// assembler. I.e. no patterns are specified.
//
//===----------------------------------------------------------------------===//

// The real hardware assembler codes. Cannot directly be used in the
// DAG translations, since they affect the register stack, which
// is difficult to combine with the "constructed" registers that
// are located in the workspace.

def PFIX   : T8InstImm<0x2,
		(outs),
		(ins i32imm:$imm),
		"pfix $imm",
		[]>;

def LDNLP : T8InstImm<0x5,
		(outs OReg:$dst),
		(ins OReg:$src, i32imm:$imm),
		"ldnlp $imm",
		[]> {
		}

def NFIX   : T8InstImm<0x6,
		(outs),
		(ins i32imm:$imm),
		"nfix $imm",
		[]>;

// Note: This is the transputer suggested call mechanism. Currently not
// used since the saving of AREG-CREG seems unnecessary for the time being.
def CALLrel   : T8InstImm<0x9,
		(outs),
		(ins brtarget:$imm),
		"call $imm",
		[]>;

def RETrel : T8InstOp8<0x20, (outs), (ins), "ret", []>;


def AJW   : T8InstImm<0xB,
		(outs),
		(ins i32imm:$imm),
		"ajw $imm",
		[]>;

def EQC   : T8InstImm<0xC,
		(outs OReg:$dst),
		(ins OReg:$src1, i32imm:$imm),
		"eqc $imm",
		[]>;

def REV :     T8InstOp4<0x0,
		(outs),
		(ins),
		"rev",
		[]>;

def DUP :         T8InstOp8<0x5A, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$src1), "dup", []>;   // T800 only


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

// Channel communications
def IN :      T8InstOp4<0x7, (outs), (ins OReg:$cnt, MemSrc:$chan, MemSrc:$addr), "in", []>;
def OUT :     T8InstOp4<0xB, (outs), (ins OReg:$cnt, MemSrc:$chan, MemSrc:$addr), "out", []>;
def OUTBYTE:  T8InstOp4<0xE, (outs), (ins OReg:$byte, MemSrc:$chan), "outbyte", []>;
def OUTWORD:  T8InstOp4<0xF, (outs), (ins OReg:$word, MemSrc:$chan), "outword", []>;
def RESETCH : T8InstOp8<0x12, (outs OReg:$dst), (ins OReg:$src), "resetch", []>;

// Diverse functions
def MINT :    T8InstOp8<0x42, (outs OReg:$dst), (ins), "mint", []>;
def BCNT :    T8InstOp8<0x34, (outs OReg:$dst), (ins OReg:$src), "bcnt", []>;
def WCNT :    T8InstOp8<0x3F, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$src), "wcnt", []>;
def MOVE :    T8InstOp8<0x4A, (outs), (ins OReg:$cnt, MemSrc:$src_addr, MemSrc:$dst_addr), "move", []>;

def BSUB :    T8InstOp4<0x2, (outs OReg:$dst), (ins OReg:$src1, OReg:$src2), "bsub", []>;   // Byte subscript
def WSUB :    T8InstOp4<0xA, (outs OReg:$dst), (ins OReg:$src1, OReg:$src2), "wsub", []>;   // Word subscript
def CSUB0 :   T8InstOp8<0x13, (outs), (ins OReg:$src1, OReg:$src2), "csub0", []>;
def CSNGL :   T8InstOp8<0x4C, (outs), (ins OReg:$src1, OReg:$src2), "csgnl", []>;

def PROD :    T8InstOp4<0x8, (outs OReg:$dst), (ins OReg:$src1, OReg:$src2), "prod", []>;  // Product

// TODO: Look if this can be represented somehow in outs/ins
def LEND :    T8InstOp8<0x21, (outs), (ins), "lend", []>;  // Loop end

def SETERR :  T8InstOp8<0x10, (outs), (ins), "seterr", []>;
def TESTERR : T8InstOp8<0x29, (outs OReg:$dst), (ins), "testerr", []>;
def STOPERR : T8InstOp8<0x55, (outs), (ins), "stoperr", []>;

def CLRHALTERR :  T8InstOp8<0x57, (outs), (ins), "clrhalterr", []>;
def SETHALTERR :  T8InstOp8<0x58, (outs), (ins), "sethalterr", []>;
def TESTHALTERR : T8InstOp8<0x59, (outs), (ins), "testhalterr", []>;

def TESTPRANAL : T8InstOp8<0x2A, (outs OReg:$dst1), (ins), "testpranal", []>;

// TODO: Swaps WPTR and AREG. Should be reflected in outs/ins.
def GAJW :    T8InstOp8<0x3C, (outs), (ins), "gajw", []>;

// Long arithmetic
def XDBLE :   T8InstOp8<0x1D, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$src), "xdble", []>;
def LSHR :    T8InstOp8<0x35, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$cnt, OReg:$src1, OReg:$src2), "lshr", []>;
def LSHL :    T8InstOp8<0x36, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$cnt, OReg:$src1, OReg:$src2), "lshl", []>;
def LADD :    T8InstOp8<0x16, (outs OReg:$dst1), (ins OReg:$src1, OReg:$src2, OReg:$src3), "ladd", []>;
def LSUB :    T8InstOp8<0x38, (outs OReg:$dst1), (ins OReg:$src1, OReg:$src2, OReg:$src3), "lsub", []>;
def LSUM :    T8InstOp8<0x37, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$src1, OReg:$src2, OReg:$src3), "lsum", []>;
def LDIFF :   T8InstOp8<0x4F, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$src1, OReg:$src2, OReg:$src3), "ldiff", []>;
def LMUL :    T8InstOp8<0x31, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$src1, OReg:$src2, OReg:$src3), "lmul", []>;
def LDIV :    T8InstOp8<0x1A, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$src1, OReg:$src2, OReg:$src3), "ldiv", []>;
def NORM :    T8InstOp8<0x19, (outs OReg:$dst1, OReg:$dst2), (ins OReg:$src1, OReg:$src2, OReg:$src3), "norm", []>;

def SUM :     T8InstOp8<0x52, (outs OReg:$dst), (ins OReg:$src1, OReg:$src2), "sum", []>;

// Note: There is no binary not operation in the LLVM instruction set.
def NOT : T8InstOp8<0x32, (outs OReg:$dst), (ins OReg:$src), "not", []>;

// Fractional arithmetic
def FMUL :        T8InstOp8<0x72, (outs OReg:$dst),
    	 	  		  (ins OReg:$src1, OReg:$src2),
				   "fmul", []>;   // T800 and T414 only
def UNPACKSN :   T8InstOp8<0x63, (outs OReg:$frac, OReg:$exp, OReg:$type), (ins OReg:$fnum),
    	     	 		  "unpacksn", []>;   // T414 only
def POSTNORMSN : T8InstOp8<0x6C, (outs OReg:$guardo, OReg:$fraco, OReg:$shift),
    	       	 		 (ins OReg:$guardi, OReg:$fraci, OReg:$exp),
				  "postnormsn", []>; // T414 only
def ROUNDSN :    T8InstOp8<0x6D, (outs OReg:$fnum), (ins OReg:$guard, OReg:$frac, OReg:$exp),
    	    	 		  "roundsn", []>;    // T414 only
def LDINF :       T8InstOp8<0x71, (outs OReg:$dst), (ins), "ldinf", []>;     // T414 only
def CFLERR :      T8InstOp8<0x73, (outs), (ins OReg:$dst), "cflerr", []>;    // T414 only

// Process management
def STLF :    T8InstOp8<0x1C, (outs), (ins OReg:$fptr), "stlf", []>;
def STLB :    T8InstOp8<0x17, (outs), (ins OReg:$bptr), "stlb", []>;
def STHF :    T8InstOp8<0x18, (outs), (ins OReg:$fptr), "sthf", []>;
def STHB :    T8InstOp8<0x50, (outs), (ins OReg:$bptr), "sthb", []>;

def SAVEL :   T8InstOp8<0x3D, (outs), (ins OReg:$src), "savel", []>;
def SAVEH :   T8InstOp8<0x3E, (outs), (ins OReg:$src), "saveh", []>;

def LDPRI :   T8InstOp8<0x1E, (outs OReg:$dst), (ins), "ldpri", []>;

def STARTP :  T8InstOp4<0xD, (outs), (ins OReg:$ws, OReg:$iptr), "startp", []>;
def STOPP :   T8InstOp8<0x15, (outs), (ins), "stopp", []>;
def RUNP :    T8InstOp8<0x39, (outs), (ins OReg:$desc), "runp", []>;
def ENDP :    T8InstOp4<0x3, (outs), (ins OReg:$iptr), "endp", []>;

// ALT, TODO: Not fully specified, however instruction description is rather vague in documentation
def ALT :     T8InstOp8<0x43, (outs), (ins), "alt", []>;
def ALTWT :   T8InstOp8<0x44, (outs), (ins), "altwt", []>;
def ALTEND :  T8InstOp8<0x45, (outs), (ins), "altend", []>;
def TALT :    T8InstOp8<0x4E, (outs), (ins), "talt", []>;
def TALTWT :  T8InstOp8<0x51, (outs), (ins), "taltwt", []>;
def ENBS :    T8InstOp8<0x49, (outs), (ins), "enbs", []>;
def DISS :    T8InstOp8<0x30, (outs), (ins), "diss", []>;
def ENBC :    T8InstOp8<0x48, (outs), (ins), "enbc", []>;
def DISC :    T8InstOp8<0x2F, (outs), (ins), "disc", []>;
def ENBT :    T8InstOp8<0x47, (outs), (ins), "enbt", []>;
def DIST :    T8InstOp8<0x2E, (outs OReg:$dst1, OReg:$dst2), (ins), "dist", []>;
def CCNT1 :   T8InstOp8<0x4D, (outs), (ins OReg:$src1, OReg:$src2), "ccnt1", []>;

// Timers
def LDTIMER : T8InstOp8<0x22, (outs OReg:$dst), (ins), "ltimer", []>;
def TIN :     T8InstOp8<0x2B, (outs), (ins OReg:$src), "tin", []>;
def STTIMER : T8InstOp8<0x54, (outs), (ins OReg:$src), "sttimer", []>;

// 2D functions (T800 only)
def MOVE2DINIT :  T8InstOp8<0x5B, (outs), (ins OReg:$len, OReg:$dst_stride, OReg:$src_stride), "move2dinit", []>;
def MOVE2DALL :   T8InstOp8<0x5C, (outs), (ins OReg:$width, OReg:$dst_addr, OReg:$src_addr), "move2dall", []>;
def MOVE2DNONZERO : T8InstOp8<0x5D, (outs), (ins OReg:$width, OReg:$dst_addr, OReg:$src_addr), "move2dnonzero", []>;
def MOVE2DZERO :  T8InstOp8<0x5E, (outs), (ins OReg:$width, OReg:$dst_addr, OReg:$src_addr), "move2dzero", []>;

// Bit modifications and CRC (T800 only)
def CRCWORD :     T8InstOp8<0x74, (outs OReg:$crc), (ins OReg:$val, OReg:$acc, OReg:$gen), "crcword", []>;
def CRCBYTE :     T8InstOp8<0x75, (outs OReg:$crc), (ins OReg:$val, OReg:$acc, OReg:$gen), "crcbyte", []>;
def BITCNT :      T8InstOp8<0x76, (outs OReg:$dst), (ins OReg:$src1, OReg:$src2), "bitcnt", []>;
def BITREVWORD :  T8InstOp8<0x77, (outs OReg:$dst), (ins OReg:$src), "bitrevword", []>;
def BITREVNBITS : T8InstOp8<0x78, (outs OReg:$dst), (ins OReg:$src1, OReg:$src2), "bitrevnbits", []>;
def WSUBDB :     T8InstOp8<0x81, (outs OReg:$dst), (ins OReg:$src1, OReg:$src2), "wsubdb", []>;  // Form double word subscript

/*
Floating point instructions

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
def FPUR32TOR64 :  T8InstFPU4<0x7, "fpur32tor64">;
def FPUR64TOR32 :  T8InstFPU4<0x8, "fpur64tor32">;
def FPUEXPDEC32 :  T8InstFPU4<0x9, "fpuexpdec32">;
def FPUEXPINC32 :  T8InstFPU4<0xA, "fpuexpinc32">;
def FPUABS :  	   T8InstFPU4<0xB, "fpuabs">;
def FPUNOROUND :   T8InstFPU4<0xD, "fpunoround">;
def FPUCHKI32 :    T8InstFPU4<0xE, "fpuchki32">;
def FPUCHKI64 :    T8InstFPU4<0xF, "fpuchki64">;

def FPUDIVBY2 :    T8InstFPU8<0x11, "fpudivby2">;
def FPUMULBY2 :    T8InstFPU8<0x12, "fpumulby2">;
def FPURN :        T8InstFPU8<0x22, "fpurn">;
def FPUSETERR :    T8InstFPU8<0x23, "fpuseterr">;
def FPUCLRERR :    T8InstFPU8<0x9C, "fpuclrerr">;
*/

def XWORD  : T8InstOp8<0x3A,
		(outs OReg:$dst),
       	 	(ins OReg:$src1, OReg:$src2),
		"xword",
		[]>;

def CWORD :  T8InstOp8<0x56,
		(outs),
		(ins OReg:$src1, OReg:$src2),
		"cword",
		[]>;

