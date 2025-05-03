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

/* https://discourse.llvm.org/t/how-to-resolve-decoding-conflict/32818 */

// The Transputer floating point unit maintains a flag with the floating
// point register stack, which indicates whether the content is single
// or double precision. Thus, fmul, fdiv, fadd, fsub can cope with
// mixed precision and include the respective rounding or extension
// The LLVM standard FP binary op assumes that both operands and
// the result are the same. Therefore, the two type patterns are
// defined to enable a description of the aforementioned transputer
// instructions.

// Result and second operand are F64, first operand is F32
def SDTFP_DS_BinOp : SDTypeProfile<1, 2, [      // fadd, fmul, etc.
  SDTCisVT<0, f32>, SDTCisVT<1, f64>, SDTCisVT<2, f32>
]>;

// Result and second operand are F32, first operand is F64
def SDTFP_SD_BinOp : SDTypeProfile<1, 2, [      // fadd, fmul, etc.
  SDTCisVT<0, f64>, SDTCisVT<1, f32>, SDTCisVT<2, f64>
]>;


class T8BinaryDBOp8<bits<8> opval, string opstr, SDNode opnode> :
      T8InstOp8<opval,
                (outs DFPReg:$dst),
       	 	(ins DFPReg:$src1, DFPReg:$src2),
		opstr,
		[(set f64:$dst, (opnode f64:$src1, f64:$src2))]>
{
}

class T8BinarySNOp8<bits<8> opval, string opstr, SDNode opnode> :
      T8InstOp8<opval,
                (outs FPReg:$dst),
       	 	(ins FPReg:$src1, FPReg:$src2),
		opstr,
		[(set f32:$dst, (opnode f32:$src1, f32:$src2))]>
{
	 	 let DecoderNamespace = "FPSN";
}

class T8BinaryDSOp8<bits<8> opval, string opstr, SDNode opnode> :
      T8InstOp8<opval,
                (outs FPReg:$dst),
       	 	(ins DFPReg:$src1, FPReg:$src2),
		opstr,
		[(set f32:$dst, (opnode f64:$src1, f32:$src2))]>
{
	 	 let DecoderNamespace = "FPDS";
}

class T8BinarySDOp8<bits<8> opval, string opstr, SDNode opnode> :
      T8InstOp8<opval,
                (outs DFPReg:$dst),
       	 	(ins FPReg:$src1, DFPReg:$src2),
		opstr,
		[(set f64:$dst, (opnode f32:$src1, f64:$src2))]>
{
	 	 let DecoderNamespace = "FPSD";
}

// Arithmethic
//def FPADD :      T8BinarySNOp8<0x87, "fpadd", fadd>;
//def FPSUB :      T8BinarySNOp8<0x89, "fpsub", fsub>;
// Note: The default fmul does not allow for mixed inputs!
//def SDTFPBinOp : SDTypeProfile<1, 2, [      // fadd, fmul, etc.
//  SDTCisSameAs<0, 1>, SDTCisSameAs<0, 2>, SDTCisFP<0>
//]>;
// 


multiclass T8BinaryCastOp<bits<8> opval, string opstr,
	   		SDNode opnode>
{
	def "T8xxDS_"#NAME : SDNode<"T8xxISD::DS_"#NAME, SDTFP_DS_BinOp, []>;
	def "T8xxSD_"#NAME : SDNode<"T8xxISD::SD_"#NAME, SDTFP_SD_BinOp, []>;

	def NAME#DB :      T8BinaryDBOp8<opval, opstr, opnode>;
	def NAME#SN :      T8BinarySNOp8<opval, opstr, opnode>;
	def NAME#DS :      T8BinaryDSOp8<opval, opstr, !cast<SDNode>("T8xxDS_"#NAME)>;
	def NAME#SD :      T8BinarySDOp8<opval, opstr, !cast<SDNode>("T8xxSD_"#NAME)>;
}

// Note: The respective T8xxISD::DS_... need to be defined int T8xxISelLowering.h!
defm FADD : T8BinaryCastOp<0x87, "fpadd", fadd>;
defm FSUB : T8BinaryCastOp<0x89, "fpsub", fsub>;
defm FMUL : T8BinaryCastOp<0x8B, "fpmul", fmul>;
defm FDIV : T8BinaryCastOp<0x8C, "fpdiv", fdiv>;



// Store f32 to global address
def FPSTNLSN : T8InstOp8<0x88,
		(outs),
       	 	(ins FPReg:$val, MemSrc:$addr),
		"fpstnlsn",
		[(store f32:$val, ADDRr:$addr)]> {
		}

// Pattern for FP stores to WPtr stack
def : Pat<(store f32:$val, ADDRri:$addr),
          (FPSTNLSN f32:$val, (LDLP ADDRri:$addr) )>;

// Store f64 to global address
def FPSTNLDB : T8InstOp8<0x84,
		(outs),
       	 	(ins DFPReg:$val, MemSrc:$addr),
		"fpstnldb",
		[(store f64:$val, ADDRr:$addr)]> {
		}

def : Pat<(store f64:$val, ADDRri:$addr),
          (FPSTNLDB f64:$val, (LDLP ADDRri:$addr) )>;


// Load f32 from global address
def FPLDNLSN : T8InstOp8<0x8E,
		(outs FPReg:$val),
       	 	(ins MemSrc:$addr),
		"fpldnlsn",
		[(set f32:$val, (load ADDRr:$addr))]>
{
}

// Pattern for FP loads from WPtr stack
def : Pat<(f32 (load ADDRri:$addr)),
          (FPLDNLSN (LDLP ADDRri:$addr) )>;



// Load f32 from global address
def FPLDNLDB : T8InstOp8<0x8A,
		(outs DFPReg:$val),
       	 	(ins MemSrc:$addr),
		"fpldnldb",
		[(set f64:$val, (load ADDRr:$addr))]>
{
}

// Pattern for FP loads from WPtr stack
def : Pat<(f64 (load ADDRri:$addr)),
          (FPLDNLDB (LDLP ADDRri:$addr) )>;


// Patterns for load / mul and load / add operations. These do not
// save any processor cycles but make the code more compact (2 bytes saved).
def FPLDNLADDSN : T8InstOp8<0xAA,
			   (outs FPReg:$dst),
			   (ins FPReg:$src, MemSrc:$addr),
			   "fpldnladdsn",
			   [(set f32:$dst, (fadd f32:$src, (load ADDRr:$addr)))]>;

def FPLDNLADDDB : T8InstOp8<0xA6,
			   (outs DFPReg:$dst),
			   (ins DFPReg:$src, MemSrc:$addr),
			   "fpldnladddb",
			   [(set f64:$dst, (fadd f64:$src, (load ADDRr:$addr)))]>;

def FPLDNLMULSN : T8InstOp8<0xAC,
			   (outs FPReg:$dst),
			   (ins FPReg:$src, MemSrc:$addr),
			   "fpldnlmulsn",
			   [(set f32:$dst, (fmul f32:$src, (load ADDRr:$addr)))]>;

def FPLDNLMULDB : T8InstOp8<0xA8,
			   (outs DFPReg:$dst),
			   (ins DFPReg:$src, MemSrc:$addr),
			   "fpldnlmuldb",
			   [(set f64:$dst, (fadd f64:$src, (load ADDRr:$addr)))]>;

def : Pat<(fmul FPReg:$op1, (load ADDRri:$addr)),
          (FPLDNLMULSN FPReg:$op1, (LDLP ADDRri:$addr) )>;
def : Pat<(fmul DFPReg:$op1, (load ADDRri:$addr)),
          (FPLDNLMULDB DFPReg:$op1, (LDLP ADDRri:$addr) )>;

def : Pat<(fadd FPReg:$op1, (load ADDRri:$addr)),
          (FPLDNLADDSN FPReg:$op1, (LDLP ADDRri:$addr) )>;
def : Pat<(fadd DFPReg:$op1, (load ADDRri:$addr)),
          (FPLDNLADDDB DFPReg:$op1, (LDLP ADDRri:$addr) )>;


// Conversions between f32 and f64
def FPUR64TOR32  : T8InstFPU4<0x8, (outs FPReg:$dst), (ins DFPReg:$src),
                         "fpur64tor32",
               	         [(set f32:$dst, (fpround f64:$src))]> {
}

def FPUR32TOR64  : T8InstFPU4<0x7, (outs DFPReg:$dst), (ins FPReg:$src),
                     	 "fpur32tor64",
               	         [(set f64:$dst, (fpextend f32:$src))]> {
}


// Conversion from int to f32 or f64
def FPI32TOR32 :  T8InstOp8<0x96,
			    (outs FPReg:$dst),
			    (ins OReg:$src),
			    "fpi32tor32", []>;

def FPI32TOR64 :  T8InstOp8<0x98,
		            (outs DFPReg:$dst),
			    (ins OReg:$src),
			    "fpi32tor64", []>;

// Round FAreg to integer according to current rounding mode
// Result remains in floating point register!
def FPINTSN :       T8InstOp8<0xA1,
			   (outs FPReg:$dst),
			   (ins FPReg:$src),
			   "fpint", []>
{
 let DecoderNamespace = "FPSN";
}

def FPINTDB :       T8InstOp8<0xA1,
			   (outs DFPReg:$dst),
			   (ins DFPReg:$src),
			   "fpint", []>;


// Comparison (TODO: These instructions also have the "casting property" that
// the length of FAreg is set to the length of FBreg)
def FPEQSN :      T8InstOp8<0x95,
    	 	  (outs OReg:$cc),
		  (ins FPReg:$lhs, FPReg:$rhs),
		  "fpeq",
		  [(set i32:$cc,
		  (i32 (setcc f32:$lhs, f32:$rhs, SETOEQ)))]>
{
	 let DecoderNamespace = "FPSN";
}


def FPEQDB :      T8InstOp8<0x95,
    	 	  (outs OReg:$cc),
		  (ins DFPReg:$lhs, DFPReg:$rhs),
		  "fpeq",
		  [(set i32:$cc,
		  (i32 (setcc f64:$lhs, f64:$rhs, SETOEQ)))]>
{
	 let DecoderNamespace = "FPDB";
}


def FPGTSN :      T8InstOp8<0x94,
    	 	  (outs OReg:$cc),
		  (ins FPReg:$lhs, FPReg:$rhs),
		  "fpgt",
		  [(set i32:$cc,
		  (i32 (setcc f32:$lhs, f32:$rhs, SETOGT)))]>
{
	 let DecoderNamespace = "FPSN";
}

def FPGTDB :      T8InstOp8<0x94,
    	 	  (outs OReg:$cc),
		  (ins DFPReg:$lhs, DFPReg:$rhs),
		  "fpgt",
		  [(set i32:$cc,
		  (i32 (setcc f64:$lhs, f64:$rhs, SETOGT)))]>
{
	 let DecoderNamespace = "FPDB";
}



/* Other comparisons will be created by patterns */
def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETONE),
           (EQC (FPEQSN $rhs, $lhs), 0)>;

def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETONE),
           (EQC (FPEQDB $rhs, $lhs), 0)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETOGE),
           (EQC (FPGTSN $rhs, $lhs), 0)>;

def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETOGE),
           (EQC (FPGTDB $rhs, $lhs), 0)>;


// Indexed Load/Store
def FPLDNLDBI :  T8InstOp8<0x82, (outs), (ins),  "fpldnldbi", []>;
def FPLDNLSNI :  T8InstOp8<0x86, (outs), (ins),  "fpldnlsni", []>;

// Tests for Nan, not finite and orderability
def FPNAN :       T8InstOp8<0x91,
			    (outs OReg:$nan),
			    (ins),  // Needs one FPReg, but does not "consume" it from FP-stack
			    "fpnan", []>;

def FPORDERED :   T8InstOp8<0x92,
			    (outs OReg:$ord),
			    (ins),  // Needs two FPReg, but does not "consume" them from FP-stack
			    "fpordered", []>;

def FPNOTFINITE : T8InstOp8<0x93,
			    (outs OReg:$inf),
			    (ins),  // Needs one FPReg, but does not "consume" it from FP-stack
			    "fpnotfinite", []>;

def FPB32TOR64 :  T8InstOp8<0x9A, (outs), (ins),  "fpb32tor64", []>;
def FPTESTERR :   T8InstOp8<0x9C, (outs), (ins),  "fptesterr", []>;
def FPRTOI32 :    T8InstOp8<0x9D, (outs), (ins),  "fprtoi32", []>;
def FPSTNLI32 :   T8InstOp8<0x9E, (outs), (ins),  "fpstnli32", []>;
def FPLDZEROSN :  T8InstOp8<0x9F, (outs), (ins),  "fpldzerosn", []>;

def FPLDZERODB :  T8InstOp8<0xA0, (outs), (ins),  "fpldzerodb", []>;
def FPDUP :       T8InstOp8<0xA3, (outs), (ins),  "fpdup", []>;
def FPREV :       T8InstOp8<0xA4, (outs), (ins),  "fprev", []>;
def FPENTRY :     T8InstOp8<0xAB, (outs), (ins),  "fpentry", []>;

def FPREMFIRST :  T8InstOp8<0x8F, (outs), (ins), "fpremfirst", []>;
def FPREMSTEP :   T8InstOp8<0x90, (outs), (ins), "fpremstep", []>;


/* These are in fact combinations of ldc <opcode> and fpentry */
def FPUSQRTFIRST : T8InstFPU4<0x1, (outs), (ins), "fpusqrtfirst", []>;
def FPUSQRTSTEP :  T8InstFPU4<0x2, (outs), (ins), "fpusqrtstep", []>;
def FPUSQRTLAST :  T8InstFPU4<0x3, (outs), (ins), "fpusqrtlast", []>;
def FPURP :  	   T8InstFPU4<0x4, (outs), (ins), "fpurp", []>;
def FPURM :  	   T8InstFPU4<0x5, (outs), (ins), "fpurm", []>;
def FPURZ :  	   T8InstFPU4<0x6, (outs), (ins), "fpurz", []>;
def FPUEXPDEC32 :  T8InstFPU4<0x9, (outs), (ins), "fpuexpdec32", []>;
def FPUEXPINC32 :  T8InstFPU4<0xA, (outs), (ins), "fpuexpinc32", []>;
def FPUABS :  	   T8InstFPU4<0xB, (outs), (ins), "fpuabs", []>;
def FPUNOROUND :   T8InstFPU4<0xD, (outs), (ins), "fpunoround", []>;

def FPUCHKI32 :    T8InstFPU4<0xE, (outs), (ins), "fpuchki32", []>;
def FPUCHKI64 :    T8InstFPU4<0xF, (outs), (ins), "fpuchki64", []>;

def FPUDIVBY2 :    T8InstFPU8<0x11, (outs), (ins), "fpudivby2", []>;
def FPUMULBY2 :    T8InstFPU8<0x12, (outs), (ins), "fpumulby2", []>;
def FPURN :        T8InstFPU8<0x22, (outs), (ins), "fpurn", []>;

def FPCHKERR :     T8InstOp8<0x83,  (outs), (ins), "fpchkerr", []>;
def FPUSETERR :    T8InstFPU8<0x23, (outs), (ins), "fpuseterr", []>;
def FPUCLRERR :    T8InstFPU8<0x9C, (outs), (ins), "fpuclrerr", []>;

