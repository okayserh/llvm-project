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
The T8xx floating unit uses the same assembler instruction for
single and double precision operations. However, for binary
operations both parameters must have the same length.
*/

/* https://discourse.llvm.org/t/how-to-resolve-decoding-conflict/32818 */

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


multiclass T8BinaryCastOp<bits<8> opval, string opstr,
	   		SDNode opnode>
{
	def NAME#DB :      T8BinaryDBOp8<opval, opstr, opnode>;
	def NAME#SN :      T8BinarySNOp8<opval, opstr, opnode>;
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
			    (ins MemSrc:$addr),
			    "fpi32tor32",
			    [(set f32:$dst, (sint_to_fp (i32 (load ADDRr:$addr))))]>;

def : Pat<(sint_to_fp (i32 (load ADDRri:$addr))),
          (FPI32TOR32 (LDLP ADDRri:$addr) )>;

def FPI32TOR64 :  T8InstOp8<0x98,
		            (outs DFPReg:$dst),
			    (ins MemSrc:$addr),
			    "fpi32tor64",
			    [(set f64:$dst, (sint_to_fp (i32 (load ADDRr:$addr))))]>;

def : Pat<(sint_to_fp (i32 (load ADDRri:$addr))),
          (FPI32TOR64 (LDLP ADDRri:$addr) )>;


// Conversion from uint to f32 or f64
def FPB32TOR64 :  T8InstOp8<0x9A,
                            (outs DFPReg:$dst),
			    (ins MemSrc:$addr),
			    "fpb32tor64",
			    [(set f64:$dst, (uint_to_fp (i32 (load ADDRr:$addr))))]>;

def : Pat<(uint_to_fp (i32 (load ADDRri:$addr))),
          (FPB32TOR64 (LDLP ADDRri:$addr) )>;

// Conversions to f32 are not natively supported, add a fp conversion to f32
def : Pat<(uint_to_fp (i32 (load ADDRr:$addr))),
          (FPUR64TOR32 (FPB32TOR64 (ADDRr:$addr) ) )>;

def : Pat<(uint_to_fp (i32 (load ADDRri:$addr))),
          (FPUR64TOR32 (FPB32TOR64 (LDLP ADDRri:$addr) ) )>;


// Round FAreg to integer according to current rounding mode
// Result remains in floating point register!
// "This rounds FAReg, wrt Round.Mode, to a floating point number of the
// same format with an integer value.
def FPINTSN :       T8InstOp8<0xA1,
			   (outs FPReg:$dst),
			   (ins FPReg:$src, FPRMReg:$rm),
			   "fpint", []>
{
 let DecoderNamespace = "FPSN";
}

def FPINTDB :       T8InstOp8<0xA1,
			   (outs DFPReg:$dst),
			   (ins DFPReg:$src, FPRMReg:$rm),
			   "fpint", []>;

def FPSTNLI32 :   T8InstOp8<0x9E,
			(outs),
			(ins FPReg:$val, MemSrc:$addr),
			"fpstnli32",
			[(store (i32 (fp_to_sint f32:$val)), ADDRr:$addr)]>;

// Pseudo instructions to provide code for the conversion from floating
// point to integer. These are multi instruction sequences on the T8xx.
def FPR32TOI32 :   T8Inst<(outs OReg:$dst),
			(ins FPReg:$src),
			"fpr32toi32",
			[(set i32:$dst, (fp_to_sint f32:$src))]>
{
    let isCodeGenOnly = 1;
    let isPseudo = 1;
}

// The Inmos C compiler generates the same code to convert a
// floating point number into an unsigned value.
// Note: Was tested with an example and seems to work.
def : Pat<(fp_to_uint (f32 FPReg:$src)),
          (FPR32TOI32 $src)>;

def FPR64TOI32 :   T8Inst<(outs OReg:$dst),
			(ins DFPReg:$src),
			"fpr64toi32",
			[(set i32:$dst, (fp_to_sint f64:$src))]>
{
    let isCodeGenOnly = 1;
    let isPseudo = 1;
}

def : Pat<(fp_to_uint (f64 DFPReg:$src)),
          (FPR64TOI32 $src)>;


// "This rounds FAReg, wrt Round.Mode, to a floating point number of the
// same format with an integer value ---and sets error if this lies outside
// the INT32 range.---
def FPRTOI32 :    T8InstOp8<0x9D, (outs), (ins),  "fprtoi32", []>;


// Floating point comparisons
// Since these comparisons don't check for NaN, they declare the
// "Don't care operations".
/*
  SETEQ,     Real instruction
  SETGT,     Real instruction
  SETGE,     Pattern (A >= B  ->   ! B > A)
  SETLT,     Pattern (A < B   ->     B > A)
  SETLE,     Pattern (A <= B  ->   ! A > B)  
  SETNE,     Pattern (A != B  ->   ! A = B)
*/

def FPEQSN :      T8InstOp8<0x95,
    	 	  (outs OReg:$cc),
		  (ins FPReg:$lhs, FPReg:$rhs),
		  "fpeq",
		  [(set i32:$cc,
		  (i32 (setcc f32:$lhs, f32:$rhs, SETEQ)))]>
{
	 let DecoderNamespace = "FPSN";
}

def FPEQDB :      T8InstOp8<0x95,
    	 	  (outs OReg:$cc),
		  (ins DFPReg:$lhs, DFPReg:$rhs),
		  "fpeq",
		  [(set i32:$cc,
		  (i32 (setcc f64:$lhs, f64:$rhs, SETEQ)))]>;

def FPGTSN :      T8InstOp8<0x94,
    	 	  (outs OReg:$cc),
		  (ins FPReg:$lhs, FPReg:$rhs),
		  "fpgt",
		  [(set i32:$cc,
		  (i32 (setcc f32:$lhs, f32:$rhs, SETGT)))]>
{
	 let DecoderNamespace = "FPSN";
}

def FPGTDB :      T8InstOp8<0x94,
    	 	  (outs OReg:$cc),
		  (ins DFPReg:$lhs, DFPReg:$rhs),
		  "fpgt",
		  [(set i32:$cc,
		  (i32 (setcc f64:$lhs, f64:$rhs, SETGT)))]>;

// Other "don't care" comparisons will be created by patterns
def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETNE),
           (EQC (FPEQSN $lhs, $rhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETNE),
           (EQC (FPEQDB $lhs, $rhs), 0)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETGE),
           (EQC (FPGTSN $rhs, $lhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETGE),
           (EQC (FPGTDB $rhs, $lhs), 0)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETLT),
           (FPGTSN $rhs, $lhs)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETLT),
           (FPGTDB $rhs, $lhs)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETLE),
           (EQC (FPGTSN $lhs, $rhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETLE),
           (EQC (FPGTDB $lhs, $rhs), 0)>;


// Pseudo instructions for comparisons
class T8PseudoCompareSN<CondCode condnode> :
      T8Inst<(outs OReg:$cc),
       	     (ins FPReg:$lhs, FPReg:$rhs),
	      "",
	     [(set i32:$cc,
	      (i32 (setcc f32:$lhs, f32:$rhs, condnode)))]>
{
    let isCodeGenOnly = 1;
    let isPseudo = 1;
}

class T8PseudoCompareDB<CondCode condnode> :
      T8Inst<(outs OReg:$cc),
       	     (ins DFPReg:$lhs, DFPReg:$rhs),
	      "",
	     [(set i32:$cc,
	      (i32 (setcc f64:$lhs, f64:$rhs, condnode)))]>
{
    let isCodeGenOnly = 1;
    let isPseudo = 1;
}

multiclass T8PseudoCmpOp<CondCode condnode>
{
	def NAME#DB :      T8PseudoCompareDB<condnode>;
	def NAME#SN :      T8PseudoCompareSN<condnode>;
}

defm FPOGT : T8PseudoCmpOp<SETOGT>;
defm FPOLE : T8PseudoCmpOp<SETOLE>;
defm FPOEQ : T8PseudoCmpOp<SETOEQ>;
defm FPONE : T8PseudoCmpOp<SETONE>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETOLT),
           (FPOGTSN $rhs, $lhs)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETOLT),
           (FPOGTDB $rhs, $lhs)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETOGE),
           (FPOLESN $rhs, $lhs)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETOGE),
           (FPOLEDB $rhs, $lhs)>;


// Pattern fors the unordered variants. These are created from
// the ordered variants by Morgans rule.
def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETUEQ),
           (EQC (FPONESN $lhs, $rhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETUEQ),
           (EQC (FPONEDB $lhs, $rhs), 0)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETUGT),
           (EQC (FPOGTSN $rhs, $lhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETUGT),
           (EQC (FPOGTDB $rhs, $lhs), 0)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETUGE),
           (EQC (FPOGTSN $lhs, $rhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETUGE),
           (EQC (FPOGTDB $lhs, $rhs), 0)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETULT),
           (EQC (FPOGTSN $lhs, $rhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETULT),
           (EQC (FPOGTDB $lhs, $rhs), 0)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETULE),
           (EQC (FPOGTSN $rhs, $lhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETULE),
           (EQC (FPOGTDB $rhs, $lhs), 0)>;

def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETUNE),
           (EQC (FPOEQSN $lhs, $rhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETUNE),
           (EQC (FPOEQDB $lhs, $rhs), 0)>;


// Tests for Nan, not finite and orderability
// Note: Needs two FPReg, but does not "consume" them from FP-stack
// To have this working with the stackifiction, assume the registers are consumed.
def FPORDEREDSN :   T8InstOp8<0x92,
	    (outs OReg:$cc),
	    (ins FPReg:$lhs, FPReg:$rhs),
	    "fpordered",
	    [(set i32:$cc,
	      (i32 (setcc f32:$lhs, f32:$rhs, SETO)))]>
{
	 let DecoderNamespace = "FPSN";
}

def FPORDEREDDB :   T8InstOp8<0x92,
	    (outs OReg:$cc),
	    (ins DFPReg:$lhs, DFPReg:$rhs),
	    "fpordered",
	    [(set i32:$cc,
	      (i32 (setcc f64:$lhs, f64:$rhs, SETO)))]>;

// Check for unordered is just the inverse
def : Pat<(setcc (f32 FPReg:$lhs), (f32 FPReg:$rhs), SETUO),
           (EQC (FPORDEREDSN $lhs, $rhs), 0)>;
def : Pat<(setcc (f64 DFPReg:$lhs), (f64 DFPReg:$rhs), SETUO),
           (EQC (FPORDEREDDB $lhs, $rhs), 0)>;


def FPNAN :       T8InstOp8<0x91,
			    (outs OReg:$nan, FPReg:$tst_out),
			    (ins FPReg:$tst_in),  // Needs one FPReg, but does not "consume" it from FP-stack
			    "fpnan", []>;

def FPNOTFINITE : T8InstOp8<0x93,
			    (outs OReg:$inf, FPReg:$tst_out),
			    (ins FPReg:$tst_in),  // Needs one FPReg, but does not "consume" it from FP-stack
			    "fpnotfinite", []>;

// Instructions to set the rounding mode. They modify the rounding mode register
def FPURP :  	   T8InstFPU4<0x4, (outs FPRMReg:$rm), (ins), "fpurp", []>;
def FPURM :  	   T8InstFPU4<0x5, (outs FPRMReg:$rm), (ins), "fpurm", []>;
def FPURZ :  	   T8InstFPU4<0x6, (outs FPRMReg:$rm), (ins), "fpurz", []>;
def FPURN :        T8InstFPU8<0x22, (outs FPRMReg:$rm), (ins), "fpurn", []>;

// Indexed Load/Store
def FPLDNLDBI :  T8InstOp8<0x82, (outs), (ins),  "fpldnldbi", []>;
def FPLDNLSNI :  T8InstOp8<0x86, (outs), (ins),  "fpldnlsni", []>;

def FPTESTERR :   T8InstOp8<0x9C, (outs), (ins),  "fptesterr", []>;
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

def FPUEXPDEC32 :  T8InstFPU4<0x9, (outs), (ins), "fpuexpdec32", []>;
def FPUEXPINC32 :  T8InstFPU4<0xA, (outs), (ins), "fpuexpinc32", []>;
def FPUABS :  	   T8InstFPU4<0xB, (outs), (ins), "fpuabs", []>;
def FPUNOROUND :   T8InstFPU4<0xD, (outs), (ins), "fpunoround", []>;

def FPUCHKI32 :    T8InstFPU4<0xE, (outs), (ins), "fpuchki32", []>;
def FPUCHKI64 :    T8InstFPU4<0xF, (outs), (ins), "fpuchki64", []>;

def FPUDIVBY2 :    T8InstFPU8<0x11, (outs), (ins), "fpudivby2", []>;
def FPUMULBY2 :    T8InstFPU8<0x12, (outs), (ins), "fpumulby2", []>;

def FPCHKERR :     T8InstOp8<0x83,  (outs), (ins), "fpchkerr", []>;
def FPUSETERR :    T8InstFPU8<0x23, (outs), (ins), "fpuseterr", []>;
def FPUCLRERR :    T8InstFPU8<0x9C, (outs), (ins), "fpuclrerr", []>;

