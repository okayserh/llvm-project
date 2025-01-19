//===-- T8xxRegisterInfo.td - T8xx Register defs ---------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the T8xx register file
//===----------------------------------------------------------------------===//

class T8xxReg<bits<16> Enc, string n> : Register<n> {
  let HWEncoding = Enc;
  let Namespace = "T8xx";
}

// Regular representation of operand registers
def AREG : T8xxReg<0, "AREG">, DwarfRegNum<[16]>;
def BREG : T8xxReg<1, "BREG">, DwarfRegNum<[17]>;
def CREG : T8xxReg<2, "CREG">, DwarfRegNum<[18]>;

def OREG : T8xxReg<3, "OREG">, DwarfRegNum<[19]>;

def IPTR : T8xxReg<4, "IPTR">, DwarfRegNum<[20]>;
def WPTR : T8xxReg<5, "WPTR">, DwarfRegNum<[21]>;

// Floating point stack
def FAREG : T8xxReg<0, "FAREG">, DwarfRegNum<[22]>;
def FBREG : T8xxReg<1, "FBREG">, DwarfRegNum<[23]>;
def FCREG : T8xxReg<2, "FCREG">, DwarfRegNum<[24]>;

// Long registers (combines two stack registers)
def ABREG : T8xxReg<6, "ABREG">, DwarfRegNum<[25]>;
def BCREG : T8xxReg<7, "BCREG">, DwarfRegNum<[26]>;


/*
// Aliases of the F* registers to hold 64-bit fp values (doubles)
class T8xxDFReg<bits<16> Enc, string n, list<Register> subregs> : T8xxReg<Enc, n> {
      let SubRegs = subregs;
      let CoveredBySubRegs = 1;
}

def DFAREG: T8xxDFReg<70, "DFAREG", [FAREG]>, DwarfRegNum<[70]>;
def DFBREG: T8xxDFReg<71, "DFBREG", [FBREG]>, DwarfRegNum<[71]>;
def DFCREG: T8xxDFReg<72, "DFCREG", [FCREG]>, DwarfRegNum<[72]>;
*/

// Register classes.
//
// FIXME: the register order should be defined in terms of the preferred
// allocation order...
//

// The frame pointer or workspace pointer is special
// in that it is used in special adressing modes
def WptrRegs : RegisterClass<"T8xx", [i32], 32,
                            (add WPTR)>;

def OReg : RegisterClass<"T8xx", [i32], 32,
     	                  (add AREG, BREG, CREG)>;

def LReg : RegisterClass<"T8xx", [i64], 32,
     	                  (add ABREG, BCREG)>;

def FPReg:  RegisterClass<"T8xx", [f32], 32,
     	                  (add FAREG, FBREG, FCREG)>;

def DFPReg:  RegisterClass<"T8xx", [f64], 32,
     	                  (add FPReg)>;


