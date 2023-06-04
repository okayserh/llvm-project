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

// Integer (pseudo) registers
def R0 : T8xxReg< 0, "R0">, DwarfRegNum<[0]>;
def R1 : T8xxReg< 1, "R1">, DwarfRegNum<[1]>;
def R2 : T8xxReg< 2, "R2">, DwarfRegNum<[2]>;
def R3 : T8xxReg< 3, "R3">, DwarfRegNum<[3]>;
def R4 : T8xxReg< 4, "R4">, DwarfRegNum<[4]>;
def R5 : T8xxReg< 5, "R5">, DwarfRegNum<[5]>;
def R6 : T8xxReg< 6, "R6">, DwarfRegNum<[6]>;
def R7 : T8xxReg< 7, "R7">, DwarfRegNum<[7]>;
def R8 : T8xxReg< 8, "R8">, DwarfRegNum<[8]>;
def R9 : T8xxReg< 9, "R9">, DwarfRegNum<[9]>;
def R10 : T8xxReg<10, "R10">, DwarfRegNum<[10]>;
def R11 : T8xxReg<11, "R11">, DwarfRegNum<[11]>;
def R12 : T8xxReg<12, "R12">, DwarfRegNum<[12]>;
def R13 : T8xxReg<13, "R13">, DwarfRegNum<[13]>;
def R14 : T8xxReg<14, "R14">, DwarfRegNum<[14]>;
def R15 : T8xxReg<15, "R15">, DwarfRegNum<[15]>;

def AREG : T8xxReg<0, "AREG">, DwarfRegNum<[16]>;
def BREG : T8xxReg<1, "BREG">, DwarfRegNum<[17]>;
def CREG : T8xxReg<2, "CREG">, DwarfRegNum<[18]>;

def OREG : T8xxReg<3, "OREG">, DwarfRegNum<[19]>;

def IPTR : T8xxReg<4, "IPTR">, DwarfRegNum<[20]>;
def WPTR : T8xxReg<5, "WPTR">, DwarfRegNum<[21]>;

def CPSR  : T8xxReg<16, "cpsr">;

// The processor stack register. For informational
// purpose to flag instructions that modify the processor stack.
def PSTACK  : T8xxReg<17, "PSTACK">;

// Register classes.
//
// FIXME: the register order should be defined in terms of the preferred
// allocation order...
//
// This register class should not be used to hold i64 values, use the I64Regs
// register class for that. The i64 type is included here to allow i64 patterns
// using the integer instructions.

def IntRegs : RegisterClass<"T8xx", [i32], 32,
                            (add (sequence "R%u", 1, 15))>;

// The frame pointer or workspace pointer is special
// in that it is used in special adressing modes
def WptrRegs : RegisterClass<"T8xx", [i32], 32,
                            (add WPTR)>;

def OperandReg : RegisterClass<"T8xx", [i32], 32,
                                (add AREG)>;

