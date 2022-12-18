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
  let Namespace = "T8";
}

//class T8xxCtrlReg<bits<16> Enc, string n>: Register<n> {
//  let HWEncoding = Enc;
//  let Namespace = "T8";
//}

// Registers are identified with 4-bit ID numbers.
// Ri - 32-bit integer registers
class Ri<bits<16> Enc, string n> : T8xxReg<Enc, n>;

// Integer registers
def R0 : Ri< 0, "R0">, DwarfRegNum<[0]>;
def R1 : Ri< 1, "R1">, DwarfRegNum<[1]>;
def R2 : Ri< 2, "R2">, DwarfRegNum<[2]>;
def R3 : Ri< 3, "R3">, DwarfRegNum<[3]>;
def R4 : Ri< 4, "R4">, DwarfRegNum<[4]>;
def R5 : Ri< 5, "R5">, DwarfRegNum<[5]>;
def R6 : Ri< 6, "R6">, DwarfRegNum<[6]>;
def R7 : Ri< 7, "R7">, DwarfRegNum<[7]>;
def R8 : Ri< 8, "R8">, DwarfRegNum<[8]>;
def R9 : Ri< 9, "R9">, DwarfRegNum<[9]>;
def R10 : Ri<10, "R10">, DwarfRegNum<[10]>;
def R11 : Ri<11, "R11">, DwarfRegNum<[11]>;
def R12 : Ri<12, "R12">, DwarfRegNum<[12]>;
def R13 : Ri<13, "R13">, DwarfRegNum<[13]>;
def R14 : Ri<14, "R14">, DwarfRegNum<[14]>;
def R15 : Ri<15, "R15">, DwarfRegNum<[15]>;


// Register classes.
//
// FIXME: the register order should be defined in terms of the preferred
// allocation order...
//
// This register class should not be used to hold i64 values, use the I64Regs
// register class for that. The i64 type is included here to allow i64 patterns
// using the integer instructions.
def IntRegs : RegisterClass<"T8", [i32], 32,
                            (add (sequence "R%u", 0, 15))>;
