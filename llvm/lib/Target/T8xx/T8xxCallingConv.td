//===-- T8xxCallingConv.td - Calling Conventions T8xx ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the T8xx architectures.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Transputer, 32-bit.
//===----------------------------------------------------------------------===//

def CC_T8xx32 : CallingConv<[
  // Call arguments are assigned to the stack in 4-byte aligned units.
  // Note: CCAsignToStack <int size, int align>
  CCIfType<[i32], CCAssignToStack<4, 4>>,
  CCIfType<[i64], CCAssignToStack<8, 4>>,
  CCIfType<[f32], CCAssignToStack<4, 4>>,
  CCIfType<[f64], CCAssignToStack<8, 4>>
]>;

def RetCC_T8xx32 : CallingConv<[
  // i32 and smaller are returned in AREG  
  CCIfType<[i32], CCAssignToReg<[AREG]>>,

  // i64 is returned on stack
  CCIfType<[i64], CCAssignToStack<8, 4>>,

  // f32 is returned in FAREG
  CCIfType<[f32], CCAssignToReg<[FAREG]>>,

  // f64 is returned in FAREG
  CCIfType<[f64], CCAssignToReg<[FAREG]>>,
]>;

// TODO: Probably needs to span all integer registers
// (registers are all on stack)
def CC_Save : CalleeSavedRegs<(add AREG)>;
