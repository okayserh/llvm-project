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
  // i32 f32 arguments get passed in integer registers if there is space.
//  CCIfType<[i32], CCAssignToReg<[R1, R2, R3]>>,

  // Alternatively, they are assigned to the stack in 4-byte aligned units.
  CCIfType<[i32], CCAssignToStack<4, 4>>,

  CCIfType<[f32], CCAssignToStack<4, 4>>
]>;

def RetCC_T8xx32 : CallingConv<[
  // i32 are returned in registers R1, R2
  CCIfType<[i32], CCAssignToReg<[AREG]>>,

  CCIfType<[f32], CCAssignToReg<[FAREG]>>,

// Integer values get stored in stack slots that are 4 bytes in
  // size and 4-byte aligned.
//  CCIfType<[i32], CCAssignToStack<4, 4>>
]>;

// TODO: Probably needs to span all integer registers
// (registers are all on stack)
def CC_Save : CalleeSavedRegs<(add AREG)>;
