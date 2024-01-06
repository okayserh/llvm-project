//===-- T8xxInstrAliases.td - Instruction Aliases for T8xx Target -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains instruction aliases for T8xx.
//===----------------------------------------------------------------------===//

// Section A.3 Synthetic Instructions

// Most are marked as Emit=0, so that they are not used for disassembly. This is
// an aesthetic issue, but the chosen policy is to typically prefer using the
// non-alias form, except for the most obvious and clarifying aliases: cmp, jmp,
// call, tst, ret, retl.

// Note: cmp is handled in T8xxInstrInfo.
//       jmp/call/ret/retl have special case handling for output in
//       T8xxInstPrinter.cpp

// jmp addr -> jmpl addr, %g0
/*
def : InstAlias<"jmp $addr", (JMPLrr G0, MEMrr:$addr), 0>;
def : InstAlias<"jmp $addr", (JMPLri G0, MEMri:$addr), 0>;
*/

def : InstAlias<"stl $addr", (STL AREG, WPtrSrc:$addr), 0>;
def : InstAlias<"ldl $addr", (LDL AREG, WPtrSrc:$addr), 0>;

def : InstAlias<"ldnl $addr", (LDNL AREG, RPtrSrc:$addr), 0>;
def : InstAlias<"stnl $addr", (STNL AREG, RPtrSrc:$addr), 0>;


/*
def : MnemonicAlias<"iflush", "flush">;
*/
