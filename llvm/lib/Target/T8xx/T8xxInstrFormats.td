//===-- T8xxInstrFormats.td - T8xx Instruction Formats ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

class T8Inst<dag outs, dag ins, string asmstr, list<dag> pattern,
             InstrItinClass itin = NoItinerary>
   : Instruction {
  field bits<8> Inst;

  let Namespace = "T8";
  let Size = 0;

  bits<4> op;
  let Inst{7-4} = op;               // Top four bits are the 'op' field

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString   = asmstr;
  let Pattern = pattern;

  let DecoderNamespace = "T8xx";
  field bits<32> SoftFail = 0;

  let Itinerary = itin;
}

class T8InstImm<bits<4> opval, dag outs, dag ins, string asmstr, list<dag> pattern,
             InstrItinClass itin = NoItinerary>
   : T8Inst<outs, ins, asmstr, pattern, itin> {
  let op = opval;
}

