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

  let Namespace = "T8xx";
  let DecoderNamespace = "T8xx";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString   = asmstr;
  let Pattern = pattern;
  let Size = 4;

  bits<8> op;
  let Inst = op;               // Top four bits are the 'op' field

//  let Inst = 0;               // Top four bits are the 'op' field

  let Itinerary = itin;
}


// LEG pseudo instructions format
class T8xxPseudoInst<dag outs, dag ins, string asmstr, list<dag> pattern>
    : T8Inst<outs, ins, asmstr, pattern> {
  let isPseudo = 1;
}

class T8InstImm<bits<8> opval, dag outs, dag ins, string asmstr, list<dag> pattern,
             InstrItinClass itin = NoItinerary>
   : T8Inst<outs, ins, asmstr, pattern, itin> {
    let op = opval;
}

class T8InstDirect<bits<8> opval, string asmstr,
             InstrItinClass itin = NoItinerary>
   : T8InstImm<opval, (outs), (ins i32imm:$imm), !strconcat(asmstr, " $imm"), [], itin> {
   let isCodeGenOnly = 0;
}


class T8InstShort<bits<8> opval, string asmstr,
             InstrItinClass itin = NoItinerary>
   : T8InstImm<opval, (outs), (ins), asmstr, [], itin> {
   let isCodeGenOnly = 0;
}
