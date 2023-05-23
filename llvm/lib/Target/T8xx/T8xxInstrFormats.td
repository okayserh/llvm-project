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
  field bits<16> Inst;

  let Namespace = "T8xx";
  let DecoderNamespace = "T8xx";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString   = asmstr;
  let Pattern = pattern;
  let Size = 4;

  bits<8> op1;
  bits<8> op2;
  let Inst{15-8} = op2;               // Top four bits are the 'op' field
  let Inst{7-0} = op1;               // Top four bits are the 'op' field

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
    let op1 = opval;
    let Size = 1;
}


class T8InstWithOperands<bits<8> pfix1, bits<8> opval,
      	     dag outs, dag ins, string asmstr, list<dag> pattern,
             InstrItinClass itin = NoItinerary>
   : T8Inst<outs, ins, asmstr, pattern, itin> {
    let op1 = opval;
    let op2 = pfix1;
    let Size = 2;
    let isCodeGenOnly = 1;
}



class T8InstDirect<bits<8> opval, string asmstr,
             InstrItinClass itin = NoItinerary>
   : T8InstImm<opval, (outs), (ins i32imm:$imm), !strconcat(asmstr, " $imm"), [], itin> {
    let isCodeGenOnly = 0;
    let op1 = opval;
    let op2 = 0;
}


class T8InstShort<bits<8> opval, string asmstr,
             InstrItinClass itin = NoItinerary>
   : T8InstImm<opval, (outs), (ins), asmstr, [], itin> {
    let isCodeGenOnly = 0;
    let op1 = opval;
    let op2 = 0;
}


class T8InstOnePFix<bits<8> pfix1, bits<8> opval, string asmstr,
             InstrItinClass itin = NoItinerary>
   : T8InstImm<opval, (outs), (ins), asmstr, [], itin> {
   field bits<8> PFix1;
   let isCodeGenOnly = 0;
   let PFix1 = pfix1;

   let Size = 2;
   let op2 = pfix1;
   let op1 = opval;
}
