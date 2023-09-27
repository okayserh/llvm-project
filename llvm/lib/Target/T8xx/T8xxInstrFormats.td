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
  let Namespace = "T8xx";
  let DecoderNamespace = "T8xx";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString   = asmstr;
  let Pattern = pattern;
  let Itinerary = itin;
}


class T8Inst8<dag outs, dag ins, string asmstr, list<dag> pattern>
      :T8Inst<outs, ins, asmstr, pattern> {
      field bits<8> Inst;
      let Size = 1;

      bits<4> op;
      let Inst{7 - 4} = op;
}

class T8Inst16<dag outs, dag ins, string asmstr, list<dag> pattern>
      :T8Inst<outs, ins, asmstr, pattern> {
      field bits<16> Inst;
      let Size = 2;
}

class T8Inst24<dag outs, dag ins, string asmstr, list<dag> pattern>
      :T8Inst<outs, ins, asmstr, pattern> {
      field bits<24> Inst;
      let Size = 3;
}

class T8Inst32<dag outs, dag ins, string asmstr, list<dag> pattern>
      :T8Inst<outs, ins, asmstr, pattern> {
      field bits<32> Inst;
      let Size = 4;
}


// Pseudo instructions format
class T8xxPseudoInst<dag outs, dag ins, string asmstr, list<dag> pattern>
    : T8Inst<outs, ins, asmstr, pattern> {
  let isPseudo = 1;
}


class T8InstImm<bits<4> opval, dag outs, dag ins, string asmstr, list<dag> pattern>
   : T8Inst8<outs, ins, asmstr, pattern> {
   bits<4> imm;

   let op = opval;
   let Inst{3 - 0} = imm;

   let DecoderMethod = "decodeImm";
}


class T8InstOp4<bits<4> opval2, dag outs, dag ins, string asmstr, list<dag> pattern>
   : T8Inst8<outs, ins, asmstr, pattern> {

   let op = 0xF;
   let Inst{3 - 0} = opval2;

   let DecoderMethod = "decodeRegStack";
}

class T8InstOp8<bits<8> opval, dag outs, dag ins, string asmstr, list<dag> pattern>
   : T8Inst16<outs, ins, asmstr, pattern> {
   let Inst{15- 12} = 0x2;
   let Inst{11-  8} = opval{7-4};
   let Inst{7 -  4} = 0xF;
   let Inst{3 -  0} = opval{3-0};
   let DecoderMethod = "decodeRegStack";
}


// Special class of instruction for FPU of T800/1/5
class T8InstFPU4<bits<4> opval, dag outs, dag ins, string asmstr, list<dag> pattern>
   : T8Inst24<outs, ins, asmstr, pattern> {
   let Inst{23- 20} = 0x4;
   let Inst{19- 16} = opval;
   let Inst{15-  8} = 0x2A;
   let Inst{7 -  0} = 0xFB;
}

class T8InstFPU8<bits<8> opval, dag outs, dag ins, string asmstr, list<dag> pattern>
   : T8Inst32<outs, ins, asmstr, pattern> {
   let Inst{31- 28} = 0x2;
   let Inst{27- 24} = opval{7-4};
   let Inst{23- 20} = 0x4;
   let Inst{19- 16} = opval{3-0};
   let Inst{15-  8} = 0x2A;
   let Inst{7 -  0} = 0xFB;
}
