add_llvm_component_group(T8xx)

set(LLVM_TARGET_DEFINITIONS T8xx.td)

tablegen(LLVM T8xxGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM T8xxGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM T8xxGenCallingConv.inc -gen-callingconv)
tablegen(LLVM T8xxGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM T8xxGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM T8xxGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM T8xxGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM T8xxGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM T8xxGenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(T8xxCommonTableGen)

add_llvm_target(T8xxCodeGen
  T8xxAsmPrinter.cpp
  T8xxInstrInfo.cpp
  T8xxISelDAGToDAG.cpp
  T8xxISelLowering.cpp
  T8xxFrameLowering.cpp
  T8xxMachineFunctionInfo.cpp
  T8xxRegisterInfo.cpp
  T8xxSubtarget.cpp
  T8xxTargetMachine.cpp
  T8xxMCInstLower.cpp
  T8xxTargetObjectFile.cpp
  T8xxStackifyInt.cpp
  T8xxDebugValueManager.cpp
  
  LINK_COMPONENTS
  AsmPrinter
  CodeGen
  Core
  MC
  SelectionDAG
  T8xxDesc
  T8xxInfo
  Support
  Target

  ADD_TO_COMPONENT
  T8xx
  )

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
