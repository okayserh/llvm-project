add_llvm_component_library(LLVMT8xxDesc
  T8xxAsmBackend.cpp
  T8xxELFObjectWriter.cpp
  T8xxInstPrinter.cpp
  T8xxMCAsmInfo.cpp
  T8xxMCCodeEmitter.cpp
  T8xxMCTargetDesc.cpp
  T8xxMCExpr.cpp
  T8xxTargetStreamer.cpp

  LINK_COMPONENTS
  MC
  T8xxInfo
  Support

  ADD_TO_COMPONENT
  T8xx
  )
