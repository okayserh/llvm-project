add_llvm_component_library(LLVMT8xxAsmParser
  T8xxAsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  T8xxDesc
  T8xxInfo
  Support

  ADD_TO_COMPONENT
  T8xx
  )
