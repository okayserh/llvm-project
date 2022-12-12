add_llvm_component_library(LLVMT8xxDisassembler
  T8xxDisassembler.cpp

  LINK_COMPONENTS
  MCDisassembler
  T8xxInfo
  Support

  ADD_TO_COMPONENT
  T8xx
  )
