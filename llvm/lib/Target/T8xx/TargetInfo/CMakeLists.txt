add_llvm_component_library(LLVMT8xxInfo
  T8xxTargetInfo.cpp

  LINK_COMPONENTS
  MC
  Support

  ADD_TO_COMPONENT
  T8xx
  )
