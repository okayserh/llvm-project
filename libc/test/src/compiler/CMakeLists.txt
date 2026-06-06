add_custom_target(libc_stack_chk_guard_unittests)

# TODO: OKH/T8XX, Revisit at a later time if a suitable baremetal implementation can be provided
# The test requires some basic process handling
if(NOT LIBC_TARGET_OS_IS_BAREMETAL)
  add_libc_unittest(
    stack_chk_guard_test
    SUITE
      libc_stack_chk_guard_unittests
    SRCS
      stack_chk_guard_test.cpp
    DEPENDS
      libc.hdr.signal_macros
      libc.src.__support.macros.sanitizer
      libc.src.compiler.__stack_chk_fail
      libc.src.string.memset
    COMPILE_OPTIONS
      -fstack-protector-all
    )
endif()
