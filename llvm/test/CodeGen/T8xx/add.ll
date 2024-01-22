; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

; CHECK-LABEL: foo:
; Generated from test3.c
; Inspired by SPARC/32abi.ll

; CHECK: stl 0
; CHECK: ajw -3
; CHECK: ldl 2
; CHECK: adc 1
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 2
; CHECK: ldl 1
; CHECK: ajw 3
; CHECK: ldl 0
; CHECK: gcall

define dso_local i32 @foo(i32 noundef %c) #0 {
entry:
  %c.addr = alloca i32, align 4
  store i32 %c, ptr %c.addr, align 4
  %0 = load i32, ptr %c.addr, align 4
  %inc = add nsw i32 %0, 1
  store i32 %inc, ptr %c.addr, align 4
  %1 = load i32, ptr %c.addr, align 4
  ret i32 %1
}
