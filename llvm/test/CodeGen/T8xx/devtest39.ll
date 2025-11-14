; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test39(i32 noundef %c) #0 {
entry:
  %c.addr = alloca i32, align 4
  %sc = alloca i16, align 2
  %res = alloca i32, align 4
  store i32 %c, ptr %c.addr, align 4
  store i16 65, ptr %sc, align 2
  %0 = load i16, ptr %sc, align 2
  %conv = sext i16 %0 to i32
  store i32 %conv, ptr %res, align 4
  %1 = load i32, ptr %res, align 4
  ret i32 %1
; CHECK-LABEL: test39:
; CHECK: stl 0
; CHECK: ajw -5
; CHECK: ldc 65
; CHECK: stl 3
; CHECK: ldlp 3
; CHECK: ldlp 1
; CHECK: ldc 2
; CHECK: move
; CHECK: ldc 65
; CHECK: stl 2
; CHECK: ldc 65
; CHECK: ajw 5
; CHECK: ldl 0
; CHECK: gcall
}
