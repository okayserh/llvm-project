; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @deepstack(i32 noundef %a, ptr noundef %b, i32 noundef %c) #0 {
; CHECK-LABEL: deepstack:
; CHECK: ldl 9
; CHECK: stl 2
; CHECK: ldl 7
; CHECK: stl 3
; CHECK: ldl 8
; CHECK: stl 4
; CHECK: ldl 2
; CHECK: ldl 4
; CHECK: ldnl 0
; CHECK: add
; CHECK: ldl 3
; CHECK: add
; CHECK: stl 1
; CHECK: ldl 2
; CHECK: ldl 3
; CHECK: mul
; CHECK: ldc 2
; CHECK: shl
; CHECK: ldl 4
; CHECK: add
; CHECK: ldnl 0
; CHECK: ldl 2
; CHECK: mul
; CHECK: ldl 3
; CHECK: ldl 4
; CHECK: adc 4
; CHECK: ldnl 0
; CHECK: add
; CHECK: add
; CHECK: ldl 1
; CHECK: div
; CHECK: stl 5

entry:
  %a.addr = alloca i32, align 4
  %b.addr = alloca ptr, align 4
  %c.addr = alloca i32, align 4
  %d = alloca i32, align 4
  store i32 %a, ptr %a.addr, align 4
  store ptr %b, ptr %b.addr, align 4
  store i32 %c, ptr %c.addr, align 4
  %0 = load i32, ptr %a.addr, align 4
  %1 = load ptr, ptr %b.addr, align 4
  %2 = load i32, ptr %a.addr, align 4
  %3 = load i32, ptr %c.addr, align 4
  %mul = mul nsw i32 %2, %3
  %arrayidx = getelementptr inbounds i32, ptr %1, i32 %mul
  %4 = load i32, ptr %arrayidx, align 4
  %mul1 = mul nsw i32 %0, %4
  %5 = load i32, ptr %c.addr, align 4
  %6 = load ptr, ptr %b.addr, align 4
  %arrayidx2 = getelementptr inbounds i32, ptr %6, i32 1
  %7 = load i32, ptr %arrayidx2, align 4
  %add = add nsw i32 %5, %7
  %add3 = add nsw i32 %mul1, %add
  %8 = load i32, ptr %a.addr, align 4
  %9 = load ptr, ptr %b.addr, align 4
  %arrayidx4 = getelementptr inbounds i32, ptr %9, i32 0
  %10 = load i32, ptr %arrayidx4, align 4
  %add5 = add nsw i32 %8, %10
  %11 = load i32, ptr %c.addr, align 4
  %add6 = add nsw i32 %add5, %11
  %div = sdiv i32 %add3, %add6
  store i32 %div, ptr %d, align 4
  %12 = load i32, ptr %d, align 4
  ret i32 %12
}
