; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @test_ui_lt(i32 noundef %a, i32 noundef %b) #0 {
entry:
  %a.addr = alloca i32, align 4
  %b.addr = alloca i32, align 4
  %res = alloca i32, align 4
  store i32 %a, ptr %a.addr, align 4
  store i32 %b, ptr %b.addr, align 4
  store i32 0, ptr %res, align 4
  %0 = load i32, ptr %a.addr, align 4
  %1 = load i32, ptr %b.addr, align 4
  %cmp = icmp ult i32 %0, %1
  br i1 %cmp, label %if.then, label %if.end

if.then:                                          ; preds = %entry
  store i32 1, ptr %res, align 4
  br label %if.end

if.end:                                           ; preds = %if.then, %entry
  %2 = load i32, ptr %res, align 4
  ret i32 %2
}

; CHECK-LABEL: test_ui_lt:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldc 0
; CHECK: stl 1
; CHECK: ldc 1
; CHECK: ldl 3
; CHECK: ldl 2
; CHECK: ldiff
; CHECK: rev
; CHECK: eqc 0
; CHECK: cj .LBB0_2
; CHECK-LABEL: .LBB0_1:
; CHECK: ldc 1
; CHECK: stl 1
; CHECK-LABEL: .LBB0_2:
; CHECK: ldl 1
; CHECK: ajw 4
; CHECK: ldl 0
; CHECK: gcall
