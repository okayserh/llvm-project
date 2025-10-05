; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @fp_test1(float noundef %a, float noundef %b) {
entry:
  %a.addr = alloca float, align 4
  %b.addr = alloca float, align 4
  %d = alloca float, align 4
  %c = alloca i32, align 4
  store float %a, ptr %a.addr, align 4
  store float %b, ptr %b.addr, align 4
  %0 = load float, ptr %a.addr, align 4
  %1 = load float, ptr %b.addr, align 4
  %mul = fmul float %0, %1
  store float %mul, ptr %d, align 4
  %2 = load float, ptr %d, align 4
  %3 = load float, ptr %b.addr, align 4
  %cmp = fcmp oge float %2, %3
  %4 = zext i1 %cmp to i64
  %cond = select i1 %cmp, i32 25, i32 15
  store i32 %cond, ptr %c, align 4
  %5 = load i32, ptr %c, align 4
  ret i32 %5
; CHECK-LABEL: fp_test1:
; CHECK: stl 0
; CHECK: ajw -8
; CHECK: ldlp 7
; CHECK: fpldnlsn
; CHECK: ldlp 3
; CHECK: fpstnlsn
; CHECK: ldlp 3
; CHECK: fpldnlsn
; CHECK: ldlp 6
; CHECK: fpldnlmulsn
; CHECK: ldlp 4
; CHECK: fpstnlsn
; CHECK: ldlp 4
; CHECK: fpldnlsn
; CHECK: ldlp 1
; CHECK: fpstnlsn
; CHECK: ldlp 3
; CHECK: fpldnlsn
; CHECK: ldlp 4
; CHECK: fpldnlsn
; CHECK: fpordered
; CHECK: cj 3
; CHECK: fpgt
; CHECK: eqc 0
; CHECK: cj .LBB0_1
; CHECK: ldc 25
; CHECK: j .LBB0_3
; CHECK-LABEL: .LBB0_1:
; CHECK: ldc 15
; CHECK-LABEL: .LBB0_3:
; CHECK: stl 5
; CHECK: ldl 5
; CHECK: stl 2
; CHECK: ldl 5
; CHECK: ajw 8
; CHECK: ldl 0
; CHECK: gcall
}
