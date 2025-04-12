; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @fp_test3(float noundef %a, float noundef %b) {
entry:
  %a.addr = alloca float, align 4
  %b.addr = alloca float, align 4
  %d = alloca float, align 4
  store float %a, ptr %a.addr, align 4
  store float %b, ptr %b.addr, align 4
  %0 = load float, ptr %a.addr, align 4
  %1 = load float, ptr %b.addr, align 4
  %2 = load float, ptr %b.addr, align 4
  %3 = call float @llvm.fmuladd.f32(float %0, float %1, float %2)
  store float %3, ptr %d, align 4
  ret i32 0
; CHECK-LABEL: fp_test3:
; CHECK: stl 0
; CHECK: ajw -5
; CHECK: ldlp 3
; CHECK: fpldnlsn
; CHECK: ldlp 1
; CHECK: fpstnlsn
; CHECK: ldlp 4
; CHECK: fpldnlsn
; CHECK: ldlp 1
; CHECK: fpldnlsn
; CHECK: fpmul
; CHECK: ldlp 1
; CHECK: fpldnlsn
; CHECK: fpadd
; CHECK: ldlp 2
; CHECK: fpstnlsn
; CHECK: ldc 0
; CHECK: ajw 5
; CHECK: ldl 0
; CHECK: gcall
}
