; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK

define dso_local i32 @fp_test2(float noundef %a, double noundef %b) {
entry:
  %a.addr = alloca float, align 4
  %b.addr = alloca double, align 8
  %d = alloca float, align 4
  %c = alloca i32, align 4
  store float %a, ptr %a.addr, align 4
  store double %b, ptr %b.addr, align 8
  %0 = load float, ptr %a.addr, align 4
  %conv = fpext float %0 to double
  %1 = load double, ptr %b.addr, align 8
  %mul = fmul double %conv, %1
  %conv1 = fptrunc double %mul to float
  store float %conv1, ptr %d, align 4
  %2 = load float, ptr %d, align 4
  %conv2 = fpext float %2 to double
  %3 = load double, ptr %b.addr, align 8
  %cmp = fcmp oge double %conv2, %3
  %4 = zext i1 %cmp to i64
  %cond = select i1 %cmp, i32 25, i32 15
  store i32 %cond, ptr %c, align 4
  %5 = load i32, ptr %c, align 4
  ret i32 %5
; CHECK-LABEL: fp_test2:
; CHECK: stl 0
; CHECK: ajw -11
; CHECK: ldlp 9
; CHECK: fpldnldb
; CHECK: ldlp 2
; CHECK: fpstnldb
; CHECK: ldlp 10
; CHECK: fpldnlsn
; CHECK: ldlp 2
; CHECK: fpldnldb
; CHECK: fpmul
; CHECK: fpur64tor32
; CHECK: ldlp 4
; CHECK: fpstnlsn
; CHECK: ldlp 2
; CHECK: fpldnldb
; CHECK: ldlp 7
; CHECK: fpstnldb
; CHECK: ldlp 4
; CHECK: fpldnlsn
; CHECK: ldlp 6
; CHECK: fpstnlsn
; CHECK: ldlp 2
; CHECK: fpldnldb
; CHECK: ldlp 4
; CHECK: fpldnlsn
; CHECK: fpur32tor64
; CHECK: fpgt
; CHECK: eqc 0
; CHECK: j .LBB0_1
; CHECK: ldc 15
; CHECK: j .LBB0_3
; CHECK-LABEL: .LBB0_1:
; CHECK: ldc 25
; CHECK-LABEL: .LBB0_3:
; CHECK: stl 1
; CHECK: ldl 1
; CHECK: stl 5
; CHECK: ldl 1
; CHECK: ajw 11
; CHECK: ldl 0
; CHECK: gcall
}
