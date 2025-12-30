; RUN: llc < %s -march=t8xx | FileCheck %s -v --check-prefix=CHECK
; Corresponds to fp/test4.ll

define dso_local i32 @fp_mul_sn(float noundef %a, float noundef %b) #0 {
entry:
  %a.addr = alloca float, align 4
  %b.addr = alloca float, align 4
  %d = alloca float, align 4
  store float %a, ptr %a.addr, align 4
  store float %b, ptr %b.addr, align 4
  %0 = load float, ptr %a.addr, align 4
  %1 = load float, ptr %b.addr, align 4
  %mul = fmul float %0, %1
  store float %mul, ptr %d, align 4
  ret i32 0
}

; CHECK-LABEL: fp_mul_sn:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldlp 2
; CHECK: fpldnlsn
; CHECK: ldlp 3
; CHECK: fpldnlmulsn
; CHECK: ldlp 1
; CHECK: fpstnlsn
; CHECK: ldc 0
; CHECK: ajw 4
; CHECK: ldl 0
; CHECK: gcall

define dso_local i32 @fp_mul_db(double noundef %a, double noundef %b) #0 {
entry:
  %a.addr = alloca double, align 8
  %b.addr = alloca double, align 8
  %d = alloca float, align 4
  store double %a, ptr %a.addr, align 8
  store double %b, ptr %b.addr, align 8
  %0 = load double, ptr %a.addr, align 8
  %1 = load double, ptr %b.addr, align 8
  %mul = fmul double %0, %1
  %conv = fptrunc double %mul to float
  store float %conv, ptr %d, align 4
  ret i32 0
}

; CHECK-LABEL: fp_mul_db:
; CHECK: stl 0
; CHECK: ajw -4
; CHECK: ldlp 0
; CHECK: adc -48
; CHECK: ldc 7
; CHECK: not
; CHECK: and
; CHECK: gajw
; CHECK: stl 11
; CHECK: ldl 11
; CHECK: fpldnldb
; CHECK: ldlp 9
; CHECK: fpstnldb
; CHECK: ldl 11
; CHECK: adc 8
; CHECK: fpldnldb
; CHECK: ldlp 7
; CHECK: fpstnldb
; CHECK: ldlp 9
; CHECK: fpldnldb
; CHECK: ldlp 2
; CHECK: fpstnldb
; CHECK: ldlp 7
; CHECK: fpldnldb
; CHECK: ldlp 4
; CHECK: fpstnldb
; CHECK: ldlp 9
; CHECK: fpldnldb
; CHECK: ldlp 7
; CHECK: fpldnldb
; CHECK: fpmul
; CHECK: fpur64tor32
; CHECK: ldlp 6
; CHECK: fpstnlsn
; CHECK: ldc 0
; CHECK: ldl 11
; CHECK: gajw
; CHECK: rev
; CHECK: ajw 4
; CHECK: ldl 0
; CHECK: gcall

define dso_local i32 @fp_mul_sd(float noundef %a, double noundef %b) #0 {
entry:
  %a.addr = alloca float, align 4
  %b.addr = alloca double, align 8
  %d = alloca float, align 4
  store float %a, ptr %a.addr, align 4
  store double %b, ptr %b.addr, align 8
  %0 = load float, ptr %a.addr, align 4
  %conv = fpext float %0 to double
  %1 = load double, ptr %b.addr, align 8
  %mul = fmul double %conv, %1
  %conv1 = fptrunc double %mul to float
  store float %conv1, ptr %d, align 4
  ret i32 0
}

; CHECK-LABEL: fp_mul_sd:
; CHECK: stl 0
; CHECK: ajw -3
; CHECK: ldlp 0
; CHECK: adc -36
; CHECK: ldc 7
; CHECK: not
; CHECK: and
; CHECK: gajw
; CHECK: stl 7
; CHECK: ldl 7
; CHECK: adc 4
; CHECK: fpldnldb
; CHECK: ldlp 5
; CHECK: fpstnldb
; CHECK: ldlp 5
; CHECK: fpldnldb
; CHECK: ldlp 2
; CHECK: fpstnldb
; CHECK: ldl 7
; CHECK: fpldnlsn
; CHECK: fpur32tor64
; CHECK: ldlp 5
; CHECK: fpldnldb
; CHECK: fpmul
; CHECK: fpur64tor32
; CHECK: ldlp 4
; CHECK: fpstnlsn
; CHECK: ldc 0
; CHECK: ldl 7
; CHECK: gajw
; CHECK: rev
; CHECK: ajw 3
; CHECK: ldl 0
; CHECK: gcall

define dso_local i32 @fp_mul_ds(double noundef %a, float noundef %b) #0 {
entry:
  %a.addr = alloca double, align 8
  %b.addr = alloca float, align 4
  %d = alloca float, align 4
  store double %a, ptr %a.addr, align 8
  store float %b, ptr %b.addr, align 4
  %0 = load double, ptr %a.addr, align 8
  %1 = load float, ptr %b.addr, align 4
  %conv = fpext float %1 to double
  %mul = fmul double %0, %conv
  %conv1 = fptrunc double %mul to float
  store float %conv1, ptr %d, align 4
  ret i32 0
}

; CHECK-LABEL: fp_mul_ds:
; CHECK: stl 0
; CHECK: ajw -3
; CHECK: ldlp 0
; CHECK: adc -36
; CHECK: ldc 7
; CHECK: not
; CHECK: and
; CHECK: gajw
; CHECK: stl 7
; CHECK: ldl 7
; CHECK: fpldnldb
; CHECK: ldlp 5
; CHECK: fpstnldb
; CHECK: ldlp 5
; CHECK: fpldnldb
; CHECK: ldlp 2
; CHECK: fpstnldb
; CHECK: ldlp 5
; CHECK: fpldnldb
; CHECK: ldl 7
; CHECK: adc 8
; CHECK: fpldnlsn
; CHECK: fpur32tor64
; CHECK: fpmul
; CHECK: fpur64tor32
; CHECK: ldlp 4
; CHECK: fpstnlsn
; CHECK: ldc 0
; CHECK: ldl 7
; CHECK: gajw
; CHECK: rev
; CHECK: ajw 3
; CHECK: ldl 0
; CHECK: gcall
