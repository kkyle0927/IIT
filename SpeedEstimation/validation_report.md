# SpeedEstimation Pre-Validation Report

생성: 2026-02-24 20:57:58

총 이슈: 45개
| 심각도 | 개수 |
|--------|------|
| CRITICAL | 0 |
| MAJOR | 1 |
| MINOR | 0 |
| INFO | 44 |

## 이슈 목록

### C1: CODE_BUG
**심각도:** MAJOR  
**조치:** FLAG_AND_EXCLUDE  

AR Feedback 훈련 코드가 teacher_forcing_schedule 설정값을 무시하고 항상 zeros(prev_channel)를 사용함. teacher_forcing: 1.0 (k1)과 teacher_forcing_schedule: 1.0→0.0 (sched)가 구현되지 않아 두 실험이 사실상 동일하게 동작.

> **Fix:** AR feedback 실험을 'AR mechanism analysis'에서 제외하고 주석 표기. 훈련/평가 모두 zeros를 사용해 내적 일관성은 유지됨. 실제 teacher forcing을 구현하려면 LazyWindowDataset 수정 + SpeedEstimator_TCN_MLP_experiments.py의 AR 루프 수정 필요 (CRITICAL NOTICE 파일이므로 사용자 승인 필요).

### C1b: METRICS_MATCH
**심각도:** INFO  
**조치:** DOCUMENT  

[확인] exp_ar_feedback_k1 MAEs=[0.04447, 0.017771, 0.026154]와 exp_ar_feedback_sched MAEs=[0.04447, 0.017771, 0.026154]가 완전히 일치. C1 이슈 확증.

> **Fix:** C1 이슈로 인한 예상된 결과.

### C2: METRICS_INTERPRETATION
**심각도:** INFO  
**조치:** DOCUMENT  

H=5 multi-step 실험. metrics.json test_mae=0.0963은 훈련 시 5개 step 평균(avg_H_mae=0.0962). compare_results.py의 MAE는 overlap_ensemble 미적용 시 step-1(t+1) 예측값만으로, 적용 시 overlap-averaged로 계산.

> **Fix:** 분석 시 metrics.json 대신 compare_results.py inference 결과(load_and_evaluate)를 canonical metric으로 사용.

영향 실험 (30개):
- exp_multistep_H05 / m1_S011: H=5 multi-step 실험. metrics.json test_mae=0.0151은 훈련 시 5개 step 평균(avg_H_mae=0.015
- exp_multistep_H05 / m2_S008: H=5 multi-step 실험. metrics.json test_mae=0.0349은 훈련 시 5개 step 평균(avg_H_mae=0.035
- exp_multistep_H10 / m1_S004: H=10 multi-step 실험. metrics.json test_mae=0.0222은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10 / m1_S011: H=10 multi-step 실험. metrics.json test_mae=0.0127은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10 / m2_S008: H=10 multi-step 실험. metrics.json test_mae=0.0468은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_ema_a090 / m1_S004: H=10 multi-step 실험. metrics.json test_mae=0.0222은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_ema_a090 / m1_S011: H=10 multi-step 실험. metrics.json test_mae=0.0127은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_ema_a090 / m2_S008: H=10 multi-step 실험. metrics.json test_mae=0.0468은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_mean / m1_S004: H=10 multi-step 실험. metrics.json test_mae=0.0222은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_mean / m1_S011: H=10 multi-step 실험. metrics.json test_mae=0.0127은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_mean / m2_S008: H=10 multi-step 실험. metrics.json test_mae=0.0468은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_median / m1_S004: H=10 multi-step 실험. metrics.json test_mae=0.0222은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_median / m1_S011: H=10 multi-step 실험. metrics.json test_mae=0.0127은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_median / m2_S008: H=10 multi-step 실험. metrics.json test_mae=0.0468은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_wmean / m1_S004: H=10 multi-step 실험. metrics.json test_mae=0.0222은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_wmean / m1_S011: H=10 multi-step 실험. metrics.json test_mae=0.0127은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_overlap_wmean / m2_S008: H=10 multi-step 실험. metrics.json test_mae=0.0468은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam001 / m1_S004: H=10 multi-step 실험. metrics.json test_mae=0.0222은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam001 / m1_S011: H=10 multi-step 실험. metrics.json test_mae=0.0127은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam001 / m2_S008: H=10 multi-step 실험. metrics.json test_mae=0.0468은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam01 / m1_S004: H=10 multi-step 실험. metrics.json test_mae=0.0222은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam01 / m1_S011: H=10 multi-step 실험. metrics.json test_mae=0.0127은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam01 / m2_S008: H=10 multi-step 실험. metrics.json test_mae=0.0468은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam05 / m1_S004: H=10 multi-step 실험. metrics.json test_mae=0.0222은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam05 / m1_S011: H=10 multi-step 실험. metrics.json test_mae=0.0127은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H10_smooth_lam05 / m2_S008: H=10 multi-step 실험. metrics.json test_mae=0.0468은 훈련 시 10개 step 평균(avg_H_mae=0.0
- exp_multistep_H20 / m1_S004: H=20 multi-step 실험. metrics.json test_mae=0.0490은 훈련 시 20개 step 평균(avg_H_mae=0.0
- exp_multistep_H20 / m1_S011: H=20 multi-step 실험. metrics.json test_mae=0.0209은 훈련 시 20개 step 평균(avg_H_mae=0.0
- exp_multistep_H20 / m2_S008: H=20 multi-step 실험. metrics.json test_mae=0.0517은 훈련 시 20개 step 평균(avg_H_mae=0.0

### C3: METRICS_INTERPRETATION
**심각도:** INFO  
**조치:** DOCUMENT  

EMA alpha=0.9 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.py의 MAE는 EMA 적용 후 값. 두 수치가 의도적으로 다름.

> **Fix:** 분석 시 반드시 compare_results.py inference 재실행으로 EMA 적용 후 MAE를 획득해야 함. metrics.json의 수치로 EMA 효과를 비교하면 안 됨.

영향 실험 (12개):
- exp_multistep_H10_ema_a090 / m1_S011: EMA alpha=0.9 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.p
- exp_multistep_H10_ema_a090 / m2_S008: EMA alpha=0.9 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.p
- exp_single_ema_a080 / m1_S004: EMA alpha=0.8 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.p
- exp_single_ema_a080 / m1_S011: EMA alpha=0.8 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.p
- exp_single_ema_a080 / m2_S008: EMA alpha=0.8 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.p
- exp_single_ema_a090 / m1_S004: EMA alpha=0.9 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.p
- exp_single_ema_a090 / m1_S011: EMA alpha=0.9 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.p
- exp_single_ema_a090 / m2_S008: EMA alpha=0.9 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.p
- exp_single_ema_a095 / m1_S004: EMA alpha=0.95 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.
- exp_single_ema_a095 / m1_S011: EMA alpha=0.95 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.
- exp_single_ema_a095 / m2_S008: EMA alpha=0.95 실험. metrics.json의 test_mae는 EMA 미적용 상태(훈련 시 측정). compare_results.

### C5: PASS
**심각도:** INFO  
**조치:** PASS  

compare_results.py가 SpeedEstimator_TCN_MLP_experiments.py의 build_nn_dataset_multi를 import하여 사용함. _dot 변수의 30Hz hardcoded LPF가 훈련/평가 모두 동일하게 적용됨.

> **Fix:** 일관성 확인됨. 조치 불필요.

## 분석 시 주의사항 요약

| Check | 영향 실험 | 분석 시 처리 방법 |
|-------|----------|-----------------|
| C1 | AR feedback (6개) | factor 분석에서 'AR 미구현' 주석 표기, AR mechanism 분석 제외 |
| C2 | H>1 multi-step (15개) | metrics.json 대신 compare_results.py inference 결과 사용 |
| C3 | EMA 실험 (9개) | compare_results.py inference 재실행으로 EMA 적용 후 MAE 획득 |
| C4 | 전체 (pass 시) | est_tick_ranges 명시 확인 완료 |
| C5 | 전체 | build_nn_dataset_multi 공유 확인 → 일관성 보장 |
| C6 | 전체 | 핵심 파라미터 일치 확인 |

> **결론:** Phase 1 분석에서 모든 60개 실험에 대해 `compare_results.py`의
> `load_and_evaluate()`를 통한 inference를 실행하고 그 결과를 canonical metric으로 사용.
> `metrics.json`의 값은 보조 참고용으로만 활용.