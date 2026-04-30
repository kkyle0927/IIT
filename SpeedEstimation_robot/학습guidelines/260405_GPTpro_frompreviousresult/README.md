# SpeedEstimation 전달용 번들

이 폴더는 학습 프로젝트/연구 노트로 바로 전달할 수 있도록, 이번 분석에서 생성된 결과물과 원본 컨텍스트를 한데 모은 번들입니다.

## 구성

### 1) analysis/
- `deep_analysis_speed_estimation.md`
  - 업로드된 프로젝트 정리 zip과 plot을 바탕으로 한 심층 분석 문서
  - 핵심 결론: 현재 병목은 모델 용량 부족보다 **assist-dependent representation mismatch + slow calibration failure** 쪽에 가깝고,
    다음 우선순위는 **fast nominal branch + slow assist-calibration branch** 구조를 시험하는 것
- `next_experiment_plan.csv`
  - 바로 실험 backlog에 넣을 수 있는 우선순위별 실험 계획표

### 2) plots/
- `trajectory_stack.png`
  - 주요 trajectory plot 비교 콜라주
- `training_curves_stack.png`
  - 대표 모델 학습곡선 비교
- `m2_S008_level_100mps_stack.png`
- `m2_S008_level_125mps_stack.png`
- `m2_S008_stopandgo_stack.png`
  - 조건별 top 모델 비교 콜라주

### 3) source_context/
- `260325_SpeedEstimation_정리.pdf`
  - 기존 내부 종합 보고서
- `GPTPro_용_프로젝트정리.zip`
  - 사용자가 제공한 원본 프로젝트 정리 패키지

## 가장 중요한 실험 제안
1. `J1_hybrid_power_base`를 baseline으로 사용
2. **fast nominal branch (IMU + kinematics + imu_integrated_speed)** 와
   **slow assist-calibration branch (EMA/RMS/impulse 기반 robot summary feature)** 로 분리
3. raw torque를 fast branch에서 제거
4. steady plateau bias를 직접 때리는 loss와 regime-wise evaluation을 추가

## 전달 시 한 줄 요약
"현재 실패의 핵심은 assisted plateau에서 생기는 느린 calibration failure와 일부 phase-locked confounding이므로,
더 큰 end-to-end 모델보다 two-timescale structured estimator가 더 유망하다."
