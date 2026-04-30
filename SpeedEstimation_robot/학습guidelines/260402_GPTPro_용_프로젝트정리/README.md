# GPTPro용 프로젝트 정리

이 폴더는 외부 web 기반 LLM에게 `SpeedEstimation_robot` 프로젝트의 현재 상태를 빠르게 설명하고,
상위 5개 모델의 구조와 오차 양상을 함께 보여주기 위한 패키지다.

핵심 전제:
- 목표 target은 treadmill belt speed가 아니라 `common/speed_y`에 bidirectional LPF를 적용한 human gait speed다.
- 입력 feature 전처리는 항상 real-time / causal 처리 가능해야 한다.
- output target 생성용 offline smoothing은 허용된다.

구성:
- `overview.md`: 프로젝트 목적, 입력, target, 제약, 현재 핵심 문제
- `llm_questions.md`: 외부 LLM에 바로 넣을 질문 템플릿
- `error_notes.md`: 현재 관찰된 오차 패턴과 해석
- `top5_summary.csv`, `top5_summary.json`: 상위 5개 모델 요약
- `model_cards/`: 모델별 구조/입력/장단점 요약
- `plots/`: 모델별 핵심 png 복사본

현재 season2 기준 top 5:
1. `exp_S2A1_J7_hybrid_longwin_envelope_Test-m2_S008_seed42`
2. `exp_S2A1_J1_hybrid_power_base_Test-m2_S008_seed42`
3. `exp_S2A1_L7_affine_calibration_biasloss_Test-m2_S008_seed42`
4. `exp_S2A1_G2_fullkin_deeptcn_Test-m2_S008_seed42`
5. `exp_S2A1_M8_positive_residual_bestshot_Test-m2_S008_seed42`
