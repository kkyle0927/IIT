# 외부 LLM 질문 템플릿

아래 자료는 exoskeleton 센서 + trunk IMU만으로 human gait speed를 추정하는 프로젝트의 현재 최고 성능 모델 5개에 대한 요약이다.
target은 treadmill belt speed가 아니라 `common/speed_y`에 bidirectional LPF를 적용한 human gait speed다.
입력 feature 전처리는 항상 causal / online 가능해야 한다.

질문:

1. top 5 모델의 구조와 plot을 보면, 현재 오차를 가장 잘 설명하는 원인은 무엇인가?
2. 이 문제를 assist-dependent domain shift, calibration failure, representation mismatch 중 무엇으로 보는 것이 가장 타당한가?
3. `level_100mps`, `level_125mps`, `stopandgo`의 assisted condition에서 plateau underprediction을 줄이려면
   - 입력 feature
   - 모델 구조
   - loss / objective
   중 어느 축을 우선적으로 바꾸는 것이 좋은가?
4. causal / online feature 제약을 유지하면서도 성능을 높일 수 있는 구조적 제안을 해달라.
5. top 5 모델 중 어떤 구조를 baseline으로 삼아 다음 실험을 설계하는 것이 가장 합리적인가?

응답 시 요청:
- 추상적인 조언보다 구조, feature, 계산량, latency, deployability 기준으로 설명해달라.
- 가능하면 3개 이상의 대안을 비교한 뒤 최종안을 제안해달라.
