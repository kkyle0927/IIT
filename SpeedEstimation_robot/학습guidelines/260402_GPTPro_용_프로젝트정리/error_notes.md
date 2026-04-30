# 현재 오차 양상

## 관찰된 패턴
- `lv0`에서는 대체로 잘 맞는다.
- `lv4`, `lv7`로 갈수록 특정 조건에서 prediction plateau가 낮아진다.
- 특히 아래 조건에서 확인이 필요하다.
  - `m2_S008_level_100mps`
  - `m2_S008_level_125mps`
  - `m2_S008_stopandgo`

## 해석
- assist가 커질수록 입력 분포는 변하지만 target gait speed는 크게 변하지 않는 상황이라,
  모델이 assist-induced pattern change를 speed 감소로 오인할 가능성이 있다.
- torque를 직접 speed predictor로 쓰는 구조는 일관되게 강하지 않았고,
  `power/envelope + kinematics hybrid` 계열이 더 견고했다.

## 외부 LLM에 기대하는 답
- 이 오차를 calibration failure / domain shift / target mismatch 중 무엇으로 보는가
- 입력 feature, 구조, objective 중 어떤 축을 우선적으로 바꾸는 게 맞는가
- causal 제약을 유지하면서도 assisted condition bias를 줄일 수 있는 구조가 무엇인가
