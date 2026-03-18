# Misalign_compensation

Hip assistive wearable robot과 human hip joint 사이의 기구적 정합성 문제를 보상하는 프로젝트.
Human gait 중 hip rotation 및 hip abduction 시 발생하는 misalignment로 인해
robot이 측정하는 thigh angle과 실제 human thigh angle이 다른 경우가 있으며,
이 간극을 보상하기 위한 알고리즘을 개발한다.

## 유지보수 규칙

- 프로젝트 구조, 코드 변경 등이 발생하면 이 CLAUDE.md도 함께 업데이트할 것
- 코드 구조가 변경되면 관련 섹션 반영

## 파일 생성 규칙

- 새로운 파일을 repository에 추가하기 전 반드시 사용자에게 한국어로 파일 경로, 목적, 생성 이유를 설명하고 허락을 받을 것
- 작업 중 임시 파일을 만들었다면 같은 작업 안에서 직접 삭제할 경우 사용자 허락 없이 진행해도 됨
- 기존 코드 실행 결과로 생성되는 `png`, `log`, `csv`, `pt`, `npz` 등의 산출물은 사용자 허락 없이 생성되어도 됨

## CLAUDE.md 작성 규칙

- 이 파일에 규칙을 추가하거나 수정할 때 `가능하면`, `적절히`, `웬만하면` 같은 모호한 표현을 쓰지 말 것
- 규칙은 검증 가능한 조건-행동 형식으로 작성할 것. 예: `X를 바꾸면 Y도 같이 갱신할 것`
- 권장 표현보다 금지 표현을 우선 사용할 것. 예: `X를 쓰지 말고 Y만 사용할 것`
- 이 파일은 200줄 이하를 유지할 것
- 규칙이 길어지면 핵심 규칙만 이 파일에 남기고 나머지는 별도 문서나 규칙 파일로 분리할 것

## 수정 제한 파일

아래 파일들은 수정 전 반드시 사용자에게 한국어로 변경 내용/이유를 설명하고 허락을 받아야 함:
- `model_training.py` — GOLDEN BASELINE 학습 스크립트
- `train_server.sh` — SLURM 학습 제출 스크립트

## 실행 환경

- conda 환경: `IIT`
- SLURM 파티션: `idx0`~`idx3` (대시보드 GPU 번호 = SLURM idx 번호)
- GPU 독점 방지: 최대 3개까지만 사용
- 학습: `sbatch -p idxN --job-name="이름" --gres=gpu:idxN:1 --mem=15G --cpus-per-task=8 train_server.sh configs/path/to/config.yaml`
- 평가: `sbatch -p idxN --job-name="이름" --gres=gpu:idxN:1 eval_server.sh --auto`
- 로컬 테스트: `conda run -n IIT python model_training.py --config configs/...`

### GPU 사용 규칙
- 사용자가 `gpu:idxN:1`로 지정할 때 N은 http://143.248.65.114:5050/ 대시보드 기준 GPU 번호 (nvidia-smi 번호 아님)
- train/eval 제출 시 반드시 `--job-name`을 지정하여 대시보드에서 job으로 식별 가능하게 할 것
- 작업 제출 전 대시보드(http://143.248.65.114:5050/)에서 다른 사용자가 점유 중인 GPU를 확인하고, 점유 중인 GPU에는 절대 작업을 올리지 말 것 (사용자가 특정 idx를 지정하더라도 해당 GPU가 타인 점유 중이면 사용자에게 알리고 다른 GPU를 제안)

## 프로젝트 구조 (모놀리식)

모듈 분리 없이 `model_training.py` 하나에 데이터 로딩, 모델 정의, 학습 루프 등 전체 파이프라인이 포함됨.

```
Misalign_compensation/
├── model_training.py      # 전체 파이프라인 (~3,400줄): 데이터 로딩, body frame 변환,
│                          # 모델 정의, 학습 루프, feature importance
├── compare_results.py     # 평가/비교/ablation 분석
├── config_utils.py        # YAML config getter 헬퍼 (~50개 함수)
├── visualize_features.py  # Feature visualization hook (현재 no-op)
├── generate_experiments.py # Config 자동 생성 도구
├── configs/               # YAML 실험 설정
│   ├── baseline.yaml      # 마스터 config
│   ├── archive_season1/   # 과거 실험
│   └── archive_season2/   # 실험
├── experiments/           # 학습 결과
├── train_server.sh        # SLURM 학습 제출 (TUI config 선택)
├── eval_server.sh         # SLURM 평가 제출
└── combined_data.h5       # symlink → ../combined_data.h5
```

## 데이터 파이프라인

- 데이터: `combined_data.h5`, 구조: `Subject/Condition/Level/Trial`
- 피험자: S001~S008 (m2_ prefix), 조건: 7개 (level_075/100/125mps, accel_sine, decline/incline, stopandgo)
- 샘플링: 100Hz
- Level selection: lv0 (misalign 없음), lv4, lv7 (misalign 있음)
- 출력: `mocap/kin_q/hip_flexion_l,r` (실제 human hip flexion angle)

### Raw 데이터 (H5에 저장됨)
- `robot/back_imu`: accel_x/y/z, gyro_x/y/z, quat_w/x/y/z
- `robot/left`, `robot/right`: hip_angle, thigh_angle, torque
- `sub_info`: height, weight

### 가공된 입력 features
- `derived/euler`: complementary filter roll/pitch/yaw (3ch)
- `derived/body_frame_imu`: body-aligned accel/gyro (실험에 따라)
- `robot/left`, `robot/right`: hip_angle, thigh_angle, torque (6ch)
- `robot/back_imu`: quat_w/x/y/z (4ch)
- `sub_info`: height, weight

### Body frame 변환 (핵심 알고리즘)
`estimate_body_frame_features()` (model_training.py 내부):
1. Quaternion canonicalization (sign continuity)
2. Quaternion → rotation matrix → global frame 변환
3. Gravity 제거 → linear acceleration
4. Sliding window PCA로 forward direction 추정 (motion-adaptive)
5. Orthonormal body frame 구축 (right, forward, up)
6. Treadmill acceleration 보상 → forward_accel_overground

### Complementary filter
`ComplementaryFilter` class: IMU accel + gyro fusion으로 Euler angle 추정.
`use_complementary_filter_euler: true` 설정 시 활성화.

## YAML config 구조

3-section 계층:
```yaml
shared:       # 공통 설정 (data_sources, subjects, conditions, input_vars, output_vars)
02_train:     # 학습 전용 오버라이드 (model, loader, train params)
03_eval:      # 평가 전용 (split mode, loso_subjects)
```
우선순위: `02_train` > `shared` > root level. `config_utils.py`의 getter 함수들이 이 우선순위 처리.

## 학습 설정 요약

- 모델: TCN (기본), channels [32,64,64,128], kernel 5, dropout 0.15
- 학습: batch 128, epochs 30, lr 0.001, wd 0.01, AMP
- Loss: Huber (기본) + smoothness/kinematic/frequency penalty (선택적)
- CV: LOSO (Leave-One-Subject-Out)
- 옵션: `--run_distribution_analysis`, `--run_feature_importance` (기본 비활성)
