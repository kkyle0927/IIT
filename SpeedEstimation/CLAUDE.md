# SpeedEstimation

Exoskeleton + full biomechanics 센서(mocap, GRF, IMU) 기반 treadmill 보행 속도 추정 딥러닝 프로젝트.

## 유지보수 규칙

- 프로젝트 구조, 코드 변경 등이 발생하면 이 CLAUDE.md도 함께 업데이트할 것
- 코드 구조가 변경되면 (새 모듈 추가 등) 관련 섹션 반영

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
- `SpeedEstimator_TCN_MLP_experiments.py` — GOLDEN BASELINE 학습 스크립트
- `train.sh` — SLURM 학습 제출 스크립트

## 실행 환경

- conda 환경: `IIT`
- SLURM 파티션: `idx0`~`idx3` (대시보드 GPU 번호 = SLURM idx 번호)
- GPU 독점 방지: 최대 3개까지만 사용
- 학습: `sbatch -p idxN --job-name="이름" --gres=gpu:idxN:1 --mem=15G --cpus-per-task=8 train.sh configs/path/to/config.yaml`
- 평가: `sbatch -p idxN --job-name="이름" --gres=gpu:idxN:1 eval_server.sh --auto --ablation`
- 로컬 테스트: `conda run -n IIT python SpeedEstimator_TCN_MLP_experiments.py --config configs/...`

### GPU 사용 규칙
- 사용자가 `gpu:idxN:1`로 지정할 때 N은 http://143.248.65.114:5050/ 대시보드 기준 GPU 번호 (nvidia-smi 번호 아님)
- train/eval 제출 시 반드시 `--job-name`을 지정하여 대시보드에서 job으로 식별 가능하게 할 것
- 작업 제출 전 대시보드(http://143.248.65.114:5050/)에서 다른 사용자가 점유 중인 GPU를 확인하고, 점유 중인 GPU에는 절대 작업을 올리지 말 것 (사용자가 특정 idx를 지정하더라도 해당 GPU가 타인 점유 중이면 사용자에게 알리고 다른 GPU를 제안)

## 프로젝트 구조

```
SpeedEstimation/
├── data/
│   ├── loader.py          # H5 데이터 로딩, derived feature 계산, trial-level 캐시
│   ├── body_frame.py      # quaternion 기반 body-aligned IMU 변환
│   ├── model_com_speed.py # kinematic COM speed + IMU integrated speed
│   └── utils.py           # Butterworth 필터, GRF contact 감지, H5 유틸
├── core/
│   ├── models.py          # TCN_MLP, TCN_GRU_Head, AttentionTCN 등 6개 모델
│   ├── datasets.py        # LazyWindowDataset (windowing + time warp augmentation)
│   └── losses.py          # smoothness, kinematic, gradient penalty loss
├── training/
│   └── trainer.py         # 학습 루프, LOSO CV, early stopping, AMP, checkpointing
├── configs/               # YAML 실험 설정
├── experiments/           # 학습 결과 (model.pt, metrics.json, scaler.npz)
├── SpeedEstimator_TCN_MLP_experiments.py  # 메인 학습 스크립트
├── compare_results.py     # 평가/비교/ablation 분석
├── validate_and_retrain.py # 학습-평가 일관성 감사
├── train.sh               # SLURM 학습 제출 (TUI config 선택 지원)
└── eval_server.sh         # SLURM 평가 제출
```

## 데이터 파이프라인

- 데이터: `combined_data.h5`, 구조: `Subject/Condition/Level/Trial`
- 피험자: S001~S008, 조건: level_075/100/125mps, accel_sine, decline_5deg, incline_10deg, stopandgo
- 샘플링: 100Hz, 출력: treadmill belt speed (m/s)

### 입력 feature 그룹
- `mocap/kin_q`: hip_flexion, knee_angle, ankle_angle (l/r) + _dot, _ddot (18ch)
- `derived/body_frame_imu`: body-aligned accel/gyro + forward_accel_overground (7ch)
- `derived/model_com_speed`: forward kinematics 기반 COM speed (1ch)
- `derived/imu_integrated_speed`: forward_accel_overground의 leaky integration (1ch)
- `robot/left`, `robot/right`: exo torque (2ch)
- `forceplate/grf/left`, `right`: x, y, z force + contact (8ch)
- `sub_info`: height, weight (2ch)

### Derived features (H5에 없고 loader에서 on-the-fly 계산)
- `derived/body_frame_imu`: quaternion 기반 body frame 변환
- `derived/model_com_speed`: joint angle forward kinematics + GRF stance detection
- `derived/imu_integrated_speed`: leaky integrator
- `_dot`, `_ddot` 변수: `np.gradient() * fs`
- `contact_l/r`: GRF z-force thresholding

### Trial-level 캐시
`TRIAL_DATASET_CACHE`로 H5 로딩 + derived feature 계산 결과를 캐시하여 LOSO fold 간 재사용.
첫 fold에서 전체 trial 로딩 후 캐시, 이후 fold에서는 subject 기준 필터링만 수행.

## Residual Skip Connection

`model.residual_skip.enable: true` + `input_channel_idx: N` 설정 시:
- `output = network(x) + x[:, -1, N]` (input normalization 전 raw 값 사용)
- 물리적 base estimate(COM/IMU speed) 위에 network가 보정값만 학습

## 학습 설정 요약

- 모델: TCN_GRU (기본), channels [32,32,64,64,128,128], kernel 5, GRU hidden 32
- 학습: batch 128, epochs 30, lr 0.001, AMP, Huber loss (delta=0.1)
- CV: LOSO (Leave-One-Subject-Out)
- 옵션 플래그: `--run_distribution_analysis`, `--run_feature_importance` (기본 비활성)
