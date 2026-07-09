# IL Evaluation and Benchmark

학습이 끝난 뒤에는 offline evaluation과 runtime benchmark를 모두 확인해야 합니다.

## Offline evaluation

```bash
ros2 run il_data_tools eval_policy.py \
  --csv ~/xycar_ws/datasets/processed/drive/test.csv \
  --model ~/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18_scripted.pt \
  --model-type resnet18 \
  --output-dir ~/xycar_ws/models/il_policies/drive_resnet18/eval \
  --max-steer-deg 100
```

출력:

- `predictions.csv`
- `eval_metrics.json`

phase 모델이면 `--use-phase`를 추가하거나 `model-type`에 `_phase`가 포함된 이름을 사용합니다.

```bash
ros2 run il_data_tools eval_policy.py \
  --csv ~/xycar_ws/datasets/processed/overtake/test.csv \
  --model ~/xycar_ws/models/il_policies/overtake_pilotnet_phase/overtake_pilotnet_phase_scripted.pt \
  --model-type pilotnet_phase \
  --use-phase \
  --output-dir ~/xycar_ws/models/il_policies/overtake_pilotnet_phase/eval \
  --max-steer-deg 100
```

## Prediction visualization

큰 오차 sample을 눈으로 확인합니다.

```bash
ros2 run il_data_tools visualize_policy_predictions.py \
  --predictions-csv ~/xycar_ws/models/il_policies/drive_resnet18/eval/predictions.csv \
  --output-dir ~/xycar_ws/models/il_policies/drive_resnet18/eval/visualized \
  --top-n 50 \
  --random-n 50
```

## Jetson Orin Nano benchmark

TorchScript `.pt`가 실시간으로 충분히 빠른지 확인합니다.

```bash
python3 ~/xycar_ws/src/kookmin_autonomous_competition_teamKAI/il_data_tools/scripts/benchmark_policy_model.py \
  --model ~/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18_scripted.pt \
  --device cuda \
  --image-width 160 \
  --image-height 90 \
  --iterations 500
```

JSON으로 저장하려면:

```bash
python3 ~/xycar_ws/src/kookmin_autonomous_competition_teamKAI/il_data_tools/scripts/benchmark_policy_model.py \
  --model ~/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18_scripted.pt \
  --device cuda \
  --image-width 160 \
  --image-height 90 \
  --iterations 500 \
  --output-json ~/xycar_ws/models/il_policies/drive_resnet18/benchmark.json
```

phase 모델 benchmark:

```bash
python3 ~/xycar_ws/src/kookmin_autonomous_competition_teamKAI/il_data_tools/scripts/benchmark_policy_model.py \
  --model ~/xycar_ws/models/il_policies/overtake_pilotnet_phase/overtake_pilotnet_phase_scripted.pt \
  --device cuda \
  --phase-enabled \
  --image-width 160 \
  --image-height 90 \
  --iterations 500
```

## Latency guide

| p95 latency | 판단 |
|---:|---|
| under 20 ms | excellent |
| 20~35 ms | good |
| 35~50 ms | acceptable |
| 50~80 ms | risky |
| over 80 ms | too slow |

## Model comparison

여러 모델의 평가/벤치마크 결과를 모읍니다.

```bash
ros2 run il_data_tools compare_models.py \
  --eval-metrics ~/xycar_ws/models/il_policies/drive_resnet18/eval/eval_metrics.json \
  --benchmark-json ~/xycar_ws/models/il_policies/drive_resnet18/benchmark.json \
  --output-dir ~/xycar_ws/models/il_policies/comparison
```

출력:

- `model_comparison.csv`
- `model_comparison.md`

## 최종 선택 기준

최종 모델은 validation loss만 보고 고르면 안 됩니다.

함께 판단하세요.

- offline MAE/RMSE
- Jetson Orin Nano p95 latency
- 실제 저속 주행 안정성
- steering oscillation
- penalty risk
- rule-based safety와의 호환성

실차에서는 반드시 낮은 속도, 안전한 공간, rule-based 안전 로직이 살아 있는 상태에서 검증해야 합니다.
