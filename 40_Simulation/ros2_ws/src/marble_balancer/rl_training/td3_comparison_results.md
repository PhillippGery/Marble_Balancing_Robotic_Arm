# TD3 vs LQR Comparison Results

| Field | Value |
|---|---|
| Date | 2026-05-06 |
| Model | `checkpoint_td3_v2_200000_steps.zip` |
| Episodes | 10 |
| Friction (mu) | 0.3 |
| TCP Lissajous | true |

## Results

| Metric | RL | LQR | Improvement |
|---|---|---|---|
| Reward (avg) | +679.803 ± 98.352 | +728.050 ± 42.072 | -6.6% |
| RMSE Error (m) | 0.1787 ± 0.0540 | 0.1981 ± 0.0561 | +9.8% |
| Max Error (m) | 0.3743 ± 0.1296 | 0.3939 ± 0.0896 | +5.0% |
| Win Rate | 3/10 (30%) | 7/10 (70%) | — |

---

| Field | Value |
|---|---|
| Date | 2026-05-06 |
| Model | `checkpoint_td3_v2_200000_steps.zip` |
| Episodes | 10 |
| Friction (mu) | 0.5 |
| TCP Lissajous | true |

## Results

| Metric | RL | LQR | Improvement |
|---|---|---|---|
| Reward (avg) | +720.342 ± 52.430 | +747.047 ± 38.894 | -3.6% |
| RMSE Error (m) | 0.1741 ± 0.0538 | 0.2120 ± 0.0404 | +17.9% |
| Max Error (m) | 0.3581 ± 0.1557 | 0.4567 ± 0.1074 | +21.6% |
| Win Rate | 3/10 (30%) | 7/10 (70%) | — |

---

| Field | Value |
|---|---|
| Date | 2026-05-06 |
| Model | `checkpoint_td3_v2_200000_steps.zip` |
| Episodes | 10 |
| Friction (mu) | 0.6 |
| TCP Lissajous | true |

## Results

| Metric | RL | LQR | Improvement |
|---|---|---|---|
| Reward (avg) | +729.606 ± 42.982 | +739.092 ± 53.638 | -1.3% |
| RMSE Error (m) | 0.1920 ± 0.0540 | 0.2162 ± 0.0678 | +11.2% |
| Max Error (m) | 0.4447 ± 0.1236 | 0.4509 ± 0.1154 | +1.4% |
| Win Rate | 6/10 (60%) | 4/10 (40%) | — |

---

| Field | Value |
|---|---|
| Date | 2026-05-06 |
| Model | `checkpoint_td3_v2_200000_steps.zip` |
| Episodes | 10 |
| Friction (mu) | 0.8 |
| TCP Lissajous | true |

## Results

| Metric | RL | LQR | Improvement |
|---|---|---|---|
| Reward (avg) | +727.864 ± 28.267 | +726.456 ± 70.251 | +0.2% |
| RMSE Error (m) | 0.2131 ± 0.0487 | 0.2032 ± 0.0368 | -4.9% |
| Max Error (m) | 0.4028 ± 0.0936 | 0.4330 ± 0.0986 | +7.0% |
| Win Rate | 5/10 (50%) | 5/10 (50%) | — |

---

| Field | Value |
|---|---|
| Date | 2026-05-06 |
| Model | `checkpoint_td3_v2_200000_steps.zip` |
| Episodes | 10 |
| Friction (mu) | 1.0 |
| TCP Lissajous | true |

## Results

| Metric | RL | LQR | Improvement |
|---|---|---|---|
| Reward (avg) | +728.593 ± 23.067 | +722.007 ± 29.237 | +0.9% |
| RMSE Error (m) | 0.1826 ± 0.0448 | 0.1811 ± 0.0454 | -0.8% |
| Max Error (m) | 0.4087 ± 0.0711 | 0.3873 ± 0.0663 | -5.5% |
| Win Rate | 7/10 (70%) | 3/10 (30%) | — |
