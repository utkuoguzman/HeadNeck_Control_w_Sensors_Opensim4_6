# Phase 2.4 Master Optimization & Verification Ledger

## 1. Architectural & Numerical Configuration Verified
- **72-Muscle Literature-Centered 6-DOF Equilibrium Co-Contraction (`u_min,i`):** Superficial Flexors `1.0–2.5% MVC`, Deep/Upper Flexors `2.0–4.0% MVC`, Superficial Extensors `2.0–5.0% MVC`, Deep Cervical Extensors `3.0–7.0% MVC` ($\ddot{\mathbf{q}}_{6\text{DOF}}(0) = \mathbf{0}$, `init_pitch_integral = 0.0`, `grav_trim_torque = 0.0`).
- **Incremental Active-Set Cholesky/LU QP Allocator:** $\Delta u_i = u_{\alpha,i} - u_{\min,i} \in [0, 1 - u_{\min,i}]$.
- **Resting Extrafusal & Intrafusal Spindle Initialization:** $a_i(0) = u_{\min,i}$ and $f_{\text{dyn},0,i}, f_{\text{stat},0,i}, f_{\text{chain},0,i}$ initialized prior to `model.equilibrateMuscles(state)` with incremental fusimotor efference subtraction $\Delta f_j = \max(0, f_j - f_{j,0})$.
- **Physiological Sensory & Synaptic Recruitment Thresholds:** Spindle $\Delta\text{Ia}_{\min} = 0.5\,\text{imp/s}$, Canal $\omega_{\min,\text{scc}} = 0.005\,\text{rad/s}$, Otolith $a_{\min,\text{oto}} = 0.020\,\text{m/s}^2$, Tilt $\theta_{\min,\text{tilt}} = 0.002\,\text{rad}$.
- **Integrator Stability Guarantee (`SemiExplicitEuler2`):** `min_step = 1e-4 s` (`0.1 ms`, $|h\lambda_{\max}| = 0.167 \ll 1.0$), `max_step = 0.001 s` (`1.0 ms` < $2\tau_{2,\text{irr}} = 1.2\,\text{ms}$ stability boundary).

---
## Generation 1 — Champion `W12_P24G1` (Total Score: `37964.72 pts`, Duration: `51.4 min`)

### A. CMA-ES Search Distribution & Population Health
- **Global Step Size (`sigma`):** `1.00893` | **Covariance Condition Number (`cond(C)`):** `1.0`
- **Candidates Passing All 6 Frequencies:** `23 / 28` | **Median Score:** `62876.03` | **Mean Pass Score:** `64655.47`
- **Per-Frequency Pass Rates:** `2.45Hz`: 28/28 | `5.58Hz`: 28/28 | `7.93Hz`: 28/28 | `1.10Hz`: 28/28 | `0.78Hz`: 28/28 | `0.32Hz`: 24/28

### B. Per-Frequency Bode, 6-DOF Kinematics (`pitch`, `pitch1`, `pitch2`, `roll`, `yaw`) & Equilibrium Drift
| Freq (`Hz`) | Sim / Target Gain (`dB`) | Sim / Target Phase (`err`) | Score | Pre-Drift (`0–1.5s`) | Post `pitch` (`[min, max]`) | `pitch1` (`[-12,+18°]`) | `pitch2` (`[-24.8,+36°]`) | Max `|roll|` | Max `|yaw|` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`2.446`** | `1235.8` / `423.3` (`+9.3 dB`) | `-132.9°` / `-161.3°` (`28.4°`) | **`9464.6`** | `0.0008°` | `[-3.37°, +8.60°]` | `[+0.89°, +5.63°]` | `[-4.58°, +3.81°]` | `0.507°` | `0.407°` |
| **`5.583`** | `823.6` / `503.3` (`+4.3 dB`) | `+86.0°` / `-201.4°` (`72.6°`) | **`7103.1`** | `0.0008°` | `[-2.48°, +5.71°]` | `[+0.46°, +3.04°]` | `[-4.19°, +4.03°]` | `0.710°` | `0.548°` |
| **`7.927`** | `442.3` / `572.7` (`-2.2 dB`) | `+32.0°` / `-247.6°` (`80.3°`) | **`6956.4`** | `0.0008°` | `[-0.07°, +4.93°]` | `[-0.25°, +2.52°]` | `[-1.94°, +2.63°]` | `0.823°` | `0.983°` |
| **`1.099`** | `272.3` / `319.7` (`-1.4 dB`) | `-88.7°` / `-109.2°` (`20.6°`) | **`617.7`** | `0.0008°` | `[-9.31°, +4.41°]` | `[+0.49°, +4.25°]` | `[-10.53°, +0.73°]` | `0.676°` | `1.118°` |
| **`0.783`** | `216.4` / `236.9` (`-0.8 dB`) | `-87.8°` / `-97.0°` (`9.1°`) | **`145.0`** | `0.0008°` | `[-8.63°, +6.07°]` | `[+0.39°, +4.56°]` | `[-9.07°, +2.18°]` | `1.162°` | `1.303°` |
| **`0.324`** | `64.5` / `86.5` (`-2.6 dB`) | `-252.9°` / `-7.0°` (`114.1°`) | **`13677.9`** | `0.0008°` | `[-9.77°, +15.81°]` | `[+0.47°, +5.59°]` | `[-11.98°, +11.42°]` | `0.756°` | `1.872°` |

### C. Proprioception, Vestibular Luenberger & Mahony `SE_2(3)` Observer Tracking
| Freq (`Hz`) | Mahony Pitch RMSE (`deg`) | Mahony Pitch Corr (`r`) | Canal Luenberger RMSE (`deg/s`) | Canal Luenberger Corr (`r`) | Max Spindle `dL` (`mm`) | Max `|I_pitch|` (`rad·s`) | Max `|τ_vol,pitch|` (`N·m`) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`2.446`** | `0.756°` | `+0.9969` | `2.20` | `+0.9984` | `10.542` | `0.0459` | `1.603` |
| **`5.583`** | `0.711°` | `+0.9944` | `2.88` | `+0.9924` | `6.993` | `0.0490` | `1.746` |
| **`7.927`** | `0.487°` | `+0.9759` | `3.65` | `+0.9807` | `6.825` | `0.0447` | `1.594` |
| **`1.099`** | `1.094°` | `+0.9962` | `2.90` | `+0.9924` | `12.104` | `0.0534` | `1.892` |
| **`0.783`** | `1.240°` | `+0.9963` | `2.40` | `+0.9935` | `11.386` | `0.0453` | `1.579` |
| **`0.324`** | `1.558°` | `+0.9902` | `2.99` | `+0.9924` | `15.996` | `0.1126` | `3.953` |

### D. Champion 15D Parameters vs. CMA-ES Mean & Effective Sigma
| Parameter | Champion Value | CMA-ES Mean (`μ`) | Effective Step (`σ · std`) |
| :--- | :---: | :---: | :---: |
| **`K_ton`** | `0.3961` | `0.3412` | `0.1013` |
| **`p_ton`** | `70.3568` | `79.3544` | `10.2944` |
| **`K_sc`** | `0.2189` | `0.1954` | `0.04` |
| **`r_lead_sc`** | `0.1827` | `0.1691` | `0.0353` |
| **`z_sc`** | `17.4057` | `—` | `—` |
| **`p_sc`** | `95.2765` | `92.4157` | `12.2891` |
| **`K_phas`** | `6.3072` | `6.3841` | `0.661` |
| **`p_phas`** | `14.229` | `10.0615` | `3.0195` |
| **`kp`** | `0.1392` | `0.1901` | `0.0401` |
| **`r_lead_ccr`** | `0.0272` | `0.0301` | `0.0121` |
| **`z_ccr`** | `3.073` | `—` | `—` |
| **`p_ccr`** | `113.0644` | `145.5653` | `16.7163` |
| **`k_dyn`** | `0.9755` | `0.8337` | `0.1032` |
| **`k_stat`** | `1.0486` | `0.9671` | `0.1026` |
| **`Kp_task_pitch`** | `1.796` | `1.4599` | `0.8007` |
| **`Kd_task_pitch`** | `0.029` | `0.1124` | `0.1004` |
| **`Ki_task_pitch`** | `35.9772` | `32.2751` | `5.2397` |

