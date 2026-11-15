# Learned Lessons Along the Way: Surgical Bode Plot Manipulation & Multi-Axis Tuning Guide

This document records the physical principles, frequency-band ownership maps, and surgical parameter adjustments discovered while calibrating the 72-muscle, 6-DOF `HeadNeckNeuralController` against human experimental frequency-response data (Happee et al., 2017, 2023). 

Because the 72 Hill-type muscles are anatomically cross-coupled across Pitch, Roll, and Yaw, understanding **which parameter controls which part of the Bode plot** allows us to tune Roll and Yaw (and modulate Volition) **surgically by hand**, without needing blind CMA-ES searches.

---

## 1. Master Frequency-Band Ownership Map (0.15 Hz – 8.0 Hz)

Different neural subsystems dominate distinct frequency bands of the head-in-space / trunk-perturbation Bode plot. When a specific frequency region shows a gain or phase discrepancy, only the subsystem owning that band should be adjusted:

```
Frequency (Hz):  0.15 -------- 0.60 -------- 1.10 -------- 2.00 -------- 4.50 -------- 8.00 Hz
                  |             |             |             |             |             |
Dominant Loop:    [ Cortical PID + Tonic ]    [ Spinal CCR  ]   [ Canal VCR   ]   [ VCR Lead Zero ]
                  [ & Phasic Otolith LPF ]    [ kp_prop     ]   [ K_sc & p_sc ]   [ z_sc & z_phas ]
                  |             |             |             |             |             |
Primary Effect:   DC hold &     Velocity-     Mid-band knee     Mid-to-high       High-freq phase
                  low-freq      storage       (1.099 Hz         head-in-space     lead without
                  stiffness     corner        damping clamp)    velocity brake    gain explosion
```

| Frequency Band | Target Frequencies (Hz) | Controlling Parameters | What Increasing the Parameter Does to the Bode Plot |
| :--- | :--- | :--- | :--- |
| **DC & Ultra-Low**<br>*(0.15 – 0.6 Hz)* | `0.312`, `0.506` | • $K_{\text{ton}}$ & $p_{\text{ton}}$ ($16.23\text{ rad/s}$)<br>• $K_{p,\text{task}}$, $K_{i,\text{task}}$ | • **Increases head-in-space hold** (reduces head rotation in space; increases relative head-to-trunk counter-rotation).<br>• Advances low-frequency phase ($0.3\text{ Hz}$).<br>• $K_{i,\text{task}}$ eliminates slow postural drift (keeps mean angle at $0^\circ$). |
| **Low-to-Mid Transition**<br>*(0.6 – 1.0 Hz)* | `0.735` | • $K_{\text{phas}}$ ($15.42\text{ N}\cdot\text{m/(m/s}^2\text{)}$)<br>• $p_{\text{phas}}$ ($4.71\text{ rad/s} \approx 0.75\text{ Hz}$) | • Sets the otolith **velocity-storage corner** ($f_c = p_{\text{phas}} / 2\pi \approx 0.75\text{ Hz}$).<br>• Raising $K_{\text{phas}}$ boosts counter-rotation against linear trunk acceleration in the $0.5\text{–}1.5\text{ Hz}$ band. |
| **Mid-Band "Knee"**<br>*(1.0 – 2.0 Hz)* | **`1.099`**, `1.646` | • **$k_{p,\text{prop}}$** (`kp_proprioception`)<br>• $K_{d,\text{task}}$ | • **$k_{p,\text{prop}}$ acts as a head-on-trunk damper/clamp.**<br>• **Increasing $k_{p,\text{prop}}$** pins the head to the trunk, **pulling DOWN the $1.099\text{ Hz}$ gain** (overdamping the knee).<br>• **Decreasing $k_{p,\text{prop}}$** releases the neck to respond dynamically, **lifting the $1.099\text{ Hz}$ gain UP** toward the experimental curve! |
| **Mid-to-High Band**<br>*(2.0 – 5.0 Hz)* | `2.423`, `3.716` | • **$K_{\text{sc}}$** ($4.10\text{ N}\cdot\text{m/(rad/s)}$)<br>• **$p_{\text{sc}}$** ($53.28\text{ rad/s} \approx 8.48\text{ Hz}$) | • Semicircular canals sense angular velocity in space ($\omega$).<br>• Increasing $K_{\text{sc}}$ damps head-in-space angular velocity across $1.5\text{–}5.5\text{ Hz}$.<br>• Raising $p_{\text{sc}}$ pushes the canal filter's $-45^\circ$ phase lag corner to higher frequencies. |
| **High-Frequency Band**<br>*(5.0 – 8.0 Hz)* | **`5.583`**, **`7.927`** | • **$z_{\text{sc}}$** ($5.544\text{ ms}$)<br>• **$z_{\text{phas}}$** ($2.40\,z_{\text{sc}} = 13.31\text{ ms}$)<br>• Strict properness ($D = 0$) | • **Pure High-Frequency Phase Knob:** Tuning $z_{\text{sc}}$ injects phase lead $\Delta\phi(\omega) = \arctan(\omega\,z_{\text{sc}})$ at $5.5\text{–}8.0\text{ Hz}$ while leaving gain below $2\text{ Hz}$ completely untouched! |

---

## 2. Four Major Surgical Discoveries (And Why They Worked)

### Lesson 1: Why Improper Delay Compensators Exploded the 5.5 Hz & 7.9 Hz Gain ("Properness Rule")
* **What Happened:** Early versions used a 2nd-order Taylor delay compensator $C(s) = 1 + \tau s + \tfrac{1}{2}\tau^2 s^2$ (or biproper feedthrough $D \neq 0$) on the VCR output before the spinal QP muscle solver. At $5.58\text{ Hz}$ ($\omega = 35.1\text{ rad/s}$) and $7.93\text{ Hz}$ ($\omega = 49.8\text{ rad/s}$), the $\omega^2$ term amplified high-frequency acceleration spikes.
* **The Nonlinear Rectification Trap:** Because the 72 muscles are unidirectional (pull-only, $u_i \ge u_{\min,i}$), high-frequency oscillatory torque spikes fed into the QP solver get **half-wave rectified** into a massive **DC co-contraction bias** and resonant high-frequency whip, sending the $5.5\text{ Hz}$ and $7.9\text{ Hz}$ Bode gain through the roof.
* **The Fix (Strict Properness, $D = 0$):** Every reflex filter ($H_{\text{ton}}, H_{\text{sc}}, H_{\text{phas}}, H_{\text{ccr}}$) must be realized as a **strictly proper transfer function** (numerator degree $<$ denominator degree, relative degree $\ge 1$, direct feedthrough $D = 0$):
  $$H_{\text{sc}}(s) = K_{\text{sc}}\,\frac{p_{\text{sc}}\,p_{\text{fast}}\,(1 + z_{\text{sc}} s)}{(s + p_{\text{sc}})(s + p_{\text{fast}})}, \qquad x_{\text{sc,eff}} = x_{1,\text{sc}} + z_{\text{sc}} x_{2,\text{sc}}$$
  Because $x_{1,\text{sc}}$ and $x_{2,\text{sc}}$ are integrated ODE states, $x_{\text{sc,eff}}$ has **zero instantaneous feedthrough** of sensor spikes, and $|H_{\text{sc}}(j\omega)|$ rolls off as $1/\omega$ at high frequencies. This immediately eliminated the $5.5\text{ Hz}$ and $7.9\text{ Hz}$ gain explosion and prevented DC drift.

---

### Lesson 2: Surgical Fix for the 1.099 Hz Gain Drop (Relaxing $k_{p,\text{prop}}$)
* **The Problem:** In Adaptation Run 1, the Bode gain matched well everywhere except at **$1.099\text{ Hz}$**, where it drooped to $242.8\text{ deg/m}$ (**$-2.39\text{ dB}$** below Happee's $319.6\text{ deg/m}$).
* **Diagnosis:** Why only $1.099\text{ Hz}$? Around $1.1\text{ Hz}$ is the mechanical transition between quasi-static posture hold ($<0.6\text{ Hz}$) and head-in-space inertial stabilization ($>2\text{ Hz}$). The spinal Cervico-Collic Reflex (CCR) senses muscle stretch velocity relative to the trunk. When $k_{p,\text{prop}} = 0.5508$ was too stiff, it acted like a viscous neck brace at $1.1\text{ Hz}$, overdamping the head's pitch response.
* **Surgical Manipulation:**
  * We lowered **`kp_proprioception` from $0.5508 \to 0.4435$** ($-19.5\%$) and slightly relaxed **$K_{\text{sc}}$ from $4.39 \to 4.10$**.
  * **Result:** The $1.099\text{ Hz}$ gain rose from **$242.8 \to 285.3\text{ deg/m}$** (**$-2.39\text{ dB} \to -0.99\text{ dB}$**), while $0.312\text{ Hz}$, $0.506\text{ Hz}$, $0.735\text{ Hz}$, and $1.646\text{ Hz}$ stayed locked within $\pm 0.5\text{ dB}$!

---

### Lesson 3: Surgical Fix for High-Frequency Phase Lag (Tuning Zero $z_{\text{sc}}$)
* **The Problem:** After enforcing strict properness ($D = 0$) and lifting the $1.099\text{ Hz}$ gain, the high-frequency phase at **$7.927\text{ Hz}$** lagged behind the human data by **$28.4^\circ$** (and $5.583\text{ Hz}$ lagged by $27.9^\circ$).
* **Diagnosis:** We needed $+15^\circ \text{ to } +25^\circ$ of phase lead at $6\text{–}8\text{ Hz}$ **without** increasing high-frequency gain and **without** shifting low/mid-frequency phase ($0.3\text{–}2.4\text{ Hz}$).
* **Surgical Manipulation:**
  * A numerator zero $(1 + z_{\text{sc}} s)$ inside the strictly proper 2nd-order filter adds phase lead:
    $$\Delta \phi(\omega) = \arctan(\omega\,z_{\text{sc}})$$
    and multiplies gain by only:
    $$\Delta G(\omega) = \sqrt{1 + (\omega\,z_{\text{sc}})^2}$$
  * Look at the math when we increased **$z_{\text{sc}}$ from $2.5\text{ ms} \to 5.544\text{ ms}$** (and $z_{\text{phas}} = 2.40\,z_{\text{sc}} = 13.31\text{ ms}$):
    * **At $1.099\text{ Hz}$ ($\omega = 6.91\text{ rad/s}$):** $\omega z_{\text{sc}} = 0.0383 \implies \Delta\phi = +2.2^\circ$, $\Delta G = \sqrt{1 + 0.0383^2} = 1.0007$ (**$+0.006\text{ dB}$** — virtually zero effect!).
    * **At $7.927\text{ Hz}$ ($\omega = 49.81\text{ rad/s}$):** $\omega z_{\text{sc}} = 0.276 \implies \Delta\phi = +15.4^\circ$, while $\Delta G = \sqrt{1 + 0.276^2} = 1.037$ (**only $+0.32\text{ dB}$** gain increase!).
  * Combined with nudging $p_{\text{sc}}$ from $50.0 \to 53.28\text{ rad/s}$, the **$7.927\text{ Hz}$ phase error dropped from $28.4^\circ \to 3.8^\circ$** (and $5.583\text{ Hz}$ dropped to $16.9^\circ$)!

---

### Lesson 4: Directional Muscle Dominance Weighting ($\eta_{\text{axis},i}$) & Why Cross-Coupling Helps Us
* **The Anatomical Reality:** In the 72-muscle HYOID model:
  1. **Pitch Asymmetry:** 40 extensors have $49.45\text{ N}\cdot\text{m}$ pitch capacity vs. 24 flexors with $15.98\text{ N}\cdot\text{m}$ ($3.09:1$ ratio).
  2. **Roll & Yaw Symmetry:** Left and right muscle groups are mirror-symmetric ($1:1$ ratio).
  3. **Multi-Axis Sharing:** Sternocleidomastoid, splenius capitis, scalenes, and semispinalis generate moments across **Pitch, Roll, and Yaw simultaneously**.
* **Why Uniform CCR Caused $13.5\text{ Hz}$ Roll Clonus:** Applying pitch-tuned CCR weights to roll-dominant muscles excited a $13.5\text{ Hz}$ lateral oscillation. Weighting the extensor/flexor balance ($w_{\text{ext}} = 0.635$, $w_{\text{flex}} = 1.275$) by each muscle's directional pitch-dominance fraction:
  $$\eta_{\text{pitch},i} = \frac{r_{\text{pitch},i}^2}{r_{\text{pitch},i}^2 + r_{\text{roll},i}^2 + r_{\text{yaw},i}^2 + 10^{-8}}$$
  neutralized the $3.09:1$ pitch extensor bias while leaving roll- and yaw-dominant muscles at clean unit balance ($w_{\text{ccr},i} \approx 1.0$).

---

## 3. Step-by-Step Playbook for Manual Roll & Yaw Tuning (No CMA-ES Needed)

Because our 72-muscle spindle/CCR architecture (`kp_proprioception = 0.4435`, `p_ccr = 99.92 rad/s`, `K_gamma_dyn = 1.0`, `K_gamma_stat = 1.0`) and 3D vestibular observers are **already active across all 3 axes**, here is the exact surgical procedure when we plot the baseline Roll and Yaw Bode responses:

1. **Step 1 — Zero-Shot Baseline Plot:**
   * Apply the Roll or Yaw trunk perturbation (matching Happee's bandwidth and duration so FFT bins align with no spectral leakage) using the current converged parameter set.
2. **Step 2 — Check Low-Frequency Gain ($0.15\text{ – }0.6\text{ Hz}$):**
   * *If Roll/Yaw head-in-space rotation is too large at low frequencies (or gain in $\text{deg/deg}$ or $\text{deg/m}$ is off below $0.6\text{ Hz}$):*
   * Adjust **`K_ton` (Roll/Yaw)** or **`Kp_task_roll` / `Kp_task_yaw`**. (Note: In Yaw, gravity provides no otolith tilt signal on an Earth-vertical axis, so low-frequency Yaw stabilization relies on canal velocity integration + cortical/visual task feedback $K_{p,\text{yaw}}, K_{d,\text{yaw}}$!).
3. **Step 3 — Check the $1.0\text{ – }2.0\text{ Hz}$ Mid-Band Knee:**
   * *If the $1\text{–}2\text{ Hz}$ Roll or Yaw gain is overdamped (too low) or resonant (too high):*
   * Do **not** touch global `kp_proprioception` (which would alter our $-0.99\text{ dB}$ Pitch fit!). Instead, use directional dominance $\eta_{\text{roll},i}$ or $\eta_{\text{yaw},i}$ inside the CCR rectifier:
     $$w_{\text{ccr},i} = 1 + \eta_{\text{pitch},i}(w_{\text{pitch},i} - 1) + \eta_{\text{roll},i}(w_{\text{roll}} - 1) + \eta_{\text{yaw},i}(w_{\text{yaw}} - 1)$$
     Lowering $w_{\text{roll}} < 1.0$ raises the $1\text{–}2\text{ Hz}$ Roll gain; raising $w_{\text{roll}} > 1.0$ damps the $1\text{–}2\text{ Hz}$ Roll peak—completely orthogonally to Pitch!
4. **Step 4 — Check the $2.0\text{ – }4.5\text{ Hz}$ Canal Band:**
   * *If mid-to-high frequency gain is off:*
   * Scale axis-specific **`K_sc_roll`** or **`K_sc_yaw`** (semicircular canal velocity gain for that axis).
5. **Step 5 — Check High-Frequency Phase ($3.0\text{ – }8.0\text{ Hz}$):**
   * *If high-frequency phase lags or leads:*
   * Adjust **`z_sc`** (or axis-specific $z_{\text{sc,roll}}, z_{\text{sc,yaw}}$) using $\Delta\phi(\omega) = \arctan(\omega\,z_{\text{sc}})$.

---

## 4. Reference Summary of the Champion Pitch Calibration (Run 4, 21.52 s Pure Run)

* **Total Multi-Objective Score:** **$991.11\text{ pts}$** (`hf_penalty = 0.0`, `dc_drift_penalty = 0.0`)
* **Key Bode Metrics:** $1.099\text{ Hz}$ gain $= 285.26\text{ deg/m}$ ($-0.99\text{ dB}$), $7.927\text{ Hz}$ phase error $= 3.8^\circ$, $5.583\text{ Hz}$ phase error $= 16.9^\circ$.
* **Parameter Vector:**
  * **VCR (Sub2B):** $K_{\text{ton}} = 5.2598\text{ N}\cdot\text{m/rad}$ ($p_{\text{ton}} = 16.227\text{ rad/s}$), $K_{\text{sc}} = 4.1019\text{ N}\cdot\text{m/(rad/s)}$ ($p_{\text{sc}} = 53.284\text{ rad/s}$, $z_{\text{sc}} = 5.544\text{ ms}$), $K_{\text{phas}} = 15.4205\text{ N}\cdot\text{m/(m/s}^2\text{)}$ ($p_{\text{phas}} = 4.7119\text{ rad/s}$, $z_{\text{phas}} = 13.307\text{ ms}$).
  * **CCR & Spindles (Sub1A/1B):** $k_{p,\text{prop}} = 0.4435$, $p_{\text{ccr}} = 99.9196\text{ rad/s}$, $z_{\text{ccr}} = 0.0\text{ s}$, $K_{\gamma,\text{dyn}} = 1.0$, $K_{\gamma,\text{stat}} = 1.0$.
  * **Cortical PID (Sub4):** Pitch $(K_p = 3.7901, K_d = 0.4946, K_i = 3.7215)$, Roll $(K_p = 10.21, K_d = 1.29, K_i = 0)$, Yaw $(K_p = 0.24, K_d = 0.03, K_i = 0)$.

---

## 5. Unified Ungated 3-Axis Calibration (`Ungated_W2_cp_04_Y4_P15`) — Classic Bode Error Scores & Findings

### Why All Regime Gating Was Removed
Earlier multi-axis iterations used regime detection (`yaw_pert_active`, `lat_pert_active`) to gate Roll/Yaw overrides off during pure sagittal perturbations. Because the human head-neck system is one coupled 72-muscle 6-DOF biomechanical plant that can experience simultaneous 3D seat perturbations *and* simultaneous Pitch+Roll+Yaw voluntary commands, **all regime gating was eliminated**. Every parameter in `logs/roll_yaw_baseline/tuned_3axis_params.json` runs unconditionally across all tasks.

Using the **Classic Thousands-Scale Bode Error Score**:
$$\text{Classic Bode Score (pts)} = \sum_{k=1}^{K}\left(100\,\Delta\text{Gain}_{\text{dB},k}^2 + \Delta\phi_{\text{deg},k}^2\right)$$
the converged ungated controller achieves:

| Benchmark | Zero-Shot Score | Tuned Classic Score | Gain / Phase Split (`pts`) | Velocity HF Noise & Correlation |
| :--- | :--- | :--- | :--- | :--- |
| **`APEO` Pitch** *(6 freqs)* | `991.11 pts` *(Run 4)* | **`1,195.22 pts`** | `G = 536.10` / `P = 659.12` | `RMS = 33.4 °/s` (`ref 31.2`), **`r = 0.931`** |
| **`LatEC` Roll** *(8 freqs)* | `27,528.90 pts` | **`8,147.22 pts`** (**`-70.4%`**) | `G = 3,719.65` / `P = 4,427.57` | Noise **`5.90 -> 4.90 deg/s`**, **`r = 0.545 -> 0.735`** |
| **`YawMA` Yaw** *(9 freqs)* | `155,917.55 pts` | **`7,273.95 pts`** (**`-95.3%`**) | `G = 3,733.60` / `P = 3,540.35` | Noise **`11.49 -> 1.93 deg/s`** (**`5.95×`**), **`r = 0.148 -> 0.876`** |

### Lesson 5: Fixing `LatEC` Low-Frequency Phase (`0.12–0.32 Hz`) & `3.99 Hz` Gain Without Hurting Pitch/Yaw
* **Relieving Low-Frequency Somatogravic Lock (`K_ton_roll = -0.85`, `Ki_roll = 1.50`):** At `0.1209` and `0.1986 Hz`, the baseline (`Kp_roll = 10.21`, `K_ton_roll = 5.26`) held the head rigidly upright (`-190.7°` vs. Happee's `-304.4°`, **`+113.7°`** error). Switching Roll to **`Kp_roll = 1.40, Kd_roll = 1.14, Ki_roll = 1.50`** and **`K_ton_roll = -0.85, p_ton_roll = 3.0 rad/s`** accounts for inverted-pendulum lateral mechanics, pulling `0.1209 Hz` phase error down from **`+113.7° -> +24.2°`**, `0.1986 Hz` from **`+81.3° -> +58.9°`**, and `0.3214 Hz` from **`+35.0° -> +13.7°`**.
* **Lifting `3.9915 Hz` Roll Gain & Keeping HF Velocity Clean (`deriv_comp_roll = 0.0`):** Removing 1st/2nd-order Taylor derivative amplification on Roll (`deriv_comp_roll = 0.0` on Branches `4C`, `3B`, `5`) while tuning **`K_sc_roll = 2.029 (p_sc = 60.0, z_sc = 0)`**, **`K_phas_roll = 16.653 (p_phas = 5.747, z_phas = 0)`**, and **`Kp_prop_roll = 4.20 (z_ccr_roll = 0.10 s)`** lifted `3.9915 Hz` Roll gain by **`+3.20 dB`** (`254.23 -> 367.71 deg/m`, `-5.78 -> -2.58 dB`) while lowering Roll HF noise (`5.90 -> 4.90 deg/s`).

### Lesson 6: Voluntary-Trajectory-Referenced Roll & Yaw Task CCR (`YawMA` & Voluntary Compatibility)
* **Problem with Zero-Referenced Task CCR:** A Roll/Yaw CCR pulling back toward `q_neck = 0` fights intentional voluntary head turns.
* **C++ Solution (`HeadNeckNeuralController.cpp` lines 3043–3074):** Both Roll and Yaw Task CCR are always active (`Kp_prop_roll = 4.20, z_ccr_roll = 0.10 s`; `Kp_prop_yaw = 8.50, z_ccr_yaw = 0.06 s`) and track the error relative to the delayed **Branch 4C (`14.5 ms`) planned voluntary trajectory**:
  $$\tau_{\text{CCR},a} = -K_{p,\text{prop},a}\Bigl((\hat{q}_{\text{neck},a} - x_{1,4C,a}) + z_{\text{ccr},a}(\dot{\hat{q}}_{\text{neck},a} - x_{2,4C,a})\Bigr)$$
  using un-jerked `0th-order` delayed `raw_neck_relative_angle/speed` (`Branch 1A→1B`, `5.5 ms`). Pairing this with **`Kp_yaw = 0.24, Kd_yaw = 0.03`**, **`K_ton_yaw = 5.2598 (16.227 rad/s)`**, **`K_sc_yaw = 1.20 (3.50 rad/s)`**, **`K_phas_yaw = 0`**, and **`deriv_comp_yaw = 0.25`** lifts collapsed HF `YawMA` gains (`[0.48, 0.23, 0.15, 0.08] -> [1.06, 1.20, 1.10, 0.92] deg/deg`) and cuts HF noise by **`5.95×`** (`11.49 -> 1.93 deg/s`, `r = 0.876`).

### Verified Per-Frequency Comparison (`logs/roll_yaw_baseline/tuned_3axis_summary.json`)
* **`LatEC` Roll (`8` freqs, Zero-Shot `27,528.90` -> Tuned `8,147.22 pts`):**
  * `0.1209 Hz`: Gain `10.71 (+1.69 dB) -> 6.36 (-2.83 dB)` *(tgt `8.81`)* | Phase `-190.7° (+113.7°) -> -280.2° (+24.2°)` *(tgt `-304.4°`)*
  * `0.1986 Hz`: Gain `13.36 (-3.49 dB) -> 23.79 (+1.53 dB)` *(tgt `19.96`)* | Phase `-199.5° (+81.3°) -> -221.8° (+58.9°)` *(tgt `-280.7°`)*
  * `0.3214 Hz`: Gain `29.53 (-3.05 dB) -> 30.73 (-2.71 dB)` *(tgt `41.97`)* | Phase `-222.2° (+35.0°) -> -243.4° (+13.7°)` *(tgt `-257.1°`)*
  * `0.5588–2.0936 Hz`: Gains `70.89 (-2.12 dB)`, `214.36 (+0.99 dB)`, `512.08 (+1.89 dB)`, `446.99 (+1.95 dB)` | Phases within **`-3.8°`, `+1.7°`, `-3.1°`, `-5.8°`**
  * `3.9915 Hz`: Gain `254.23 (-5.78 dB) -> 367.71 (-2.58 dB)` *(tgt `494.80`)* | Phase `-355.3° (+11.2°)` *(tgt `-366.5°`)*
* **`YawMA` Yaw (`9` freqs, Zero-Shot `155,917.55` -> Tuned `7,273.95 pts`):**
  * `0.2016–0.9651 Hz`: Gains `0.74 (+2.08 dB)`, `0.76 (+1.56 dB)`, `0.76 (-0.16 dB)`, `0.80 (-0.37 dB)`, `0.88 (-2.16 dB)` | Phase errors `+1.2°` to `-11.3°` *(vs `-52.0°` zero-shot)*
  * `1.4081–4.0528 Hz`: Gains `1.06 (-2.65 dB)`, `1.20 (-1.62 dB)`, `1.10 (-3.16 dB)`, `0.92 (-2.47 dB)` *(vs `0.08 / -24.10 dB` zero-shot)* | Phases `-1.7° (+7.6°)`, `-17.9° (+14.0°)`, `-39.9° (+25.2°)`, `-52.4° (+47.6°)`
* **`APEO` Pitch Check (`6` freqs, Ungated `1,195.22 pts`, Run 4 baseline `991.11 pts`):**
  * `0.324 Hz`: `79.63 (-0.72 dB)` / `-24.6° (+17.6°)` | `0.783 Hz`: `217.14 (-0.76 dB)` / `-95.7° (+1.2°)` | `1.099 Hz`: `263.01 (-1.70 dB)` / `-118.0° (+8.8°)`
  * `2.446 Hz`: `457.13 (+0.67 dB)` / `-159.4° (+2.0°)` | `5.583 Hz`: `562.05 (+0.96 dB)` / `+142.5° (+16.1°)` | `7.927 Hz`: `563.49 (-0.14 dB)` / `+109.6° (+2.8°)`

---

## 6. Complete C++ Delay Compensator Time Constants & Orders Audit (`HeadNeckNeuralController.cpp`)

To eliminate HF derivative amplification on non-sagittal axes and preserve clean stability across all feedback loops, `HeadNeckNeuralController.cpp` uses **exact mixed compensator orders** (with per-axis scaling `d_comp = [1.0 Pitch, 0.0 Roll, 0.25 Yaw]`):

| Pathway / Branch | C++ Delay (`ms`) | Compensated Signal(s) | Exact Order in C++ |
| :--- | :--- | :--- | :--- |
| **`2A → 2B` (CN VIII → VN)** | `2.0 ms` (`tau_2a`) | `ω̂_irr` (`canals`), `f̂_irr` (`otolith`) | **Full 2nd-order** (`1 + τs + 0.5τ²s²`) |
| **`2A → 3` (CN VIII → Cerebellum)** | `3.0 ms` (`tau_2b`) | `ω̂_Luen` (`reg`), `f̂_Luen` (`reg`) / `α̂_Luen` | **Full 2nd-order** (`ω̂`, `f̂`) / **1st-order** (`α̂_Luen`) |
| **`1A` Fusimotor Lead (Spindles)** | `τ_s,sp = 205 ms` | Dynamic/static `γ` bias derivative | **1st-order** (`1 + τ_s,sp·s`, zero `0.5τ²s²`) |
| **`1A → 1B` (DRG → Spinal CCR)** | `5.5 ms` (`tau_1a`) | `ΔIa[72]` / `raw_neck_rel (R/Y CCR)` | **Full 2nd-order** (`ΔIa[72]`) / **0th-order** (`raw_neck`) |
| **`1A → 3` (Cuneate → Cerebellum)** | **`7.0 ms`** (`delay_smith_proprio`) | `spindle/nk` / `trunk_ori` / `trunk_vel` | **2nd-order** (`sp/nk`) / **1st-order** (`θ_t`) / **0th-order** (`ω_t`) |
| **`3A` (Uncinate → `2B` VN)** | `1.5 ms` (`tau_3a`) | `R̂_H^W` (`SO(3)`), `g̃_H`, `Δθ̂_head` | **Full 2nd-order** |
| **`3B` (Thalamocortical → `4` M1)** | `13.0 ms` (`tau_3b`) | `θ̂_head`, `ω̂_head` | **`d_comp = [1.0, 0.0, 0.25]`** *(P: 2nd, R: 0th, Y: 0.25×2nd)* |
| **`4A` (Pontine → `3` Cerebellum)** | `9.0 ms` (`tau_4a`) | Planned `x₁` | **Full 2nd-order** (unscaled) |
| **`4B` (Bulbar → `2B`/`3` Eff. Copy)** | `10.0 ms` (`tau_4b`) | Planned `x₂, a` / `a_comp_4B` (`jerk` term dropped) | **2nd-order** (`x₂, a`) / **1st-order** (`a_comp_4B`) |
| **`4C` (CST → `5` Spinal QP & `1B`)** | `14.5 ms` (`tau_4c`) | `τ_vol` (`LPF`) / Planned `x₁, x₂` | **`d_comp` on `τ_vol`** / **2nd-order** (`x₁, x₂` traj) |
| **`2B → 5` (MVST → `5` Spinal QP)** | `2.5 ms` (`tau_5`) | `τ_VCR` (using `d_phas/sc` from strictly proper filters) | **`d_comp = [1.0, 0.0, 0.25]`** *(P: 2nd, R: 0th, Y: 0.25×2nd)* |
| **`1B → 5` (Propriospinal → `5` QP)** | `4.0 ms` (`tau_6`) | `y_ccr[72]` (`z_ccr = 0.0 s`) | **Full 2nd-order** |
| **Post-QP & Post-RI** *(internal)* | `τ_act = 10.0 ms` | Muscle activation dynamics inverse | **Full 2nd-order** (`1 + τ_act·s + 0.5τ_act²s²`) |
| **`5A` (α-MN → Plant)** | `5.5 ms` (`tau_5a`) | `u_α[72]` (clamped `[0, 1]`) | **Full 2nd-order** |
| **`5B` (γ-MN → Spindles/`3`)** | `8.0 ms` (`tau_5b`) | `f_dyn/stat[72]` (`LPF` states) | **Full 2nd-order** |

---

## 7. Architectural & Neurophysiological Comparison: Present Unified 3-Axis Controller vs. Happee et al. (2023)

This section provides a direct, code- and publication-verified comparison between the present unified 3-axis neuromusculoskeletal controller and the benchmark **Happee, Kotian & De Winkel (2023)** (*Frontiers in Neurology*, `10.3389/fneur.2023.1266345`) head-neck stabilization model, intended for thesis committee review.

### 7.1 Dimension-by-Dimension Comparison

| Architectural Axis | Happee, Kotian & De Winkel (2023) *(MADYMO / MATLAB)* | Present Unified 3-Axis Controller *(OpenSim 4.6 / C++)* | Engineering & Biological Advantage |
| :--- | :--- | :--- | :--- |
| **Musculoskeletal Plant & Integration** | `MADYMO 2022` (`9` rigid bodies, `8` joints = `48 DOF`, `34` muscles / `258` Hill elements); MATLAB `R2022b` `ode1` Euler `10 µs` fixed step (`~100×` real-time, *"taking days to fit experimental datasets"* — Sec. 2.1, 4.4). | `HYOID_HeadNeckNeural_Model_Detailed.osim` (`84` muscles: `72` active `Millard12EqMuscleWithAfferents` neck muscles + `12` extraocular eye muscles `r_*`/`l_*`; `17` joints; `14` passive bushings/coordinate forces) in C++ OpenSim 4.6 plugin. | Variable-step Simbody integration with hyoid/detailed cervical anatomy and embedded proprioceptive receptor states per muscle. |
| **Proprioceptors (`Ia` / `II`) & Fusimotor (`γ`) Control** | Two global scalar gains (`k_p`, `k_v`) jointly fitted for all `258` elements on simplified contractile-element length `L - L_0` and velocity `L_dot` (Sec. 3.1, Fig. 1). **No intrafusal spindle dynamics, no `Ia`/`II` separation, no fusimotor drive.** | Every one of the `72` neck muscles embeds biophysical **`Mileusnic06Spindle`** (`bag1`/`bag2`/`chain`, `Ia` & `II` firing) + `Lin02GolgiTendonOrgan`. Driven by a **Two-Stage Fusimotor Controller** (`γ_s` quasi-static inverse + `205 ms` lead; `γ_d` velocity-gated), per-muscle **Luenberger observers** (`55 ms / 10 ms / 5 ms`), `κ(f_hat_d)` sensitivity normalization, and a real-time `3×72` (via `6×72` quadratic Taylor `J(q)`) pseudo-inverse decoder. | Replaces direct internal-state `L - L_0` tapping with genuine intrafusal biomechanics and sensory decoding, avoiding non-physiological strain-rate access. |
| **Vestibular Afferents & Sensory Nonlinearities** | Regular & irregular canals lumped into one linear 2nd-order SCC transfer function for direct feedback; MSOM (`τ_scc = 5.7 s` 1st-order) and SVC (`τ_scc = 7 s` 2nd-order, `τ = 5 s` LP) explicitly state ***"while otolith and visual dynamics and delays are ignored"*** (Sec. 2.2) and omit sensory thresholds (Sec. 4.4). | Explicit bilateral **regular AND irregular** 2nd-order semicircular canals (`6` canal projections, `G_pass,reg = 0.436629`, `G_pass,irr = 0.541800`) AND explicit **2-stage regular (`k0 = 37.4, k = 66.0 imp/s/g, c = 1.0 s, τ1 = 27.6 ms, b = 17.5 ms`) and irregular (`k = 245.0, c = 0.25 s, τ1 = 27.2 ms, b = 7.0 ms`) otolith maculae** with smooth physiological deadzones (`vest_tilt_thresh = 0.002`, `vest_scc_thresh = 0.005`, `vest_oto_thresh = 0.020`). | Models both tonic/phasic vestibular channels plus sensory thresholds directly from bilateral receptor afferents (`Schneider15VestibularAfferent`). |
| **Spatial Orientation & Yaw Integration** | Head yaw angle computed by **open-loop 3D integration** of yaw rate (explicitly acknowledged as ***"overly simplistic ... will result in inaccuracy and drift"***, Sec. 2.2, 4.1). | Minimum-phase **Luenberger canal/otolith inverse observers** feed an **`SE_2(3)` / Mahony spatial quaternion observer (`SO(3)`)** with **Smith-Predictor delay alignment** (`τ_2b = 3.0 ms` CN VIII vs. `τ_3b = 7.0 ms` cuneate proprioception), fusing otolith gravity (`Kp = 0.20 rad/s`) and 6-DOF proprioceptive neck+trunk attitude (`Kp = 2.50 rad/s`). | Eliminates open-loop yaw integration drift whenever trunk proprioception is present and preserves temporal alignment across sensory modalities. |
| **Axonal Conduction Delays & Filter Realizability** | Two lumped bulk delays (`τ_vcr = 13 ms`, `τ_ccr = 13 ms`) plus a single lumped 1st-order LPF `H_ang` whose time constant `τ_ang` is manually switched across tasks (`30 ms` Pitch/slalom, `100 ms` AP/Lat/Roll, `150 ms` YawMA, `300 ms` YawNV, `400 ms` YawVS — Sec. 3.1–3.3). | **`13` anatomically grounded axonal branches (`1.5 ms`–`14.5 ms`)** plus `τ_act = 10.0 ms` muscle activation inverse, strictly proper **2nd-order VCR/CCR filters (`p_fast = 250 rad/s`, zero direct feedthrough `D = 0`)**, and axis-specific 2nd-order Taylor derivative compensation `d_comp = [1.0, 0.0, 0.25]`. | Respects distributed ascending/descending loop delays without artificially lumping sensory and effector latencies or tuning `H_ang` per trial. |
| **Synergy Recruitment & Co-contraction** | **Fixed scalar muscle synergy vectors** (`Na_flex-r`, `Na_flex-t`, `Na_ext-r`, `Na_ext-t`, `Na_CC`, `Na_post`) plus manually switched static co-contraction (`~1%` perturbations vs. `4%` slalom — Sec. 3.1, 4.1). | Real-time **15-iteration Active-Set Quadratic Program (QP)** (`min Σ(δu_i^2)` on instantaneous `J(q)` in 3-DOF or 6-DOF with analytical `u_dot`/`u_ddot` derivatives). | Configuration-dependent torque-to-muscle allocation adapts naturally as neck posture changes; removes hand-picked static `Na` vectors. |
| **Task Generalization (Parameter Sharing)** | **Per-condition parameter refitting**: ***"For each condition and model version, the postural control parameters were estimated by fitting the model to the experimental data"*** (Sec. 3 p. 7, Sec. 4.2 p. 18). Pitch angle gain shifts `APEO: 0.5 → PitchMA: 1.9 → PitchNV: 4.8 → PitchVS: 7.1 → slalom: 5.2`; roll gain shifts `LatEC: 0.6 → RollEC: 1.3 → slalom: 6.5`; Table 3 normalizes errors to an idealized *"perfect angle perception"* loop. | **Single ungated 3-axis parameter set (`Ungated_W2_cp_04_Y4_P15`)** with zero regime switching or per-trial gain scheduling, evaluated simultaneously across **Pitch (`APEO = 1,195.22 pts`)**, **Roll (`LatEC = 8,147.22 pts`)**, and **Yaw (`YawMA = 7,273.95 pts`)**. | One fixed nervous system parameterization replicates all 3 reflex axes without test-specific gain switching. |
| **Voluntary–Reflex Coexistence & Ocular Readiness** | No internal forward-model cancellation loop; voluntary tracking (`PitchVS`/`YawVS`) swaps external feedback gains or modulates `L_0` (`Fig. 1`). Footnote 1 (p. 2) excludes eye movements (`VOR`). | Unified simultaneous 3-axis voluntary command (`desired_pitch/roll/yaw` → 2nd-order `trajectory_tau` planner → M1 PID + 2 Hz LPF) coexists with reflexes via **Sub3(C) Cerebellar Efference Cancellation** (tonic tilt, canal velocity, pivot `Sub2B`), **Parallel Reference Spindle `Ia`/`II` Cancellation**, **Voluntary-Referenced Roll/Yaw CCR**, and **Voluntary-Gated Reciprocal Inhibition (`K_recip_inhib`)**. `12` extraocular eye muscles + `vestibular_omega` output are embedded in the same `.osim` / C++ stack (locked during reflex-only validation). | Structurally ready for simultaneous 3-axis voluntary head-neck movements without reflexes fighting self-generated motion or needing to replace the reflex architecture. |

### 7.2 Balanced Scientific Framing & Scope Boundaries for Jury Defense

To ensure thesis committee rigor, both architectural advances and current validation boundaries should be stated clearly:

1. **Cleaner, hack-free neuro-architecture**: Happee et al. (2023) achieve multi-task fits by retuning condition-specific feedback gains (`0.5` to `7.1` in pitch; `0.6` to `6.5` in roll), shifting lumped filter time constants (`τ_ang = 30–400 ms`), switching manual static co-contraction (`1%` vs. `4%`), using fixed synergy vectors `Na`, reading ideal contractile-element `L - L_0`, integrating yaw open-loop, and omitting otolith dynamics/delays in perception observers. The present framework removes every one of those simplifications in favor of biophysical receptor models (`Schneider15VestibularAfferent`, `Mileusnic06Spindle`), `13` distributed conduction delays, an `SE_2(3)`/Mahony quaternion observer + Smith Predictor, and a real-time active-set QP synergy solver under a **single ungated parameter vector**.
2. **Structural readiness for voluntary control**: Because `Sub3(C)` cerebellar efference cancellation, the **Parallel Reference Mileusnic Spindle**, voluntary-referenced roll/yaw CCR, and voluntary-gated `K_recip_inhib` already reside cleanly inside `HeadNeckNeuralController.cpp`, transitioning from passive reflex validation to **simultaneous 3-axis voluntary movements** requires tuning the voluntary M1/planner parameters rather than bolting on task-switching heuristics or disabling reflexes.
3. **Honest scope acknowledgment**: Happee et al. (2023) validated **`12` experimental conditions** — including visual (`VR+VV`) vs. non-visual (`NV`) loops, compliant seat (`XYZ compliant`), and `4 m/s^2` slalom motion-sickness perception (`MSOM`/`SVC`) — whereas our present benchmark validates **`3` core multisine reflex datasets** (`APEO` Pitch, `LatEC` Roll, `YawMA` Yaw) with the `12` eye muscles locked and `FixationController` disabled, and uses distinct per-axis VCR/CCR filter parameters within one shared, ungated parameterization. Future work will extend this same unified architecture cleanly to multi-axis voluntary trajectories and visual/motion-sickness scenarios.

