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
* **Why Uniform CCR Caused $13.5\text{ Hz}$ Roll Clonus:** Applying pitch-tuned CCR weights to roll-dominant muscles excited a $13.5\text{ Hz}$ lateral oscillation. Weighting the extensor/flexor balance ($w_{\text{ext}} = 0.70$, $w_{\text{flex}} = 1.15$) by each muscle's directional pitch-dominance fraction:
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
