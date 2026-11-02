# Engineering, Neurophysiological & Optimization Design Rationale

> [!IMPORTANT]
> **Interactive Block Diagram & Equation Reference**  
> All block-by-block Simulink diagrams, annotated transfer functions, state-space realizations, analytical constant derivations (`[Derivation]`), and universal pseudocode now live in the interactive explorer:  
> 👉 **[`docs/neuromotor_simulink_explorer.html`](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html)**  
>
> Rather than duplicating the block equations from the HTML explorer, this document explains **why** each architectural, biophysical, numerical, and optimization design decision was made in [`HeadNeckNeuralController.cpp`](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp), [`Mileusnic06Spindle.cpp`](../proprioception_plugin/Mileusnic06Spindle.cpp), and [`run_phase2_4_campaign.py`](../scripts/run_phase2_4_campaign.py).

---

## 1. The Zero-Ground-Truth Rule & Dual-Stream Sensory Split

### 1.1 Strict Zero-Ground-Truth Constraint
Standard musculoskeletal controllers query the physics engine directly for joint coordinates (`q`), joint speeds (`u`), skull orientation in ground, or muscle lengths (`m.getLength(s)`). In our architecture, the Central Nervous System (CNS) operates under a **strict zero-ground-truth rule**:
- The controller, observers, and decoders have **zero access** to Simbody ground-truth joint angles (`pitch1..yaw2`), joint velocities, or muscle fiber kinematics.
- Every internal belief about 3D head orientation, angular velocity, linear acceleration, and neck posture is reconstructed strictly from peripheral biological mechanoreceptors:
  1. **6 Non-Orthogonal Semicircular Canals** (Regular & Irregular afferents; *Schneider et al., 2015*).
  2. **3D Otolith Maculae** (Utricle & Saccule Regular & Irregular afferents; *Schneider et al., 2015; Fernández et al., 1972; Happee et al., 2017*).
  3. **72 Intrafusal Muscle Spindles** (Primary `Ia` & Secondary `II` afferents with $\alpha\text{–}\gamma$ coactivation; *Mileusnic et al., 2006*).

### 1.2 Why Every Sensory Organ Splits into Reflex vs. Observer Streams
Biological mechanoreceptors are **not** pure kinematic sensors—evolution tuned their hair cells and intrafusal fibers to generate **phase lead** (velocity and acceleration sensitivity) so fast brainstem and spinal reflexes can compensate for axonal conduction delays ($10\text{–}15\,\text{ms}$) and Hill-type muscle excitation-contraction lag ($\sim 30\text{–}50\,\text{ms}$):
- **Fast Reflex Pathways (Subsystems 5 & 6):** Require the raw, phase-leading **Irregular** vestibular and **Primary `Ia`** spindle signals unaltered.
- **Central State Estimation (Subsystems 2, 3 & 4):** Feeding phase-leading, adaptation-distorted, or $\gamma$-biased signals into a kinematic observer corrupts 3D attitude integration. Therefore, **per-sensor Luenberger Observers** invert the peripheral dynamics of the **Regular** vestibular afferents and all **72 muscle spindles** *before* central multisensory fusion in the Mahony `SE_2(3)` Observer.

*(See **Top-Level Architecture**, **Subsystem 2**, and **Subsystem 3** in [`docs/neuromotor_simulink_explorer.html`](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) for the interactive signal routing).*

---

## 2. Qualitative & Quantitative Verification of the Mahony Observer & Its Augmentations Against Ground Truth

To validate that the CNS can accurately reconstruct 3D skull kinematics without simulator ground truth, each stage of the estimation pipeline was benchmarked against Simbody ground truth under seated harmonic horizontal perturbations ($1.11\,\text{Hz}$, steady-state window $2.40\text{–}4.20\,\text{s}$):

### 2.1 Progressive Augmentation Rationale
1. **Canal Luenberger Augmentation (`s2_luen_scc`):** Raw semicircular canal firing exhibits a $+15^\circ\text{ to }+35^\circ$ phase lead ($r = +0.5589$, $\text{RMSE} = 10.41\,\text{deg/s}$ vs. true $\boldsymbol{\omega}_{\text{head}}$). Passing the mid-band normalized Regular canal signal through its exact biproper pole-zero inverse reduces velocity RMSE by **$6.8\times$** (to $1.53\,\text{deg/s}$, $r = +0.9897$, amplitude ratio $0.999$).
2. **72-Spindle Luenberger + $6\times 72$ Cervical Jacobian Augmentation (`s3_luen` & `s3_dec`):**
   - Subtracting the descending fusimotor ($\gamma_{\text{dyn}}, \gamma_{\text{stat}}$) efference copy per muscle prevents voluntary/reflex contractions from being misinterpreted as external neck stretch.
   - Decoding across all **6 independent cervical coordinates** (`[pitch1, pitch2, roll1, roll2, yaw1, yaw2]`) via $\mathbf{K}_{6\text{DOF}} = -(J_6 J_6^\top + \lambda I_6)^{-1} J_6$ before summing upper + lower pairs eliminates **intervertebral S-shape buckling cross-talk** (where horizontal seat acceleration flexes the lower neck `pitch1` while extending the upper neck `pitch2`), achieving $r = +0.9982$ ($\text{RMSE} = 0.916^\circ$).
3. **Two-Stage Multisensory Smith Predictor (`s4_smith`) + Sync-to-Now Innovation Blocks (`s4_oto` & `s4_prop`):**
   - Vestibular afferents arrive at the brainstem at $\tau_{\text{vest}} = 2\,\text{ms}$ (CN VIII), whereas cervical proprioceptive afferents arrive at $\tau_{\text{prop}} = 7\,\text{ms}$ (dorsal columns).
   - **Stage 1 — Multisensory Smith Predictor (`s4_smith`, immediately after Inputs `1:`, `2:`, and `3:`):** Forward-predicts all three delayed sensory streams across their respective nerve delays to **present time ($t$)** using each stream's derivative-free rate/speed estimate:
     1. **Channel 1 (Canal Angular Velocity, $2\,\text{ms} \to \text{NOW } t$):** $\tilde{\boldsymbol{\omega}}_{\text{Luen}}(t) = \hat{\boldsymbol{\omega}}_{\text{Luen}}(t - 2\,\text{ms}) + \tau_{\text{vest}}\,\hat{\boldsymbol{\alpha}}_{\text{Luen}}(t - 2\,\text{ms})$, where $\hat{\boldsymbol{\alpha}}_{\text{Luen}} = \dot{\mathbf{x}}_{\text{Luen,scc}}$ is the exact state derivative of the Subsystem 2 Canal Luenberger Observer.
     2. **Channel 2 (Otolith Gravity UP Unit Vector, $2\,\text{ms} \to \text{NOW } t$):** $\tilde{\mathbf{u}}_{\text{meas}}(t) = \text{normalize}\bigl(\mathbf{u}_{\text{meas}}(t - 2\,\text{ms}) - \tau_{\text{vest}}(\boldsymbol{\omega}_{\text{head}}(t - 2\,\text{ms}) \times \mathbf{u}_{\text{meas}}(t - 2\,\text{ms}))\bigr)$ via Poisson's rotating-frame kinematic equation.
     3. **Channel 3 (Proprioceptive Head-in-Space Posture, $7\,\text{ms} \to \text{NOW } t$):** $\tilde{\boldsymbol{\theta}}_{\text{prop}}(t) = \boldsymbol{\theta}_{\text{prop}}(t - 7\,\text{ms}) + \tau_{\text{prop}}\,\hat{\boldsymbol{\omega}}_{\text{prop}}(t - 7\,\text{ms})$ using $\hat{\boldsymbol{\omega}}_{\text{prop}} = \boldsymbol{\omega}_{\text{trunk}} + \hat{\boldsymbol{\omega}}_{\text{neck}}$ (`est_neck_speed` from Subsystem 3).
   - **Stage 2 — Pure Innovation Blocks at NOW ($t$) (`s4_oto` & `s4_prop`):**
     - **Otolith Gravity Innovation (`s4_oto`):** $\mathbf{e}_{\text{oto}}(t) = \tilde{\mathbf{u}}_{\text{meas}}(t) \times \bigl(\hat{R}^\top(t)\,\mathbf{e}_y\bigr)$.
     - **Proprioceptive Attitude Innovation (`s4_prop`):** $\mathbf{e}_{\text{prop}}(t) = \hat{R}^\top(t)\bigl(\tilde{\boldsymbol{\theta}}_{\text{prop}}(t) - \hat{\boldsymbol{\theta}}_{\text{Mahony}}(t)\bigr)$.

### 2.2 Quantitative Benchmark Table (Pre-Otolith Upgrade Baseline)

| Estimator / Augmentation Stage | Simbody Ground Truth Range | CNS Reconstructed Range | RMSE | Correlation ($r$) | Amplitude Ratio |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Raw Semicircular Canal Velocity (`deg/s`)** | `[-29.84, +19.75]` | `[-28.67, +19.17]` | `10.409 deg/s` | `+0.5589` | `1.078` |
| **+ Canal Luenberger Inverse (`deg/s`)** | `[-29.84, +19.75]` | `[-29.13, +20.16]` | **`1.530 deg/s`** | **`+0.9897`** | **`0.999`** |
| **+ 72-Spindle Luenberger & 6-DOF Decoder (`deg`)** | `[-3.94°, +11.03°]` | `[-5.57°, +10.60°]` | **`0.916°`** | **`+0.9982`** | **`1.043`** |
| **+ Full Smith-Compensated Mahony Fusion (`deg`)** | `[-3.94°, +11.03°]` | `[-4.31°, +10.13°]` | **`0.719°`** | **`+0.9997`** | **`0.963`** |

> [!NOTE]
> **Scheduled Verification Update:** Following the upgrade of the Subsystem 2 Otolith organs to the 2-stage **Schneider et al. (2015) / Happee et al. (2017)** model and its exact **2-Stage Minimum-Phase Otolith Luenberger Observer** (Section 3.1 below), we will re-run the standalone ground-truth verification suite to log the updated steady-state and transient metrics across all 6 perturbation frequencies.

---

## 3. Sensory Transfer Function Modifications & Realizations

Directly implementing published transfer functions from frequency-domain neurophysiology papers in a stiff, time-delayed multibody ODE solver (`SemiExplicitEuler2`) can cause severe numerical instability if the transfer functions are improper or lack passband calibration. Below are the exact modifications we made and why.

### 3.1 Otolith Organs: Why Schneider (2015) / Happee (2017) Over Zheng (2021) & Our Biproper Realization
In the literature, two primary macaque otolith transfer function families appear in head-neck models:
1. **Zheng et al. (2021) / Fernández & Goldberg (1976 Part III):** Fitted only over $0\text{–}2\,\text{Hz}$ centrifugal force rotations using a $10\,\text{s}$ adaptation time constant ($\tau_A = 10\,\text{s}$) and a $16\,\text{ms}$ macular lag pole ($\tau_M = 0.016\,\text{s}$). Above $1.6\,\text{Hz}$, the Regular otolith transfer function acts as a low-pass filter ($\tau_B = 9\,\text{ms} < \tau_M = 16\,\text{ms}$), attenuating high-frequency head perturbations ($2.45\text{–}7.93\,\text{Hz}$).
2. **Happee et al. (2017, Appendix Eq. 3 & Fig. 9):** Fitted simultaneously to **Schneider, Jamali, Carriot, Chacron & Cullen (2015)** naturalistic macaque recordings across **$0.5\text{–}20\,\text{Hz}$** and **Fernández, Goldberg & Abend (1972)** static tilt at **$0\,\text{Hz}$**—the exact same experimental lab and frequency band ($0.3\text{–}8\,\text{Hz}$) as our semicircular canal model (*Schneider et al., 2015*).

However, Happee et al. (2017, Eq. 3) wrote the high-frequency hair-cell lead as $\dfrac{(1 + a s)^2}{1 + b s}$, which is **improper** (numerator degree 3 over denominator degree 2) and requires differentiating noisy finite-difference linear acceleration to compute linear jerk $\dot{\mathbf{a}}_{\text{lin}}$. To make this model 100% safe for time-domain ODE integration and Luenberger inversion, we formulate the otolith pathway in three stages (detailed interactively in **Subsystem 2** of [`docs/neuromotor_simulink_explorer.html`](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html)):

- **1st-Order Taylor/Padé Biproper Macular Realization (Zero Jerk Differentiation):**  
  We replace the improper quadratic numerator $(1 + a s)^2 = 1 + 2a s + a^2 s^2$ with its first-order Taylor/Padé phase-matched linear lead $(1 + \tau_1 s)$ where $\tau_1 = 2a$, yielding a strictly **biproper 2-stage cascade** (degree 2 over degree 2):
  $$H_{\text{oto}}(s) = G_{\text{pass,oto}}\,\underbrace{\left(\frac{\alpha + c s}{1 + c s}\right)}_{\text{Stage 1: Adaptation }(\alpha = k_0/k)}\,\underbrace{\left(\frac{1 + \tau_1 s}{1 + b s}\right)}_{\text{Stage 2: Biproper Lead-Lag }(\tau_1 = 2a)}$$
  - **Regular Otoliths (`s2_otolith_reg`):** $k_0 = 37.4\,\text{spk/s/g}$, $k_{\text{reg}} = 66.0\,\text{spk/s/g}$ ($\alpha_{\text{reg}} = 0.5667$), $c_{\text{reg}} = 1.0\,\text{s}$, $\tau_{1,\text{reg}} = 2a_{\text{reg}} = 27.6\,\text{ms}$, $b_{\text{reg}} = 17.5\,\text{ms}$.
  - **Irregular Otoliths (`s2_otolith_irr`):** $k_0 = 37.4\,\text{spk/s/g}$, $k_{\text{irr}} = 245.0\,\text{spk/s/g}$ ($\alpha_{\text{irr}} = 0.1527$), $c_{\text{irr}} = 0.25\,\text{s}$, $\tau_{1,\text{irr}} = 2a_{\text{irr}} = 27.2\,\text{ms}$, $b_{\text{irr}} = 7.0\,\text{ms}$.
- **Mid-Frequency Passband Normalization (`1 / G_pass,oto`, No $B^{-1}$ Required):**  
  Unlike the 6 non-orthogonal semicircular canals, the utricular and saccular maculae already span the 3 orthogonal skull axes ($X_H, Y_H, Z_H$), so $R_W^H(t) = (R_H^W(t))^\top$ requires **no $3\times 3$ spatial matrix inverse $B^{-1}$**. Instead, the Vestibular Nuclei divide the CN VIII spike rates ($\text{imp/s}$) by their SI mid-frequency passband sensitivities ($G_{\text{pass,oto,reg}} = 66.0 / 9.81 = 6.7278\,\text{(imp/s)/(m/s}^2\text{)}$ and $G_{\text{pass,oto,irr}} = 245.0 / 9.81 = 24.9745\,\text{(imp/s)/(m/s}^2\text{)}$) to recover unit-gain physical specific force ($\text{m/s}^2$) in the physiological passband.
- **Exact 2-Stage Minimum-Phase Otolith Luenberger Observer (`s2_luen_oto`):**  
  Because both zeros of $H_{\text{oto,reg}}(s)$ lie strictly in the open left-half plane ($s = -1/\tau_{1,\text{reg}} = -36.23\,\text{rad/s}$ and $s = -\alpha_{\text{reg}}/c_{\text{reg}} = -0.5667\,\text{rad/s}$), the Regular otolith is **minimum-phase**. Our 2-stage Luenberger observer inverts *both* the macular lead-lag and the adaptation operator:
  $$H_{\text{Luen,oto}}(s) = \left(\frac{1 + c_{\text{reg}} s}{\alpha_{\text{reg}} + c_{\text{reg}} s}\right)\left(\frac{1 + b_{\text{reg}} s}{1 + \tau_{1,\text{reg}} s}\right) \implies \tilde{H}_{\text{oto,reg}}(s)\,H_{\text{Luen,oto}}(s) \equiv 1$$
  This restores the full unattenuated **$1.0g$ ($9.81\,\text{m/s}^2$) static gravity vector at DC ($\omega = 0$)** for the Mahony Observer, with all 18 otolith + observer states initialized at their exact $+1g$ upright equilibrium at $t = 0$ ($\dot{\mathbf{x}}(0) = \mathbf{0}$).

### 3.2 Semicircular Canals: Derivative-Free Partial Fractions & Passband Calibration
The 2nd-order *Schneider et al. (2015)* canal transfer function $H(s) = k \frac{s(1 + \tau_1 s)}{(1 + \tau_c s)(1 + \tau_2 s)}$ is biproper (degree 2 over degree 2). To evaluate it in C++ without differentiating angular velocity $\boldsymbol{\omega}_{\text{head}}$, we decompose $H(s)$ via partial fractions into a direct feedthrough $d = \frac{k \tau_1}{\tau_c \tau_2}$ plus two strictly-proper first-order low-pass states (`s2_scc_irr` and `s2_scc_reg` in [`docs/neuromotor_simulink_explorer.html`](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html)). After $B^{-1}$ spatial reconstruction, each stream is divided by its mid-frequency passband gain $G_{\text{pass}} = \frac{k \tau_1}{\tau_c}$ ($0.436629$ for Regular, $0.950526$ for Irregular) before the Regular stream enters the biproper Canal Luenberger inverse $H_{\text{Luen,scc}}(s) = \frac{1 + \tau_{2,\text{reg}} s}{1 + \tau_{1,\text{reg}} s}$.

### 3.3 Why We Use an Exact Continuous Ring Buffer Instead of a Padé Delay Approximation
Rational Padé approximations of transport delays $e^{-s\tau}$ introduce non-minimum-phase right-half-plane zeros that cause artificial phase distortion and high-frequency ringing on discontinuous or high-bandwidth signals.
- **In Time-Domain Simulation (`HeadNeckNeuralController.cpp`):** We avoid Padé delay approximations entirely by maintaining a continuous sliding `std::deque<std::pair<double, HistoryRecord>>` ring buffer (`top_hist` in [`docs/neuromotor_simulink_explorer.html`](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html)) and reconstructing any delayed signal $s(t - \tau)$ via exact bracketed linear interpolation (`std::lower_bound`).
- **In Offline Pre-Screening (`surrogacy/optimize_surrogate.py`):** We evaluate exact frequency-domain complex exponentials $e^{-j\omega\tau}$ directly at the 6 test frequencies, likewise avoiding Padé approximation error.

---

## 4. Neurophysiological Rationale for the 5-Delay Causal Decomposition

Every delay parameter in the controller is anchored to human neuroanatomical path lengths and myelinated axonal conduction velocities, decomposed into **5 physical segment delays** whose sums reproduce the three experimental closed-loop reflex latencies ($\text{VCR} = 10\,\text{ms}$, $\text{CCR} = 15\,\text{ms}$, $\text{Cortical} = 40\,\text{ms}$):

| Delay Symbol | Value | Anatomical Pathway & Myelinated Fiber Type | Literature Basis |
| :--- | :---: | :--- | :--- |
| **`τ_vest`** | **`2.0 ms`** | **CN VIII Vestibular Afferent (Scarpa's Ganglion $\to$ Vestibular Nuclei & Nodulus, Input to Sub 4 & Sub 5):** ~1.5–2 cm intracranial path along thick myelinated bipolar vestibular fibers + 1 synaptic delay. | *Goldberg et al. (2012); Cullen (2012)* |
| **`τ_prop`** | **`7.0 ms`** | **Shared Cervical Proprioceptive Afferent (`C1–C8` Spindles/DRG $\to$ Sub 4 Cuneocerebellar & Sub 6 Dorsal Horn CCR):** Fast Group Ia/II ($80\text{–}120\,\text{m/s}$) afferent entry into the C1–C8 dorsal horn (`Sub 6` CCR filters with $0\,\text{ms}$ extra internal delay) and dorsal-column ascent to the External Cuneate Nucleus / Cerebellar Nodulus (`Sub 4`). | *Gandevia & Burke (1992); Peterson et al. (1985)* |
| **`τ_asc`** | **`20.0 ms`** | **Ascending Cerebellothalamocortical Delay (Sub 4 $\to$ VL/VA Thalamus $\to$ M1 Cortex, Input 2 to Sub 1):** First half of the $40\,\text{ms}$ transcortical loop conducting the fused 3D head attitude and angular velocity estimate to Primary Motor Cortex M1. | *Pruszynski & Scott (2012); Happee et al. (2017)* |
| **`τ_desc` & `+10 ms` Forward Model** | **`12.0 ms`** (`-12 ms → -2 ms` at Sub 5) | **Unified Descending Pyramidal Delay (M1 $\to$ Sub 5 Brainstem VN & Sub 7 Spinal QP):** Because the final $8\,\text{ms}$ execution latency is credited to Subsystem 7 (`τ_spinal = 8 ms`), M1 uses a single unified descending delay $\tau_{\text{desc}} = 20 - 8 = 12\,\text{ms}$ for both `voluntary_torque[3]` to `Sub 7` ($12 + 8 = 20\,\text{ms}$ total descending) and the efference copy to `Sub 5` (where a $+10\,\text{ms}$ **Cerebellar Forward Predictor** advances $t - 12\,\text{ms}$ to $t - 2\,\text{ms}$ so all three VCR comparators match $\tau_{\text{vest}} = 2\,\text{ms}$). | *Cullen (2012, Trends Neurosci.); Roy & Cullen (2004, J. Neurosci.); Brooks & Cullen (2013, 2014, Curr. Biol.)* |
| **`τ_spinal`** | **`8.0 ms`** (`τ_vcr - τ_vest`) | **Shared Spinal QP/Pool & Ventral Root Execution Delay (Output of Sub 7 $\to$ 72 Muscles):** Placed inside Subsystem 7 after the Active-Set QP Synergy Solver and Lamina IX Motoneuron Pool Sum, $\tau_{\text{spinal}} = 8.0\,\text{ms}$ accounts for C1–C8 premotor interneuron recruitment, $\alpha/\gamma$-motoneuron integration, and ventral-root axonal conduction to the 72 muscle endplates—simultaneously completing the **VCR loop** ($2 + 8 = 10\,\text{ms}$), **CCR loop** ($7 + 0 + 8 = 15\,\text{ms}$), and **Cortical loop** ($20 + 12 + 8 = 40\,\text{ms}$). | *Watson & Colebatch (1998); Forbes et al. (2013, 2018); Happee et al. (2017)* |

---

## 5. Why the VCR & CCR Reflex Transfer Functions Are Formulated This Way

*(Click **Subsystem 5 (VCR)**, **Subsystem 6 (Spinal CCR Filters)**, and **Subsystem 7 (Active-Set QP & Motoneuron Pool)** in [`docs/neuromotor_simulink_explorer.html`](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) to inspect the state-space realizations and `[Derivation]` cards).*

### 5.1 Center-Frequency Normalized Lead-Lag (`H_sc(s)` and `H_ccr(s)`)
Both the Semicircular Canal VCR filter (`H_sc`) and the Spinal CCR stretch reflex filter (`H_ccr`) must provide a phase-lead zero $z < p$ around $1\text{–}8\,\text{Hz}$ to cancel the $\sim 35\,\text{ms}$ phase lag of Hill-type muscle activation dynamics (*Peng, Hain & Peterson, 1996*).
- **Problem with Standard Parameterization $K\frac{s+z}{s+p}$:** Changing the lead ratio $r_{\text{lead}} = z / p$ shifts the DC gain by $r_{\text{lead}}$, strongly coupling gain and phase in the optimizer.
- **Our Solution (Geometric Center-Frequency Normalization):** We parameterize both filters by $r_{\text{lead}} = z/p \in (0, 1]$ and divide by $\sqrt{r_{\text{lead}}}$:
  $$H_{\text{sc}}(s) = \frac{K_{\text{sc}}}{\sqrt{r_{\text{lead,sc}}}}\,\frac{s + z_{\text{sc}}}{s + p_{\text{sc}}}, \qquad H_{\text{ccr}}(s) = \frac{k_p}{\sqrt{r_{\text{lead,ccr}}}}\,\frac{s + z_{\text{ccr}}}{s + p_{\text{ccr}}}$$
  At the geometric center frequency $\omega_m = \sqrt{z p}$ (where maximum phase lead occurs), the magnitude is **identically $|H(j\omega_m)| = K$**, completely decoupling reflex gain $K$ from phase lead $r_{\text{lead}}$ during CMA-ES search.

### 5.2 Matched-Adaptation Gravity Cancellation & 1-State Strictly-Proper Phasic Otolith LPF (`H_phas(s)`, $D = 0$)
Unlike `H_sc` (driven by smooth $C^1$ canal angular velocity) and `H_ccr` (driven by smooth $C^1$ spindle firing), the Phasic Otolith VCR pathway (`s5_phas`) is driven by the Irregular Otolith gravitoinertial force $\hat{\mathbf{f}}_{\text{oto,irr}}(t - 2\,\text{ms})$ (`s2_otolith_irr`, *Schneider et al., 2015; Happee et al., 2017*):
1. **Why Gravity Cancellation Uses the Matched Irregular Adaptation Operator $H_{\text{adapt,irr}}(s)$ (`s5_eff_phas`):** Because Subsystem 2's Irregular Otolith passes $\mathbf{f}_{\text{GIF}} = \mathbf{a}_{\text{lin},H} - \mathbf{g}_H$ through Stage 1 Adaptation $H_{\text{adapt,irr}}(s) = \frac{\alpha_{\text{irr}} + c_{\text{irr}} s}{1 + c_{\text{irr}} s}$ ($\alpha_{\text{irr}} = 37.4/245.0 = 0.152653$, $c_{\text{irr}} = 0.25\,\text{s}$), its steady-state gravity component is $-\alpha_{\text{irr}}\mathbf{g}_H$ ($1.498\,\text{m/s}^2$) rather than $-1.000\,\mathbf{g}_H$ ($9.81\,\text{m/s}^2$). Filtering the delayed gravity projection $\hat{\mathbf{g}}_H(t - 2\,\text{ms}) = \bigl(\hat{R}_H^W(t - 2\,\text{ms})\bigr)^\top\mathbf{g}_W$ through the exact same adaptation operator $H_{\text{adapt,irr}}(s)$ ensures that static and dynamic head tilts cancel identically to $0.0\,\text{m/s}^2$ across all frequencies, while preserving the Irregular Otolith's adaptation zero ($c_{\text{irr}} = 0.25\,\text{s}$) and $27.2\,\text{ms}$ hair-cell phase lead ($\tau_{1,\text{irr}} = 27.2\,\text{ms}, b_{\text{irr}} = 7.0\,\text{ms}$) on true linear acceleration $\mathbf{a}_{\text{lin},H}$.
2. **Why `H_phas(s)` Is a 1-State Strictly-Proper Central LPF ($D = 0$):** Because Subsystem 2 (`s2_otolith_irr`) already embeds the biological irregular otolith adaptation zero and $27.2\,\text{ms}$ hair-cell phase lead, adding a second central lead zero in Subsystem 5 would double-count peripheral adaptation/lead. Instead, `H_phas(s)` is formulated as a clean 1-state strictly-proper central low-pass filter ($D = 0$, identical in form to `H_ton(s)`):
   $$H_{\text{phas}}(s) = \frac{K_{\text{phas}}\,p_{\text{phas}}}{s + p_{\text{phas}}}, \qquad \dot{\mathbf{x}}_{\text{phas}} = -p_{\text{phas}}\,\mathbf{x}_{\text{phas}} + \mathbf{u}_{\text{phas}}, \qquad \boldsymbol{\tau}_{\text{phas}} = K_{\text{phas}}\,p_{\text{phas}}\,\mathbf{x}_{\text{phas}}$$
   Keeping $D = 0$ prevents $1\,\text{ms}$ piecewise-constant finite-difference acceleration steps from feeding directly into `computeControls` and guarantees unconditional high-frequency stability across the $10\,\text{ms}$ VCR loop delay.

### 5.3 Voluntary-Gated Sherrington Reciprocal Inhibition & Intervertebral S-Buckling
Because the 3-DOF descending QP torque allocator (`s7_qp` in Subsystem 7) averages upper and lower cervical moment arms ($r_{\text{pitch},i} = \tfrac{1}{2}(r_{\text{p1},i} + r_{\text{p2},i})$), descending cortical and VCR torques control only total head-on-trunk angle ($\text{pitch}_1 + \text{pitch}_2$) and have **zero controllability** over the internal intervertebral S-buckling mode ($\text{pitch}_1 - \text{pitch}_2$).
- Per-muscle spinal stretch reflexes (CCR, `s6_ccr` & `s6_rect` in Subsystem 6) across all 72 muscles provide the sole active stabilization of individual cervical joints (`pitch1` vs. `pitch2`).
- Therefore, presynaptic Sherrington reciprocal inhibition (`s7_recip` in Subsystem 7 of [`docs/neuromotor_simulink_explorer.html`](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html)) is gated by the voluntary trajectory activity gate ($g_{\text{vol}} \in [0, 1]$): during quiet posture or external seat perturbations ($g_{\text{vol}} = 0$), all 72 muscles retain $100\%$ bilateral anti-buckling CCR stiffness; during active voluntary head turns ($g_{\text{vol}} \to 1$), antagonist stretch reflexes are smoothly suppressed so they do not resist the movement.

---

## 6. Anatomical Muscle Co-Contraction Zones & Active-Set QP Synergy Allocation

### 6.1 The 4 Cervical Muscle Co-Contraction Zones (`cached_minimum_excitations`)
Human cervical muscles are not recruited with a uniform baseline tone: deep slow-twitch postural extensors maintain continuous anti-gravity tone, whereas superficial fast-twitch flexors and hyoid strap muscles remain nearly silent at rest to avoid choking or anterior buckling. In [`HeadNeckNeuralController.cpp`](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp#L486-L516) (`s7_qp` in Subsystem 7 of [`docs/neuromotor_simulink_explorer.html`](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html)), the 72 muscles are classified into **4 anatomical co-contraction zones** that define each muscle's lower excitation bound $u_{\min,i}$:

| Zone & Count | Minimum Excitation Floor ($u_{\min,i}$) | Anatomical Muscle Sub-Groups & Exact Model Identifiers (`36` Bilateral Pairs = `72` Cervical Muscles) | How & Why $u_{\min,i}$ Was Chosen (Biomechanical, Histochemical & Spindle-Density Rationale) |
| :--- | :---: | :--- | :--- |
| **Zone 1: Superficial Flexors & Hyoid Strap** (`26` muscles = `13` pairs) | **`2.5%` (`0.025`)** | • **Sternocleidomastoid (`6`):** `stern_mast(_L)`, `cleid_mast(_L)`, `cleid_occ(_L)`<br>• **Suprahyoids (`14`):** `digastric_ant(_L)`, `digastric_post(_L)`, `Geniohyoid(_L)`, `Mylohyoid_Ant(_L)`, `Mylohyoid_Post(_L)`, `Stylohyoid_Lat(_L)`, `Stylohyoid_Med(_L)`<br>• **Infrahyoids (`6`):** `Sterno_hyoid(_L)`, `SternoThyroid(_L)`, `Omo_hyoid(_L)` | Because the skull's center of mass sits anterior to the `C0–C1` pivot, gravity already produces a forward flexion moment ($\approx 4.24\,\text{Nm}$). Superficial flexors and hyoid straps have long anterior moment arms; keeping them at the lowest floor ($2.5\%$) prevents unwanted downward head flexion, laryngeal compression, and metabolic waste against posterior extensors, matching human EMG near-silence during quiet upright sitting (*Keshner et al., 1989; Falla et al., 2004*). |
| **Zone 2: Deep Prevertebral Flexors & Scalenes** (`14` muscles = `7` pairs) | **`4.0%` (`0.040`)** | • **Longus Capitis (`2`):** `long_cap_sklc4(_L)`<br>• **Longus Colli (`6`):** `long_col_c1c5(_L)`, `long_col_c1thx(_L)`, `long_col_c5thx(_L)`<br>• **Scalenes (`6`):** `scalenus_ant(_L)`, `scalenus_med(_L)`, `scalenus_post(_L)` | Unlike the superficial SCM, the deep prevertebral sleeve (`longus colli`, `longus capitis`) and scalenes wrap directly along the vertebral bodies with short moment arms and high Type I slow-twitch fiber proportions (*Boyd-Clark et al., 2001*). A $4.0\%$ tonic floor flattens the cervical lordosis and provides active segmental guy-wire stiffness to prevent intervertebral S-buckling (`pitch1` vs. `pitch2`) when large posterior extensors contract (*Mayoux-Benhamou et al., 1994*). |
| **Zone 3: Superficial Multi-Articular Extensors** (`12` muscles = `6` pairs) | **`5.0%` (`0.050`)** | • **Trapezius (`4`):** `trap_cl(_L)`, `trap_acr(_L)`<br>• **Splenius Capitis & Cervicis (`6`):** `splen_cap_sklc6(_L)`, `splen_cap_sklthx(_L)`, `splen_cerv_c3thx(_L)`<br>• **Scapulocervical (`2`):** `levator_scap(_L)` | Span the full cervical column with large posterior extension, coronal roll, and axial yaw moment arms. A $5.0\%$ baseline tone shares the anti-gravity load with the deep extensors and provides broad 3D guy-wire stabilization while leaving $95\%$ of their dynamic capacity free for phasic VCR/CCR reflex recruitment and voluntary head turns. |
| **Zone 4: Deep Postural Extensors & Suboccipitals** (`20` muscles = `10` pairs) | **`7.0%` (`0.070`)** | • **Semispinalis (`6`):** `semi_cap_sklc5(_L)`, `semi_cap_sklthx(_L)`, `semi_cerv_c3thx(_L)`<br>• **Cervical Erector Spinae (`6`):** `longissi_cap_sklc6(_L)`, `longissi_cerv_c4thx(_L)`, `iliocost_cerv_c5rib(_L)`<br>• **Suboccipital Triangle (`8`, `C0–C2`):** `rectcap_post_maj(_L)`, `rectcap_post_min(_L)`, `obl_cap_sup(_L)`, `obl_cap_inf(_L)` | Possess the highest muscle-spindle density in the human body ($190\text{–}242\text{ spindles/g}$ in suboccipitals and semispinalis; *Kulkarni et al., 2001; Boyd-Clark et al., 2002*) and fatigue-resistant Type I oxidative fibers. A $7.0\%$ floor provides the primary continuous anti-gravity extension trim against skull overhang, stabilizes the upper-cervical `C0–C2` pivot (`pitch2`, `yaw1`), and keeps their high-density intrafusal spindles taut via baseline $\alpha\text{–}\gamma$ coactivation. |
| **Extraocular / Default Fallback** (`12` eye muscles, if enabled) | **`3.0%` (`0.030`)** | • **Bilateral Recti & Obliques (`12`):** `r_/l_Lateral_Rectus`, `Medial_Rectus`, `Superior_Rectus`, `Inferior_Rectus`, `Superior_Oblique`, `Inferior_Oblique` | Default baseline floor applied only when `enable_eye_afferents` is active. |

### 6.2 15-Iteration Active-Set Cholesky/LU QP Allocator
At every integration step, the descending 3-DOF torque $\boldsymbol{\tau}_{\text{total}} = \text{clamp}(\boldsymbol{\tau}_{\text{vol}} + \boldsymbol{\tau}_{\text{vcr}}, -\boldsymbol{\tau}_{\max}, +\boldsymbol{\tau}_{\max})$ (with $|\tau_{\text{total},k}| \in [0, \tau_{\max,k}]$, $\boldsymbol{\tau}_{\max} = [150, 100, 100]^\top\,\text{N}\cdot\text{m}$) is distributed across all 72 muscles by solving the minimum-effort quadratic program:
$$\min_{\mathbf{u}_\alpha} \frac{1}{2}\|\mathbf{u}_\alpha\|_2^2 \quad \text{s.t.} \quad A\,\mathbf{u}_\alpha = \boldsymbol{\tau}_{\text{total}}, \quad u_{\min,i} \le u_{\alpha,i} \le 1.0$$
where $A_{:,i} = J_{3,i}\,F_{\max,i} \in \mathbb{R}^{3\times 72}$. Because the equality constraint has dimension 3, forming the $3\times 3$ active-set Gram matrix $G_{\text{free}} = A_{\text{free}} A_{\text{free}}^\top + \lambda_{\text{QP}} I_3$ ($\lambda_{\text{QP}} = 10^{-3}$) and solving via $3\times 3$ LU factorization (`SimTK::FactorLU`) converges to the exact constrained optimum in $\le 15$ micro-iterations (microseconds of CPU time) without external QP library overhead.

### 6.3 Supraspinal-Only $\alpha\text{–}\gamma$ Coactivation (Why Segmental `Ia` CCR Bypasses $\gamma$-Motoneurons)
In Subsystem 7, the delayed extrafusal $\alpha$-motoneuron command (`ctrl_0`) sums both the supraspinal QP synergy excitation $u_{\alpha,i}$ and the segmental stretch reflex excitation $\gamma_{\text{inhib},i}\,u_{\text{ccr},i}$, whereas the dynamic and static fusimotor drives ($\gamma_{\text{dyn},i}, \gamma_{\text{stat},i}$) are computed **strictly from the delayed supraspinal QP excitation $u_{\alpha,\text{del},i}$**:
$$\text{ctrl}_{0,i}(t) = u_{\text{final},i}(t) = \text{clamp}\!\bigl(u_{\alpha,i}(t - \tau_{\text{spinal}}) + \gamma_{\text{inhib},i}(t - \tau_{\text{spinal}})\,u_{\text{ccr},i}(t - \tau_{\text{spinal}}),\; u_{\min,i},\; 1\bigr), \qquad \gamma_{\text{dyn/stat},i}(t) = 100\,K_{\gamma,\text{dyn/stat}}\,u_{\alpha,i}(t - \tau_{\text{spinal}})$$
As established in mammalian neurophysiology (*Hulliger, 1984; Burke et al., 1979*), monosynaptic `Ia` muscle spindle afferents project exclusively to homonymous $\alpha$-motoneurons and **never** make excitatory autogenetic connections onto $\gamma$-motoneurons. Driving $\gamma_{\text{dyn}}$ and $\gamma_{\text{stat}}$ solely from the supraspinal synergy $u_{\alpha,\text{del}}$ prevents a runaway positive-feedback loop ($\text{Ia} \uparrow \implies u_{\text{ccr}} \uparrow \implies \gamma \uparrow \implies \text{intrafusal tension} \uparrow \implies \text{Ia} \uparrow$).

---

## 7. Paradigm A 4-Zone Cervical Range of Motion (ROM) & Early-Abort Guard

### 7.1 The 4 Biomechanical Joint Zones
Calibrated against human cervical spine biomechanics (*White & Panjabi 1990; Dvorak et al. 1993; Vasavada et al. 1998; Nightingale et al. 2002, 2007*), passive joint resistance in [`HYOID_HeadNeckNeural_Model_Detailed.osim`](../osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim) is structured into 4 zones:
1. **Zone I — Neutral Deadband ($0 \to 75\%$ Voluntary ROM):** Zero passive limit force ($F_{\text{limit}} = 0$). All physiological seated perturbation responses occur strictly inside this zone, governed purely by active neuromuscular control and intervertebral bushing elasticity.
2. **Zone II — $C^1$ Cubic Hermite Transition Zone ($75\% \to 100\%$ Voluntary ROM):** Smooth $C^1$ polynomial ramp where facet capsules and ligaments begin to engage.
3. **Zone III — Elastic Capsular Penetration Zone ($100\% \to 120\%$ Voluntary ROM):** High linear ligamentous stiffness ($K = 50\,\text{Nm/rad}$, damping $2.0$).
4. **Zone IV — Hard Bony Stop ($120\%$ Voluntary ROM):** Anatomical bony impingement and ligament failure threshold (*Nightingale et al., 2007*).

| Coordinate | Anatomical Segment | 100% Voluntary ROM | Zone I Deadband ($0 \to 75\%$ ROM) | Zone II Transition ($75\% \to 100\%$) | Zone IV Hard Stop ($120\%$) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **`pitch1`** | Lower Neck (`C2–T1`) | `[-16.0°, +24.0°]` | **`[-12.0°, +18.0°]`** | `5.0°` (`0.0873 rad`) | `[-19.2°, +28.8°]` |
| **`pitch2`** | Upper Neck (`C0–C2`) | `[-33.0°, +48.0°]` | **`[-24.75°, +36.0°]`** | `10.0°` (`0.1745 rad`) | `[-39.6°, +57.6°]` |
| **`roll1`** | Lower Neck (`C2–T1`) | `[-6.0°, +6.0°]` | **`[-4.5°, +4.5°]`** | `1.5°` (`0.0262 rad`) | `[-7.2°, +7.2°]` |
| **`roll2`** | Upper Neck (`C0–C2`) | `[-33.0°, +33.0°]` | **`[-24.75°, +24.75°]`** | `8.25°` (`0.1440 rad`) | `[-39.6°, +39.6°]` |
| **`yaw1`** | Upper Pivot (`C1–C2`) | `[-38.0°, +38.0°]` | **`[-28.5°, +28.5°]`** | `9.5°` (`0.1658 rad`) | `[-45.6°, +45.6°]` |
| **`yaw2`** | Lower Neck (`C2–T1`) | `[-27.0°, +27.0°]` | **`[-20.25°, +20.25°]`** | `6.75°` (`0.1178 rad`) | `[-32.4°, +32.4°]` |

### 7.2 Why CMA-ES Aborts Immediately at the $75\%$ Deadband Boundary
In *Happee et al. (2017)* seated perturbations, human head pitch excursions peak at $12.36^\circ$ at $0.324\,\text{Hz}$ ($\sim 30\%$ of the deadband) and $< 6^\circ$ above $0.78\,\text{Hz}$. If an optimizer were allowed to enter Zone II ($>75\%$ ROM), a weak neural controller could "cheat" by letting the head fall onto passive ligamentous limit stops to bound its gain error. Therefore, [`cmaes_worker_p22.py`](../cmaes_worker_p22.py#L219-L228) monitors all 6 coordinates at every integration step and **immediately aborts** any simulation that crosses the $75\%$ deadband boundary, assigning a heavy penalty score.

---

## 8. C++ Hardware Vectorization (`Intel AVX2`) & CMA-ES Acceleration Architecture

Evaluating a 72-muscle, 330+ ODE-state closed-loop OpenSim model across 6 perturbation frequencies is computationally intensive. We combine low-level C++ SIMD/compiler optimizations with 6 algorithmic CMA-ES accelerators in [`run_phase2_4_campaign.py`](../scripts/run_phase2_4_campaign.py) and [`cmaes_worker_p22.py`](../cmaes_worker_p22.py):

### 8.1 Intel AVX2 SIMD Vectorization & Link-Time Code Generation (`CMakeLists.txt`)
Both performance-critical C++ plugins ([`HeadNeckNeuralController_plugin/CMakeLists.txt`](../HeadNeckNeuralController_plugin/CMakeLists.txt#L35-L38) and [`proprioception_plugin/CMakeLists.txt`](../proprioception_plugin/CMakeLists.txt#L53-L56)) are compiled under MSVC with:
```cmake
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /O2 /Ob3 /arch:AVX2 /GL)
    target_link_options(${PROJECT_NAME} PRIVATE /LTCG)
endif()
```
- **`/arch:AVX2` (256-bit Intel Advanced Vector Extensions 2 + FMA3):** Vectorizes the tight contiguous `std::vector<double>` loops over all 72 muscles (4-double wide SIMD lanes with fused multiply-add), including the 72-muscle spindle Luenberger updates, the $3\times 72$ and $6\times 72$ Jacobian matrix-vector products, and the $3\times 3$ active-set QP Gram matrix accumulations.
- **`/Ob3` + `/GL` + `/LTCG` (Aggressive Inlining & Whole-Program Link-Time Code Generation):** Inlines helper methods across translation units (`HeadNeckNeuralController.cpp` $\leftrightarrow$ `Mileusnic06Spindle.cpp`).
- **`O(1)` Cached State & Output Pointers:** All 330+ `SimTK::Stage::Velocity` state variable pointers (`traverseToStateVariable`) and 144 spindle output pointers (`primary_Ia`, `secondary_II`) are resolved once in `initializeGeometryAndBaseline` and accessed via raw pointer dereferences (`->getValue(state)`, `->setDerivative(state, ...)`), completely eliminating string hash-map lookups during numerical integration.

### 8.2 The 6 Algorithmic Accelerators in the CMA-ES Campaign
1. **Single-Threaded Worker Isolation across 14 Physical Cores (`popsize = 28 = 2 × 14 Waves`):**  
   Each worker sets `SIMB_NUM_THREADS=1` and `OMP_NUM_THREADS=1` to eliminate OpenMP thread-pool contention, running 14 independent processes in parallel across all 14 physical CPU cores (`28` candidates per generation = exactly `2` full waves of `14` cores with zero idle tail cores).
2. **Subprocess-Per-Frequency Memory Reclamation & Disk Caching:**  
   [`evaluate_candidate()`](../scripts/run_phase2_4_campaign.py#L162-L183) spawns a fresh Python/OpenSim subprocess for each individual frequency and caches per-frequency JSON results in `logs/candidates/`. This forces the OS to reclaim 100% of Simbody's C++ heap memory after every frequency (preventing multi-gigabyte RAM bloat) and allows instant crash/resume recovery.
3. **Fastest-First Frequency Ordering & Early Pruning Cascade:**  
   The 6 canonical frequencies are evaluated in order of ascending simulation duration and high instability sensitivity:  
   `[2.446 Hz, 5.583 Hz, 7.927 Hz, 1.099 Hz, 0.783 Hz, 0.324 Hz]`.  
   Because $2.446\,\text{Hz}$ requires only $2.73\,\text{s}$ of simulated time (vs. $7.67\,\text{s}$ at $0.324\,\text{Hz}$), unstable or deadband-violating candidates are caught and pruned on the first frequency in seconds—saving $>75\%$ of wasted CPU time—and assigned a graduated early-stop penalty `(6 - t_idx) * 200,000 pts`.
4. **TPA + Block-Orthogonal Antipodal Sampling (`sample_candidates_tpa_orthogonal`):**  
   Standard random Gaussian sampling in 16D suffers from directional clumping and variance in finite-sample covariance updates. Each generation of 28 candidates is structured as:
   - **Candidates 0 & 1:** 1 Two-Point Step-Size Adaptation (**TPA**) mirror pair along the mean evolution path $\mathbf{p}_c$ (`CMAAdaptSigmaTPA`), adapting global step size $\sigma$ without extra function evaluations.
   - **Candidates 2–27:** 13 mutually **QR-orthogonal antipodal pairs** ($\boldsymbol{\mu} \pm \Delta\mathbf{x}_k$) in the $(N-1)$-dimensional subspace perpendicular to $\mathbf{p}_c$, cycling through a full orthogonal basis so 88% of all search dimensions are probed uniformly every single generation.
5. **Local Gaussian Process (GP) Direction-Pair Sieve (`SimpleGP`, Active at Archive $\ge 80$):**  
   Once the evaluation archive reaches 80 real OpenSim runs, [`sample_candidates_tpa_orthogonal()`](../scripts/run_phase2_4_campaign.py#L258-L296) fits a local Cholesky Gaussian Process regressor (`SimpleGP`) on the 80 nearest historical candidates (Winsorizing crash penalties at $200,000\,\text{pts}$) and screens **12 candidate QR-orthogonal bases** in microseconds using a **Pairwise-Minimum Lower Confidence Bound ($\text{LCB} = \mu - 0.5\sigma$)** acquisition function before launching the OpenSim workers.
6. **Microsecond Frequency-Domain Warm-Start (`< 1 μs`) & Dynamic Boundary Expansion:**  
   - **First-Principles Warm-Start (`surrogacy/optimize_surrogate.py`):** Evaluates Black's closed-loop feedback formula on the measured OpenSim passive cervical compliance $G_{\text{passive}}(j\omega)$ in $< 1\,\mu\text{s}$ per candidate to identify the initial rational centroid $x_0$ (`data/prescreen_rational_champion.json`). *(All actual CMA-ES generations run 100% on the unapproximated non-linear OpenSim 4.6 model).*
   - **Dynamic Boundary Expansion (`check_and_expand_bounds`):** If a generation champion moves within $5\%$ of any active parameter bound, that boundary is automatically expanded by $+10\%$ of the range (up to hard physical floors/ceilings) and hot-updated in `pycma`'s `BoundTransform` without restarting the covariance matrix.
