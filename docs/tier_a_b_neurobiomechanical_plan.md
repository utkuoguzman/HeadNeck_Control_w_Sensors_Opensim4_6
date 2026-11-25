# Implementation Plans: Tier A (Vestibular Cleanup & Re-Opt) and Tier B (GTO, Renshaw, SRS & Cervical Ligament/Capsule Reflexes)

> [!IMPORTANT]
> **Scope & Status:** Planning specification only — no source files are modified by this document. Every plan builds strictly above the **restored clean $C^1$ baseline** (`osimHeadNeckNeural.dll` SHA-256 `26256D15E0C4A0DF3E3E31DFBABECEC45580F2F663E388C03BFE39814BE86E00`) and obeys our **strict zero-ground-truth rule**, **$C^1$-continuity**, **strict properness ($D = 0$)**, and **deterministic Noise-OFF → multi-seed Noise-ON verification**. Tier C adaptive/plastic phenomena are separated out into [nest_future_spiking_phenomena.md](nest_future_spiking_phenomena.md).

---

## 0. Clean $C^1$ Baseline & Root-Cause Reference

All three plans target two intertwined physical issues uncovered during Stage A analysis:

| Metric / Property | Verified Clean $C^1$ Baseline Value | Source / Locus |
| :--- | :--- | :--- |
| **3-Axis Passive Bode (`Noise OFF`)** | `CombinedTotal = 5,584.22 pts` (`Classic = 4,875.84`, `XTalk = 464.33`, `HF = 244.05`, `ALL 28 TRUE`, `0` aborts)<br>`APEO = 2,268.02` (`cls 2,162.48`, `rv 0.9607`), `LatEC = 2,459.82`, `YawMA = 856.38` | [standard_plot_summary_c11_continuous_gates.json](../logs/standard_plot_summary_c11_continuous_gates.json) |
| **Zheng `2g/3g` Sled Sweep (`Noise OFF`)** | `16/16` monotonic, `Settle = +0.00°`, NBDL `u = -0.75` (`+37.48°` @ `2g`, `+45.93°` @ `3g`), `nPk = 0..1` (`u <= -0.50`), **`nPk = 2` (`u >= -0.25`)**, `MaxAct <= 0.7924` | [run_zheng_2g_3g_urgency_sweep.py](../scripts/run_zheng_2g_3g_urgency_sweep.py#L378-L386) |
| **Voluntary `50°` RPY & `5-DOF` Shift** | `18.0 s` RPY complete (`0` overshoot flags); `7/7` 5-DOF Shift pass (`Case1 +1.971 cm`, `Case2 -1.564 cm`, `Case3 +0.490 cm`) | [se2_c1pos_3axis_champion.json](../logs/se2_c1pos_3axis_champion.json) |
| **Root Cause 1 (Double-Lead)** | Peripheral irregular otolith `H_irr(s)` (`τ_1 = 27.2 ms`) stacked with central `z_phas = 28.57 ms` (`K_phas = 13.07`) → `+10.18 N·m` spike at `t = 72 ms` (`Peak 1 @ 88 ms`, `-0.78°` whip) | [HeadNeckNeuralController.cpp](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp#L4045-L4076) |
| **Root Cause 2 (`g_highg` Cleft)** | Binary-like `7.80 ± 0.65 m/s²` attenuation plunges `g_highg: 1.000 → 0.005` (`88–112 ms`) right as canals brake the whip (`-2.75 N·m`) → `128 ms` cleft before `Peak 2` (`160 ms`) | [HeadNeckNeuralController.cpp](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp#L2897-L2911) |
| **Root Cause 3 (`28 Hz` Ringing Floor)** | Removing ad-hoc `highg_p_sc_floor = 22.0` + `highg_tau_lead_add_s = 0.022` triggers `~28 Hz` canal-neck sawtooth (`nPk = 3–4`) | [HeadNeckNeuralController.cpp](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp#L2977-L2978) |
| **Causal Envelope Bound (`Bode_APEO` vs `2g`)** | During `t = 50–78 ms` of `Sled 2g`, instantaneous vestibular kinematics & filter states lie strictly inside `Bode_APEO` envelopes (e.g., `|a| = 5.73 < 7.14 m/s²`, `|ts| = 4.06 < 9.50`, `|tl| = 9.30 < 9.56`) | Stage A envelope probe |

### Pre-Implementation Step — Phase B1.0: 30-Second Read-Only Physical Discriminator Probe
Before editing any C++ or `.osim` files, run one lightweight read-only Python diagnostic comparing `Bode_APEO` vs. `Sled 2g (t = 50–80 ms, u = 0)` on the **non-vestibular physical variables** where `APEO` and `Sled 2g` can actually differ:
1. Active tendon/fiber force $F_{\text{splen}}$ and force rate $dF_{\text{splen}}/dt$ (**GTO `Ib` separation ratio**),
2. Normalized fiber strain $\Delta L_m / L_{0,m}$ across superficial/deep muscles (**Short-Range Stiffness yield separation**),
3. Intervertebral differential shear/buckling `pitch1 - pitch2`, `q1..q2`, and cervical moment arms (**5-DOF suboccipital & facet-capsule separation**).

This single probe quantifies how much heavy lifting each Tier B mechanism can do *without* degrading `Bode_APEO`.

---

## 1. Plan 1 — Tier A: Vestibular Root-Cause Cleanup & Joint 3-Axis Re-Optimization

### Objective & Literature Justification
Eliminate stacked, ad-hoc phasic nonlinearities and unphysiological central double-differentiation so that every active vestibular element maps 1-to-1 to primary neurophysiology:
- *Fernández & Goldberg (1976), J. Neurophysiol.* 39:970–995 — Peripheral regular afferents (`~30–40 spk/s·g`) vs. irregular high dynamic sensitivity + early rate saturation.
- *Schneider, Jamali, Carriot, Chacron & Cullen (2015), J. Neurosci.* & *Jamali, Sadeghi & Cullen (2009, 2013)* — Irregular vestibular afferents obey a **single Linear–Nonlinear (LN) cascade** up to `8 G`.
- *Peng, Hain & Peterson (1996), Biol. Cybern.* 75:309–319 & *Happee et al. (2017), J. Biomech.* 58:203–211 — Multi-loop VCR (canal + otolith) and CCR division of stabilization across `0.2–8 Hz`.
- *Zheng, Mo, Liu & Li (2021), IEEE TNSRE* 29:1394–1402 — Unimodal *splenius capitis* impact target (`2g, 3g`).

### Implementation Phases

#### Phase A1 — Remove Central Double-Differentiation & Biophysically Bound `K_phas`
1. **Confinement of `z_phas`:** Set/bound central pitch phasic lead zero `z_phas = 0` (or `z_phas ∈ [0, 0.005] s`, and test `z_phas_roll` symmetrically) in [HeadNeckNeuralController.cpp](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp#L4045-L4066), restoring consistency with [neuromotor_control_architecture_c4.md](neuromotor_control_architecture_c4.md) Section 5.2 (peripheral `H_irr(s)` `27.2 ms` lead is the *sole* otolith phase-lead mechanism).
2. **Physical Bound on `K_phas`:** Constrain `K_phas` in CMA-ES below a biophysically anchored ceiling relative to skull translational-pitch inertial coupling ($m_{\text{head}} h_{\text{COM}} \approx 0.18\text{ N·m/(m/s}^2\text{)}$) rather than letting `K_phas = 13.07` (`~70×`) compensate Hill muscle lag at `4–8 Hz`. Shift high-frequency stabilization cleanly onto rotational canals (`K_sc, z_sc, p_sc`) and neck CCR (`Kp_prop, z_ccr`).

#### Phase A2 — Replace Stacked High-G Cliffs with Single Peripheral LN Saturation (`Sub 2A`)
1. **Clean LN Cascade at Irregular Otolith Output:** Apply **one** $C^\infty$-smooth hyperbolic saturation right after the irregular otolith filter + gravity/efference cancellation (`L2882–2914` & `L3984–4007`):
   $$f_{\text{irr,sat}} = a_{\text{sat}}\,\tanh\!\left(\frac{f_{\text{irr}}}{a_{\text{sat}}}\right)$$
   - Keep `phasic_oto_sat_mps2` property for this single peripheral LN ceiling (switching from piecewise `applyLinearKneeSaturation` or offering an explicit $C^\infty$ `tanh` mode toggle `phasic_oto_ln_mode`).
2. **Decommission Overlapping Cliffs:** Set `g_highg` (`highg_oto_knee_mps2 = 0` to bypass cliff shutdown), `phasic_torque_sat_nm = 0`, and `phasic_lead_sat_nm = 0` as the Tier-A test configuration (preserving properties at `0 = off` so exact A/B ablations remain reproducible).
   - **Why `tanh` avoids the `128 ms` cleft:** Unlike `g_highg` plunging `1.000 → 0.005`, `tanh` *plateaus smoothly* without zeroing the otolith channel during `88–112 ms`, handing braking torque continuously to semicircular canals (`-sc`).

#### Phase A3 — Joint Multi-Objective 3-Axis Bode + `2g/3g` Sled Re-Optimization
1. **Free Parameter Vector (~20–24 active parameters, focusing on pitch/shared loops):**
   - Pitch VCR + LN: `K_ton, p_ton, K_sc, z_sc, p_sc, K_phas (bounded), p_phas, z_phas (=0 or <=5ms), a_sat, tau_lead_pole`
   - Pitch CCR + Fusimotor + Task: `Kp_prop_pitch, z_ccr, p_ccr, Kp_proprioception, K_gamma_dyn, K_gamma_stat, Kp_task_pitch, Kd_task_pitch, Ki_task_pitch`
   - Optional light roll/yaw co-tuning (`K_gamma_*` affects all 3 axes!).
2. **Joint Loss Function:**
   - **3-Axis Passive Bode** (`APEO`, `LatEC`, `YawMA`): all 28 pass booleans `TRUE`, `0` Zone-II `75% ROM` aborts, minimize `CombinedTotal` (`< 5,584` ideally; record exact Pareto trade-off if `z_phas = 0`).
   - **Zheng `2g & 3g` Urgency Sweep (`u ∈ {-1, -0.75, -0.5, -0.25, 0, +0.25, +0.5, +1}`):**
     - Strict 8-point flexion monotonicity for both `2g` and `3g`.
     - NBDL `u = -0.75` inside human corridors (`33–40°` @ `2g`, `40–47°` @ `3g`).
     - **Unimodal *Splenius Capitis*** across all active urgencies using the exact ground-truth counter in [run_zheng_2g_3g_urgency_sweep.py](../scripts/run_zheng_2g_3g_urgency_sweep.py#L378-L386) (`win_half = 25`, NO local-max masking, `prom >= 0.015`, target `n_splen_peaks <= 1` for every `(g, u)` run).
     - Onset backward whip bounded (`min_flex_onset_deg > -0.35°`), `Settle ≈ 0°`.
   - **Compute Allocation:** Local 20-CPU multiprocessing (~`1.5 min`/eval batch) or **TRUBA** cluster (once interactive SSH password/key auth is enabled by the user).

#### Phase A4 — Full 4-Suite Noise-OFF + Multi-Seed Noise-ON Verification
1. Re-run 3-axis Passive Bode (`scripts/generate_standard_plot.py`), `2g/3g` Sled Sweep (`scripts/run_zheng_2g_3g_urgency_sweep.py`), `18.0 s` Voluntary RPY Sweep (`50°`), `5-DOF` C1 Shift (`7/7`), and multi-seed `enable_sensor_noise=true, enable_motor_noise=true`.
2. Sync rebuilt `osimHeadNeckNeural.dll` to workspace root and update [neuromotor_block_explorer.html](neuromotor_block_explorer.html) + root mirror (`SHA-256` verified identical).

---

## 2. Plan 2 — Tier B1: Reusing Disregarded `Lin02GolgiTendonOrgan` (`Ib`) + Renshaw Inhibition + Short-Range Stiffness (SRS)

### 2.1 Direct Verdict: Can We Reuse Our Disregarded GTO Implementation?
**YES — directly and cleanly, with 6 specific repairs first.**  
Inspecting [Lin02GolgiTendonOrgan.cpp](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp), [Lin02GolgiTendonOrgan.h](../proprioception_plugin/Lin02GolgiTendonOrgan.h), [Millard12EqMuscleWithAfferents.cpp](../proprioception_plugin/Millard12EqMuscleWithAfferents.cpp), and [HYOID_HeadNeckNeural_Model_Detailed.osim](../osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim#L3549-L3551) confirms:
- All **72+ muscles** (`Millard12EqMuscleWithAfferents`) in `HYOID_HeadNeckNeural_Model_Detailed.osim` **already instantiate `<Lin02GolgiTendonOrgan name="GTO">`** inside the XML and compile into `osimMillard12EqWithAff.dll`! It was simply turned off via `constructProperty_enable_GTO(false)` (`L186`) for simulation speed.
- However, the original Verduzco implementation contains **6 concrete defects** that MUST be fixed before closed-loop GTO control is trustworthy:

| # | Defect Found in Existing GTO Code | Exact Location | Planned Fix |
|---|---|---|---|
| **1** | **Stale / Uninitialized `cached_gto_out`:** `getGTOout(s)` returns mutable `cached_gto_out`, which is `0.0` at `t=0` (`initFromMuscle` never sets it!) and only updates inside `computeStateVariableDerivatives(s)` (risking stale trial-step values during `computeControls(s)`). | `Lin02GolgiTendonOrgan.cpp` [`L101–108`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L101-L108), [`L223–241`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L223-L241) | Compute `smooth_max(y(s), thr)` **statelessly from `s`** inside `getGTOout(s)` (valid at `SimTK::Stage::Velocity` and `Dynamics`), eliminating mutable cache bugs. |
| **2** | **Wrong Force & Negative `log` Risk:** Uses `getFiberForce(s)` directly inside `std::log((F/Gf)+1)` with no clamp (`NaN` if a trial RK step has $F \le -G_f$). | `Lin02GolgiTendonOrgan.cpp` [`L201`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L201), [`L236`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L236) | Use series tendon force `std::max(0.0, musclePtr->getTendonForce(s))` ($F_T = F_F\cos\alpha$), guaranteeing $\log(1 + \cdot) \ge 0$ (`NaN`-impossible). |
| **3** | **Stiff 4-State `X''` Cascade (288 ODE States):** Uses two `tau = 0.01 s` (`100 rad/s`) LPFs just to differentiate $X''$ and feed `68*Xpp + 103.2*Xp + 16*X`. Doubles stiff ODE count unnecessarily. | `Lin02GolgiTendonOrgan.cpp` [`L120–123`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L120-L123), [`L208–221`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L208-L221) | Replace with the **exact 2-state derivative-free partial-fraction realization** of Lin & Crago (2002) Eq. (2) (see derivation below): **144 slow ODE states** (`0.2` & `2.0 rad/s`), zero `lpf_tau` lag distortion, exact $t=0$ equilibrium! |
| **4** | **Hardcoded Cat Soleus `Gf = 4.0 N` & Missing Eye Gate:** Cervical $F_{\max}$ ranges widely (small deep rotators vs. large multi-articular extensors); also `extendAddToSystem` omits owner `isAfferentActive()`. | `Lin02GolgiTendonOrgan.cpp` [`L26–28`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L26-L28), [`L114–127`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L114-L127) | Add `gto_gf_fraction` property so $G_{f,i} = \max(1.0,\; c_f F_{\max,i})$ scales physiologically per muscle; add explicit `force_scale` toggle (`40×` aggregate vs `1×` single afferent); check owner `isAfferentActive()` before adding ODE states for `r_*`/`l_*`. |
| **5** | **No OpenSim Output on `Millard12EqMuscleWithAfferents`:** Only `primary_Ia` & `secondary_II` have `OpenSim_DECLARE_OUTPUT` (`L106–107`). | `Millard12EqMuscleWithAfferents.h` [`L104–108`](../proprioception_plugin/Millard12EqMuscleWithAfferents.h#L104-L108) | Add `OpenSim_DECLARE_OUTPUT(tendon_Ib, double, getIbAfferent, SimTK::Stage::Dynamics);` and cache `cached_tendon_afferent_outputs` in `HeadNeckNeuralController`. |
| **6** | **Dead Code:** `calculateDerivatives` (`Fornberg` method) and `ts, nl, Dnl, C0, C1` are completely unused. | `Lin02GolgiTendonOrgan.cpp` [`L247–331`](../proprioception_plugin/Lin02GolgiTendonOrgan.cpp#L247-L331) | Remove dead finite-difference helper and unused mutable buffers for clarity and cache locality. |

#### Exact Derivative-Free 2-State Realization of Lin & Crago (2002) Eq. (2)
Factoring the existing filter `L220` ($\frac{68.0 s^2 + 103.2 s + 16.0}{s^2 + 2.2 s + 0.4} = 40\,\frac{1.7 s^2 + 2.58 s + 0.4}{(s + 2.0)(s + 0.2)}$) via partial fractions yields an **exact biproper 2-state realization with zero input derivatives**:
$$H_{\text{Lin02}}(s) = 40\left(1.7 - \frac{17/15}{s + 2.0} - \frac{2/75}{s + 0.2}\right)$$
$$\dot{w}_1 = -2.0\,w_1 + u_{\text{nl}}, \qquad \dot{w}_2 = -0.2\,w_2 + u_{\text{nl}}, \qquad y = 40\left(1.7\,u_{\text{nl}} - \frac{17}{15}\,w_1 - \frac{2}{75}\,w_2\right)$$
where $u_{\text{nl}}(s) = G_g \ln\!\left(1 + \frac{\max(0, F_T(s))}{G_{f,i}}\right)$, initial equilibrium at $t = 0$ is $w_1(0) = u_{\text{nl}}(0)/2.0$, $w_2(0) = u_{\text{nl}}(0)/0.2 \implies \dot{w}_1(0) = \dot{w}_2(0) = 0,\; y(0) = 40\,u_{\text{nl}}(0)$ (exact steady-state fixed point), and the high-frequency dynamic gain (`68.0/16.0 = 1.7×` DC) is preserved identically without `0.01 s` LPF attenuation or `X''` noise amplification.

---

### 2.2 Phase B1.1 — GTO (`Ib`) Conduction Delays, Efference-Copy Cancellation & Spinal Integration

- **Literature:** Houk & Henneman (1967); Lin & Crago (2002); Pierrot-Deseilligny & Burke (2012, *The Circuitry of the Human Spinal Cord*); McCrea (1998) — Group Ib fibers conduct slightly slower than Ia (`~60–90 m/s`), project via disynaptic/trisynaptic Lamina V–VI interneurons onto homonymous and synergistic $\alpha$-motoneurons (`~5–7 ms` central + ventral delay), and undergo **strong presynaptic/interneuronal modulation during voluntary contraction** so self-generated voluntary forces do not trigger autogenic collapse.

1. **Biological Delay Chain (Integrated into the 13-Branch Delay Atlas in `HeadNeckNeuralController`):**
   - **Peripheral Group Ib Axon (`τ_ib_axon = 1.2 ms`):** Store `raw_gto_ib_rate[72]` in `HistoryRecord` (`std::deque`), interpolated via `getDelayedData(t, 0.0012, true)` alongside `τ_spindle_ia_delay = 1.0 ms` and `τ_spindle_ii_delay = 2.5 ms`.
   - **Disynaptic Ib Interneuron (`τ_ib_syn = 1.3 ms` + strictly proper membrane pole `p_ib ≈ 120–180 rad/s`, $D = 0$):** Filter efference-cancelled $\Delta\text{Ib}_i$ through 1 strictly proper spinal interneuron ODE state `ib_state_<muscle>` before subtracting at the Lamina IX $\alpha$-motoneuron pool.
   - **Ventral Root A-$\alpha$ Conduction (`τ_7a = 2.5 ms`, existing):** Total transport = `1.2 + 1.3 + 2.5 = 5.0 ms` + `~6.7 ms` interneuron + NMJ rise (`~12–15 ms` effective Ib reflex loop latency, matching human/feline physiology).

2. **Two-Layer Voluntary Efference-Copy Cancellation (Protecting `50°` RPY & `5-DOF` Shift):**
   - **Layer 1 — Parallel Reference Lin02 GTO (Mirroring `evaluateReferenceMileusnicSpindle` at `L2215`):**
     - From the delayed planned voluntary command `u_plan_alpha` (`L4615`, already computed for all 72 muscles from `vol_tau_6` & `L_hat, Lp_hat`), propagate a causal 1st-order expected muscle activation $\dot{\hat{a}}_{\text{vol},i} = (u_{\text{plan},\alpha,i} - \hat{a}_{\text{vol},i}) / \tau_{\text{act/deact}}$.
     - Predict expected voluntary force from cached muscle geometry:
       $$\hat{F}_{\text{vol},i} = F_{\max,i}\,\hat{a}_{\text{vol},i}\,f_L(\hat{L}_i)\,f_V(\dot{\hat{L}}_i)\,\cos\alpha_i$$
     - Run an identical 2-state Reference Lin02 GTO on $\hat{F}_{\text{vol},i}$ to obtain $\text{Ib}_{\text{ref},i}(t)$ and resting baseline $\text{Ib}_{0,i}$, giving expected bias $\text{Ib}_{\text{exp},i} = \text{Ib}_{0,i} + (\text{Ib}_{\text{ref},i} - \text{Ib}_{\text{ref},0,i})$.
     - Net unexpected external force signal: $\Delta\text{Ib}_i = \text{applyBiologicalDeadband}\!\bigl(\text{Ib}_{\text{meas},i}(t - \tau_{\text{ib}}) - \text{Ib}_{\text{exp},i}(t),\; \theta_{\text{ib}}\bigr)$.
   - **Layer 2 — Voluntary-Gated Presynaptic Ib Modulation (Zero-Ground-Truth Smooth Gate):**
     - During strong active voluntary turns (`voluntary_activity_gate -> 1.0`, `L4496`) or high positive voluntary force matches, modulate homonymous autogenic Ib gain smoothly (`1.0 - K_ib_vol_suppress * voluntary_activity_gate`, biologically reflecting state-dependent Ib reflex reversal / gating during voluntary motion while keeping full negative force-rate braking at `g_vol = 0` impacts).
   - **Lamina IX Pool Subtraction & Noise/Cuneate Synergy:**
     - Subtract `K_ib * ib_out[m]` at the alpha pool (`L4598`) alongside `u_qp + ccr_inh` (`enable_gto_reflex` boolean flag, `default=false` for clean A/B comparison).
     - Expand `BiologicalNoiseBank` from `24 + 3N` to `24 + 4N` (adding `72` Group Ib noise channels at `tau = 0.020 s` when `enable_sensor_noise=true`).
     - *(Bonus)* Use Cuneate/dorsal-column delayed `Ib` (`3.0 ms`) as an option to replace `act_m_now` in `computeTendonCompensatedMtuLength` (`L2275`), making Cuneate tendon-stretch compensation **100% pure zero-ground-truth receptor-based**.

---

### 2.3 Phase B1.2 — Renshaw Cell Recurrent Inhibition (`Sub 6` Lamina IX)

- **Literature:** Windhorst (1996, *Prog. Neurobiol.* 49:517–587); Hultborn et al. (1979). Cervical motor nuclei contain dense Renshaw cell networks receiving $\alpha$-motoneuron axon collaterals and feeding back recurrent inhibition over `~15–40 ms`.
- **Direct Motivation from Our Stage A Findings:** In Way C/D, lifting the ad-hoc canal detuning clamps (`highg_p_sc_floor = 22.0`, `highg_tau_lead_add_s = 0.022`) exposed a `~28 Hz` canal-neck limit cycle (`nPk = 3–4`). Renshaw recurrent inhibition directly damps `15–35 Hz` motor-pool ringing and bounds abrupt high-rate descending bursts **using textbook spinal microanatomy**.
- **Equations (Strictly Proper, Zero Ground Truth):**
  - Input: excess descending+reflex alpha excitation $\Delta u_{\text{pool},i}(t - \tau_{\text{ren,syn}})$ (`τ_ren_syn = 2.0 ms` disynaptic recurrent loop).
  - State: 1 ODE state per muscle (`ren_state_<m>`) with decay $\tau_{\text{ren}} \in [0.015, 0.035]\text{ s}$:
    $$\dot{r}_i = \frac{-r_i + \max(0, u_{\text{alpha,out},i}(t - \tau_{\text{ren,syn}}) - u_{\min,i})}{\tau_{\text{ren}}}$$
  - Pool Inhibition (with optional homonymous + moment-arm-weighted synergist pool spread $W_{\text{syn}}$):
    $$u_{\text{net},i} = \text{clamp}\!\left(u_{\text{qp},i} + \gamma_{\text{inh},i} u_{\text{ccr},i} - K_{\text{ib}} u_{\text{ib},i} - K_{\text{ren}} r_i,\; u_{\min,i},\; 1\right)$$
- **Payoff:** Allows **clean deletion** of `highg_p_sc_floor` and `highg_tau_lead_add_s` when paired with Tier A (`z_phas → 0` + LN saturation).

---

### 2.4 Phase B1.3 — Intrinsic Short-Range Muscle Stiffness (SRS) (`proprioception_plugin`)

- **Literature:** Rack & Westbury (1974, *J. Physiol.*); De Groote, Allen & Ting (2017, *PLoS Comput. Biol.* — showing Hill models without SRS are forced to inflate neural derivative gains to survive rapid postural perturbations).
- **Why SRS Specifically Breaks Our Causal Envelope Bottleneck:**
  - In **`Bode_APEO` (`3–8 Hz`)**: head rotations and cervical fiber strains are tiny (`< 1–2% L_0`). Attached actin–myosin cross-bridges resist stretch with **zero neural transport delay**, directly stabilizing the `3–8 Hz` band and removing the pressure on `K_phas * z_phas`!
  - In **`Sled 2g/3g` & `50°` Voluntary RPY**: once fiber excursion crosses cross-bridge critical strain ($\delta_{\text{yield}} \approx 0.015\text{–}0.025\,L_0$), SRS detaches smoothly (`C^1` yield ODE) — zero artificial impediment to NBDL `33–47°` flexion or full `50°` ROM!
- **Implementation Architecture (`Millard12EqMuscleWithAfferents` in `osimMillard12EqWithAff.dll`):**
  - Add 1 ODE state per active neck muscle: cross-bridge attachment anchor $\tilde{L}_{\text{anc}}$:
    $$\dot{\tilde{L}}_{\text{anc}} = \frac{\tilde{L}_m - \tilde{L}_{\text{anc}}}{\tau_{\text{detach}}(\Delta \tilde{L})}, \qquad \tau_{\text{detach}}(\Delta\tilde{L}) = \frac{\tau_{\text{slow}}}{1 + (\Delta \tilde{L} / \delta_{\text{yield}})^4}, \qquad \Delta\tilde{L} = \tilde{L}_m - \tilde{L}_{\text{anc}}$$
  - Add active-proportional cross-bridge SRS fiber force inside the Hill equilibrium / `computeForce` (gated smoothly by `enable_SRS` & gain `k_srs`):
    $$F_{\text{SRS}} = k_{\text{SRS}}\,a\,F_{\max}\,\frac{\Delta\tilde{L}}{1 + (\Delta\tilde{L}/\delta_{\text{yield}})^2}$$
  - Strictly $C^\infty$ smooth, passive at rest ($\Delta\tilde{L}(0) = 0$), and zero neural parameter bloat.

---

## 3. Plan 3 — Tier B2: Anatomical Cervical Ligaments, Facet Capsules & Ligamento-Muscular Reflexes

### 3.1 Audit of Current Passive Cervical Mechanics (`HYOID_HeadNeckNeural_Model_Detailed.osim`)
Inspecting [HYOID_HeadNeckNeural_Model_Detailed.osim](../osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim#L9990-L10117) and [`L11894–11989`](../osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim#L11894-L11989) reveals:
1. Passive intervertebral mechanics currently consist purely of **8 linear `BushingForce` elements** (`Spine_C7_Bushing` through `C1_skull_Bushing`) with symmetric rotational stiffness (`27.5 / 23.2 / 7.8 N·m/rad`), **zero translational stiffness (`0 0 0`)**, zero directional flexion/extension asymmetry, and zero physiological toe-region.
2. End-range limits are lumped into **6 coordinate-level `CoordinateLimitForce` elements** (`pitch1..yaw2`) that apply $0\text{ N·m}$ throughout Zone I (`< 75%` ROM) and emit no biological sensory signals.
3. Crucially, all 8 vertebral bodies (`spine`, `cerv7`, `cerv6`, `cerv5`, `cerv4`, `cerv3`, `cerv2`, `cerv1`, `skull`) already exist in the OpenSim `BodySet` (`L36–530`) with `CustomJoint` kinematics, so **real 3D anatomically routed ligaments and bilateral facet capsules can attach directly between adjacent vertebrae**.

### 3.2 Neuroanatomical & Biomechanical Literature Basis
- **Cervical Ligament Mechanics & Laxity:** *Yoganandan, Kumaresan & Pintar (2000, 2001)*; *Ivancic et al. (2004)*; *Panzer & Cronin (2009)*; *Mattucci, Moulton, Chandrashekar & Cronin (2012)*; *Nightingale et al. (2002, 2007)*.
- **Facet Capsule Mechanoreceptors & Ligamento-Muscular Reflexes:**
  - *McLain (1994), Spine* — Identified **Type I (Ruffini, slow posture/strain)**, **Type II (Pacinian, dynamic strain-rate $\dot{\varepsilon}$)**, **Type III (Golgi-like high-strain protective)**, and Type IV receptors in human cervical facet joint capsules.
  - *Solomonow et al. (1998, 2004), J. Electromyogr. Kinesiol.*; *Kallakuri et al. (2004)*; *Siegmund et al. (2008), Spine* — Stretch/shear of joint capsules & spinal ligaments elicits protective ligamento-muscular reflexes in surrounding stabilization musculature and flags sub-failure capsule strains at `~20–35%`.
  - *Richmond & Abrahams (1979)* & *Kulkarni et al. (2001)* — Extreme spindle density (`190–242/g`) in C0–C2 suboccipital muscles monitoring upper-cervical shear (`t_x`).

### 3.3 Implementation Phases

#### Phase B2.1 — Passive Stiffness Partitioning & Zone-I Equivalence Rule (Zero Double-Counting)
1. **Do NOT stack full ligament stiffness blindly onto existing full bushings.** Partition intersegmental resistance cleanly across `C0–T1`:
   - Retain reduced `BushingForce` elements strictly to represent **Intervertebral Disc (IVD) + uncovertebral/neutral-zone core mechanics** (small-angle neutral zone), plus physiologic IVD translational/shear stiffness rather than `0 0 0`.
   - Transfer nonlinear large-strain directional resistance into anatomical ligaments & facet capsules.
   - Replace `CoordinateLimitForce` Zone II–III (keeping Zone IV bony impingement stops near `120%` ROM strictly as extreme-crash rigid contact guards).
2. **Calibration Criterion (Small-Angle Neutral-Zone Match):** Compute the static sagittal, coronal, and axial moment–angle curves across `±6°` (`Bode_APEO` / `LatEC` / `YawMA` operating amplitude) and verify total passive joint stiffness matches the calibrated `IVD+Ligament` small-angle slope so `Bode_APEO` neutral-zone dynamics are preserved without artificial stiffening.

#### Phase B2.2 — New OpenSim `CervicalLigamentCapsuleForce` Component (`proprioception_plugin`)
Implement a custom $C^2$-smooth, zero-ground-truth-compatible `OpenSim::Force` subclass inside `osimMillard12EqWithAff.dll`:
1. **3D Anatomical Routing Per Segment (`C0–C1` through `C7–T1`):**
   - **ALL** (Anterior Longitudinal Ligament) — slack near neutral/flexion, resists **extension**.
   - **PLL** (Posterior Longitudinal Ligament) — resists **flexion** + canal shear.
   - **LF** (Ligamentum Flavum) — slight physiological resting pre-strain (`~3–5%`), posterior elastic flexion stabilizer.
   - **Bilateral Facet Joint Capsules (`CL_L`, `CL_R`)** — obliqued along true articular facet plane (~`45°`), resisting flexion, axial rotation, lateral bending, and **anterior–posterior intervertebral shear** (critical for sled impacts).
   - **ISL / SSL & Ligamentum Nuchae** — posterior spinous-process/occipital straps providing progressive large-flexion stiffening.
   - **Upper Cervical (`C0–C2`) Alar & Transverse/Cruciate Ligaments** — controlling coupled `yaw1`/`roll1` and atlanto-dental stability.
2. **$C^2$-Smooth Nonlinear Constitutive Force Law (No Integration Steps at Slack Length $\varepsilon = 0$):**
   $$f(\varepsilon) = \begin{cases} 0, & \varepsilon \le 0 \\ \dfrac{k_{\text{lin}}\,\varepsilon^3}{3\,\varepsilon_{\text{toe}}^2}, & 0 < \varepsilon < \varepsilon_{\text{toe}} \\ k_{\text{lin}}\!\left(\varepsilon - \dfrac{2}{3}\varepsilon_{\text{toe}}\right), & \varepsilon \ge \varepsilon_{\text{toe}} \end{cases}$$
   Wait — notice both force and stiffness $\partial f/\partial \varepsilon$ are strictly **$C^1$-smooth** ($f(0)=f'(0)=0$, $f(\varepsilon_{\text{toe}}) = \frac{1}{3} k_{\text{lin}}\varepsilon_{\text{toe}}$, $f'(\varepsilon_{\text{toe}}) = k_{\text{lin}}$; optionally quintic spline or softplus-quartic for full $C^2$!) plus strain-gated Kelvin-Voigt viscoelastic damping $c_{\text{visc}}\,\text{smoothstep}(\varepsilon/\varepsilon_{\text{toe}})\,\dot{\varepsilon}$.
3. **Mechanoreceptor Outputs (`OpenSim_DECLARE_OUTPUT`, Purely Local Strain/Strain-Rate):**
   - `strain` ($\varepsilon = (L - L_0)/L_0$), `strain_rate` ($\dot{\varepsilon} = \dot{L}/L_0$)
   - `ruffini_rate` (Type I: slowly adapting, low-threshold continuous tension envelope above resting pre-strain)
   - `pacinian_rate` (Type II: rapidly adapting shock/shear detector, high-passed $|\dot{\varepsilon}|$ burst above threshold)
   - `golgi_lig_rate` (Type III: high-strain protective recruitment past `ε_prot` — **strictly zero inside `< 6°` `Bode_APEO` neutral zone**!).

#### Phase B2.3 — Re-Equilibration of the 72-Muscle Resting `u_min` Vector
Because `LF` and resting neutral-zone capsules subtly adjust upright cervical preload, re-run the **6-DOF Bounded Minimum-Norm Co-Contraction Centering** ([neuromotor_control_architecture_c4.md](neuromotor_control_architecture_c4.md) Section 6.1) so all 72 `u_min` remain strictly within literature `%MVC` intervals with `|q̈_6DOF(0)| < 10^{-11} rad/s^2`.

#### Phase B2.4 — Ligamento-Muscular Reflex & 5-DOF Suboccipital Shear Stabilization (`Sub 1C → Sub 6`)
1. **Delay & Strictly-Proper Synaptic Pathway (`HeadNeckNeuralController`):**
   - Group II/III articular afferents (`30–70 m/s`) → `τ_art_axon = 3.0 ms` in `HistoryRecord` ring buffer → polysynaptic spinal interneuron filter (`τ_art_syn ≈ 8–12 ms`, strictly proper $D = 0$).
2. **Anatomical Recruitment Synergies (Per-Receptor, Per-Level Mapping):**
   - Posterior facet capsules / `PLL` / `LF` / `ISL` high strain & high strain-rate (`Type II + III`) → **facilitate synergistic cervical extensors** (`splen_cap`, `semi_cap`, `semi_cerv`, `longissi`, helping build clean flexion-arrest **Peak 2** in `2g/3g` and future `8g/15g` impacts!).
   - Anterior `ALL` hyperextension strain → facilitate anterior flexors (`stern_mast`, `longus_cap`, `longus_colli`).
   - Bilateral `Alar` / unilateral `CL` yaw-roll asymmetry → contralateral rotational/lateral stabilizers.
3. **Voluntary Efference-Copy Gating (`50°` RPY Protection):**
   - Gate Type I/III static-strain protective reflexes smoothly via `voluntary_activity_gate` and predicted voluntary strain from `plan_pos` so planned slow `50°` voluntary RPY reaches full ROM freely, while rapid unexpected impacts (`Sled 2g/3g/8g/15g`, high $\dot{\varepsilon}$) fire full protective stabilization — with **zero** disturbance inside `Bode_APEO` Zone I.
4. **Companion 5-DOF Suboccipital Shear Coupling (`t_x` & S-Buckling Control):**
   - Couple our already-decoded `cuneate_posture_shift_m` / `cuneate_diff_speed` (plus high-spindle suboccipitals `rectcap_post_*`, `obl_cap_*`) into dorsal-horn stabilization to suppress upper-cervical S-curve retraction whip (`-0.78°` at `70 ms`) before flexion develops.

#### Phase B2.5 — Full 4-Suite + Clinical Capsular Injury Verification
1. Verify all 4 Noise-OFF benchmark suites (`Bode 3-Axis`, `2g/3g Sled`, `18 s 50° RPY`, `5-DOF C1 Shift`) + multi-seed Noise-ON.
2. Add biomechanical **Facet-Capsule Injury Margin Report**: log peak `CL` tensile/shear strain across `C0–C1..C7–T1` against human sub-failure / nociceptive strain thresholds (`~20–35%` strain; *Kallakuri et al., 2004; Siegmund et al., 2008*), readying the model directly for high-G `8g` and `15g` *Zheng et al. (2021)* validation!

---

## 4. Recommended Execution Order & Cross-Plan Synergy

| Step | Work Package | Primary Payoff | Clean Rollback / Ablation Flag |
| :---: | :--- | :--- | :--- |
| **1** | **Phase B1.0 (30 s Probe)** | Measures exact `Bode_APEO` vs `Sled 2g` separation on tendon force $F_T$, strain `%L0`, and 5-DOF shear before touching C++. | Read-only Python script |
| **2** | **Tier A + Phase B1.3 (SRS) Co-Evaluation** *(or Tier A alone first)* | Fixes `z_phas` double-lead + `g_highg` cleft at the root; SRS lets `Bode_APEO` hold high-frequency gain/phase without neural lead bloat. | `z_phas`, `phasic_oto_sat_mps2`, `enable_SRS` |
| **3** | **Phase B1.1 & B1.2 (GTO Reuse + Renshaw)** | Activates repaired 2-state `Lin02GolgiTendonOrgan` (`1.2 ms` axon + Reference GTO efference cancellation) + Renshaw `28 Hz` damping (deletes `p_sc_floor`/`tau_lead_add` hacks). | `enable_GTO`, `enable_gto_reflex`, `enable_renshaw` |
| **4** | **Plan 3 — Tier B2 (Ligaments & Capsules)** | Replaces black-box limits with 3D `C0–T1` ligaments & facet capsules + Type I/II/III ligamento-muscular reflexes + clinical strain metrics for `2g–15g`. | `enable_ligament_reflex`, IVD/Ligament `.osim` preset |
| **5** | **Stage B (`Noise ON`) & Stage C (`Defaults`)** | Multi-seed stochastic validation across all 4 suites, followed by setting `enable_sensor_noise=true` & `enable_motor_noise=true` defaults. | `noise_scale`, `noise_seed` |

**Mandatory Engineering Hygiene Across Every Stage:**
- Keep `HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll` == root `osimHeadNeckNeural.dll` and `proprioception_plugin/build/Release/osimMillard12EqWithAff.dll` == root `osimMillard12EqWithAff.dll` (identical SHA-256).
- Keep [neuromotor_block_explorer.html](neuromotor_block_explorer.html) == root `neuromotor_block_explorer.html` (identical SHA-256).
- Always count `n_splen_peaks` using the exact ground-truth rule in [run_zheng_2g_3g_urgency_sweep.py](../scripts/run_zheng_2g_3g_urgency_sweep.py#L378-L386).
