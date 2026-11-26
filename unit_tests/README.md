# Head-Neck Neurobiomechanical 21-Block Functional Unit Test Suite

Every block in the musculoskeletal plant and neural controller has an isolated functional unit test with a **pre-specified analytical/biophysical expected outcome** and direct probing of its internal OpenSim `Output<T>` and `StateVariable` signals.

## How to Run

```powershell
# Run ALL 21 blocks in parallel -> generates JSON report + 3 PNG diagnostic figures
C:\Users\User\miniforge3\envs\opensim_env\python.exe unit_tests/run_all.py --workers 8

# Run a single layer (plant | vestibular | observers | brainstem | cortex | spinal)
C:\Users\User\miniforge3\envs\opensim_env\python.exe unit_tests/run_all.py --layer brainstem

# Run specific block(s)
C:\Users\User\miniforge3\envs\opensim_env\python.exe unit_tests/run_all.py --block B08 B12 B14
```

### Generated Artifacts (`unit_tests/results/`)
- `unit_test_report.json` — machine-readable pass/fail results for all 21 blocks & 69 assertions
- `unit_test_scorecard.png` — visual 21-block pass/fail scorecard
- `B12_mahony_detailed.png` — 6-panel Mahony 3D coupled tilt (`oto_heavy` vs `prop_only`), attitude error, and `20 m/s²` AP/Lat linear acceleration rejection waveforms
- `key_blocks_waveforms.png` — internal waveforms for `B03` (SRS thixotropy), `B05` (Lin02 GTO ODE match), `B07` (Velocity Storage), and `B14.4` (Phasic VCR Pitch & Roll sign vs passive whiplash)

## OpenSim 4.6 Unit-Testing Pattern (`common.py`)
- Blocks that test **vestibular/cortical pathways with fixed neck** use `lock_neck_except=()`.
- Blocks that test **72-muscle Jacobian decoding, CCR, or anti-buckling (`B10`, `B19`, `B20`)** use `spring_targets` (`SpringGeneralizedForce`) or `external_torques` (`CoordinateActuator`) on the **untouched free-neck production topology** so OpenSim 4.6's `PrescribedMotion` constraint never perturbs `MomentArmSolver`.

## 21-Block Catalog

| Layer | Block | Component / Subsystem | Key Pre-Specified Verifications |
|---|---|---|---|
| **plant** | `B01` | 6-DOF Passive Equilibrium & 72-Muscle `u0` | `max \|qddot(0)\| < 1 deg/s²`, `1s` drift `< 0.25°`, L/R symmetry `< 0.05°` |
| **plant** | `B02` | Kuo (2019) `DynamicLigamentForce` | Slack at `strain <= 1`, analytical bilinear stress-strain `±5%`, log rate stiffening |
| **plant** | `B03` | Cross-Bridge SRS & Rest-Gated Thixotropy | Positive elastic force jump at tonic `a0`, oscillation breakdown `< 0.70`, rest recovery |
| **plant** | `B04` | `Mileusnic06Spindle` (`Ia`, `II`, $\gamma_d, \gamma_s$) | `Ia` velocity burst, `II` static length, $\gamma_d \to Ia$ only, active $\alpha\text{-}\gamma_s \to II$ |
| **plant** | `B05` | `Lin02GolgiTendonOrgan` (`Ib`) | Monotonic force rise, dynamic overshoot, exact 2-pole ODE match (`< 3%`, `RMS < 3 imp/s`) |
| **vestibular** | `B06` | 3D Semicircular Canals (Reg + Irr) | Exact `+20°/s` sign & passband gain on Roll(X), Yaw(Y), Pitch(Z) |
| **vestibular** | `B07` | Canal Luenberger + Velocity Storage | `T_c=5.7s` cupula decay & exact coupled VS ODE restoration (`±5%`) |
| **vestibular** | `B08` | 3D Otolith Maculae + Luenberger | Exact `f_GIF = R_W^H(a - g)` signs (`+Pitch -> +X`, `+Roll -> -Z`), `+X/+Z` accel lead/adapt |
| **vestibular** | `B09` | Hill-4 Somatogravic Reweight & Hair Tanh | `w_oto_rw` at `2 / 8 / 20 m/s²` matches `1/(1+(a/a0)^4)`, sublinear hair tanh |
| **observers** | `B10` | 72-Muscle 6-DOF Decoder & Cuneate | Static `+10°` Pitch/Roll/Yaw decoded within `±12%`, cross-axis error `< 1.0°` |
| **observers** | `B11` | Reference Spindle & Fusimotor Cancel | Active `+15°` Pitch suppresses `est_neck_pitch` RMS by `> 70%` vs OFF |
| **observers** | `B12` | Cerebellar `SE_2(3)` Mahony Observer | `< 1.5°` 3D tilt error (oto-heavy & prop-only), `< 0.30°` false tilt under `20 m/s²` |
| **observers** | `B13` | Cerebellar Gravity Adaptation (`Sub 3C`) | `> 97%` gravity cancellation (`< 0.08 m/s²`) in `dbg_phasic_oto_input` on `20°` Pitch & Roll |
| **brainstem** | `B14` | Brainstem VCR (Tonic, `Ki_ton`, Canal, Phasic) | Restoring sign & gain on all 3 axes; empirical Phasic sign opposes passive whiplash on Pitch & Roll |
| **brainstem** | `B15` | Branch Proper Lead Compensators | Zero `t=0` kick; restores phase at `2.0 Hz` vs Lead OFF |
| **cortex** | `B16` | 3rd-Order Trajectory Planner | `0.00%` overshoot, exact steady state, analytical `T_95(u)` compression |
| **cortex** | `B17` | Continuous Urgency Gate & Zero `u=0` Leak | Strictly `< 1e-9 Nm` voluntary & shift torque at `u=0`; smooth monotonic `u in [0,1]` |
| **cortex** | `B18` | 2x2 Weighted Shift IK (AP / Lat Glide) | `+2cm` AP & `+0.5cm` Lat C1 shift while keeping skull level (`< 1.5°`) |
| **spinal** | `B19` | Spinal/Task CCR & Postural Reciprocal Inhib | Passive restoring CCR on all 3 axes; `vol_gate > 0.95` sustained during static holds |
| **spinal** | `B20` | Direct Passive Anti-Buckling & 6-DOF QP | S-buckling `tau_buckle` matches `Kp*(p1-p2)` within `15%`; QP tracking `< 5%` |
| **spinal** | `B21` | Tier B Spinal Loops (GTO, Renshaw, Capsule) | GTO & Renshaw inhibit activation; `lig_ref_0` silent at `0°`, active at `-45°` |

## Real C++ Bugs Already Caught & Fixed by This Suite
1. **`B14.4_phasic_sign_Roll_Lat` (Inverted Roll Phasic Otolith VCR Sign):** Under `+5 m/s² spine_tz`, the passive head whipped to `-5.68°` while Roll Phasic VCR fired `-17.10 Nm` (assisting whiplash!). Fixed `phasic_otolith_input_roll = -(arriving_otolith_specific_force_head[2] - arriving_eff_phasic_grav_acc[1])` so Roll Phasic fires `+17.10 Nm` opposing whiplash while preserving `99.47%` static gravity cancellation (`B13`).
2. **`B17.3_smooth_monotonic_u` (Ungated Voluntary Feedforward Leak at `u=0`):** `head_inertia * traj_acc + kv_trans * obs_v + kp_trans * obs_p` leaked `+0.06 Nm` at `u=0` when `desired_pitch != 0`. Gated by `vol_cmd_gate[axis]` so `vol_torque(u=0) == 0.000e+00 Nm` unconditionally.
3. **`B10`/`B19`/`B20` (Locked-Coordinate Jacobian Initialization Corruption):** `initializeBaseline` and `exact_jacobian_6dof` now compute `computeMomentArm` on an unlocked/unprescribed `work_state` copy, preventing locked DOFs from corrupting `J6` or emitting OpenSim `Coordinate.setValue` warnings.
