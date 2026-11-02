# 15-Parameter, 6-DOF, 72-Muscle MIMO Differentiable Neuromotor Surrogate Pipeline

An analytical, physics-grounded 6-DOF, 72-muscle MIMO frequency-domain surrogate modeling toolchain for pre-optimizing all 15 closed-loop neuromotor parameters (Vestibulocollic VCR, Cervicocollic CCR, $\alpha\text{–}\gamma$ fusimotor coactivation, and Cortical Task PID) before launching multi-generation non-linear CMA-ES campaigns in OpenSim 4.6.

---

## 📌 Why Upgrade to the 6-DOF, 72-Muscle MIMO Surrogate?

A 1-DOF scalar inverted-pendulum surrogate cannot capture the inter-segmental dynamics of the 8-vertebra cervical spine (`pitch1` C7–C2 vs. `pitch2` C2–skull, plus `roll1`, `roll2`, `yaw1`, `yaw2`) or the unilateral recruitment of the 72 HYOID neck muscles:
1. **Asymmetric Inertial vs. Active Muscle Coupling:** Passive horizontal sled acceleration ($\mathbf{G}_{\text{sled},6}$) drives upper cervical `pitch2` (`+3.585 rad/m`) almost $9\times$ more strongly than lower cervical `pitch1` (`+0.405 rad/m`), whereas the 3-DOF Active-Set QP muscle synergy solver ($\mathbf{B}_{\text{QP},6\times 3}$) applies $+35.50\,\text{rad/s}^2/\text{N}\cdot\text{m}$ to `pitch1` and $-9.24\,\text{rad/s}^2/\text{N}\cdot\text{m}$ to `pitch2`, coupled dynamically through the series-elastic inter-vertebral muscle-tendon stiffness matrix $\mathbf{A}_{\text{stiff,inst},6\times 6}$.
2. **Maxwell-Hill Series-Elastic 6-DOF Plant Impedance:** Combines the exact 6-DOF multibody mass matrix $\mathbf{M}_6$, static equilibrated stiffness $\mathbf{A}_{\text{stiff,eq},6\times 6}$, instantaneous series-elastic tendon stiffness $\mathbf{A}_{\text{stiff,inst},6\times 6}$, and 72-muscle Hill force-velocity damping $\mathbf{A}_{\text{damp,inst},6\times 6}$ anchored to the empirical OpenSim 6-DOF passive response (`surrogacy/opensim_passive_plant.json`).
3. **Full 15-Parameter Closed-Loop Architecture:** Evaluates all 15 CMA-ES parameters (`[K_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat, Kp_task_pitch, Kd_task_pitch, Ki_task_pitch]`) across the Schneider (2015) semicircular canals, Schneider (2015) / Happee (2017) 2-stage otoliths with matched-adaptation gravity cancellation, 72 Mileusnic (2006) spindles, Mahony $\text{SE}_2(3)$ observer with somatogravic tilt coupling, and Paradigm A 6-DOF joint-limit safety constraints.

---

## 🛠️ Pipeline Architecture

```
OpenSim 4.6 Model (HYOID 6-DOF, 72-Muscle)
    │
    ├──► extract_6dof_72muscle_plant.py (~4s)
    │        └──► opensim_6dof_72muscle_plant.json (M_6, J_6x72, A_3x72, B_musc_6x72, A_stiff_inst_6x6, A_damp_inst_6x6)
    │
    ├──► measure_passive_plant.py (~40s in parallel)
    │        └──► opensim_passive_plant.json (Exact 6-DOF Co-Contracted Passive Frequency Response)
    │
    ▼
optimize_surrogate.py (~5s)
    └──► exact_plant_surrogate_solution.json (Optimal 15D Seed x0* + 6-DOF Predictions)
    │
    ▼
validate_in_opensim.py (~45s in parallel)
    └──► Full 6-DOF Non-Linear OpenSim 4.6 Validation Table
```

---

## 🚀 Quick Start Guide

### Step 1a: Extract the 6-DOF, 72-Muscle State-Space & Jacobian Matrices
```powershell
& "C:\Users\User\miniforge3\envs\opensim_env\python.exe" "surrogacy/extract_6dof_72muscle_plant.py"
```

### Step 1b: Measure the Exact 6-DOF Co-Contracted Passive Plant
```powershell
& "C:\Users\User\miniforge3\envs\opensim_env\python.exe" "surrogacy/measure_passive_plant.py"
```

### Step 2: Solve for the Optimal 15D Seed ($x_0^*$) via the 6-DOF MIMO Surrogate
```powershell
& "C:\Users\User\miniforge3\envs\opensim_env\python.exe" "surrogacy/optimize_surrogate.py"
```

### Step 3: Validate in the Full Non-Linear 6-DOF OpenSim 4.6 Model
```powershell
& "C:\Users\User\miniforge3\envs\opensim_env\python.exe" "surrogacy/validate_in_opensim.py"
```
