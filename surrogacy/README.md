# Differentiable Neuromotor Surrogate Pipeline

A fast, analytical surrogate modeling toolchain for pre-optimizing closed-loop neuromotor reflex parameters (Vestibulocollic VCR, Cervicocollic CCR, and transport delays) before launching multi-hour non-linear CMA-ES campaigns in OpenSim.

---

## 📌 Why Use the Surrogate?

Simulating 18 candidates across 6 perturbation frequencies in OpenSim takes **$\approx 45\text{--}60\text{ minutes per generation}$**. Starting CMA-ES from an uncalibrated or randomly guessed initial vector ($x_0$) requires dozens of generations just to locate the basin of attraction.

This surrogate pipeline acts as a **"quick-and-dirty initialization finding step"**:
1. **Measures the exact OpenSim multi-body plant impedance** ($G_{\text{passive}}(j\omega)$) across the 6 canonical frequencies directly from the full 8-vertebra HYOID model (taking only 90 seconds once).
2. **Evaluates closed-loop feedback analytically** using Black's feedback formula with exact continuous transport delays ($e^{-s\tau}$) in $< 1\ \mu\text{s}$.
3. **Runs multi-start L-BFGS-B gradient descent in under 10 seconds** to find an elite starting vector ($x_0$), reducing the initial OpenSim error score from $>176\text{k}$ down to $<75\text{k}$ before CMA-ES starts.

---

## 🛠️ Pipeline Architecture

```
OpenSim Model (HYOID) 
    │
    ▼ (measure_passive_plant.py — ~90s)
opensim_passive_plant.json (Exact Multi-Body Plant Frequency Response)
    │
    ▼ (optimize_surrogate.py — ~7s)
exact_plant_surrogate_solution.json (L-BFGS-B Optimal Reflex Parameters)
    │
    ▼ (validate_in_opensim.py — ~90s)
OpenSim 6-Frequency Validation Score (Elite Seed x0 for CMA-ES)
```

---

## 🚀 Quick Start Guide

### Step 1: Measure the Exact OpenSim Passive Plant
```powershell
python surrogacy/measure_passive_plant.py
```
* Runs 6 simulations in parallel with sensory reflexes set to zero and calibrated gravity trim ($K_i = 40.28$) loaded.
* Saves the exact complex frequency response table to `surrogacy/opensim_passive_plant.json`.

### Step 2: Solve for Optimal Reflex Parameters via Gradient Descent
```powershell
python surrogacy/optimize_surrogate.py
```
* Runs 24 multi-start L-BFGS-B gradient descents on the analytical transfer function.
* Completes in **$\approx 7\text{ seconds}$**.
* Saves the optimal parameter vector and ratios to `surrogacy/exact_plant_surrogate_solution.json`.

### Step 3: Validate in Full Non-Linear OpenSim Model
```powershell
python surrogacy/validate_in_opensim.py
```
* Evaluates the candidate across all 6 frequencies in OpenSim.
* Achieved benchmark: **`519.4 deg/m` (+0.27 dB) at 5.58 Hz (score: 162 pts)** and accurate physiological phase lag at 0.78 Hz and 1.10 Hz.

### Step 4: Seed the Main CMA-ES Campaign
The resulting vector in `exact_plant_surrogate_solution.json` is plugged directly into `scripts/run_phase2_3_campaign.py` as $x_0$, saving generations of computational effort.

