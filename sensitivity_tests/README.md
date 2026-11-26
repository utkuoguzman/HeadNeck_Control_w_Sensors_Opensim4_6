# Full-Model Block Sensitivity & Internal-Probe Suite (`sensitivity_tests/`)

Applies the same direct internal probing culture as `unit_tests/` to full-model Bode frequency-response sensitivity analysis (`APEO`, `LatEC`, `YawMA` — **all subsystems strictly ON**).

## Usage
```powershell
# Run full-model OAT block sensitivity on any axis (8 local workers):
python sensitivity_tests/run_sensitivity.py --axis APEO  --workers 8
python sensitivity_tests/run_sensitivity.py --axis LatEC --workers 8
python sensitivity_tests/run_sensitivity.py --axis YawMA --workers 8

# Sweep a specific subset of parameters or use exact 15.8 s window:
python sensitivity_tests/run_sensitivity.py --axis APEO --subset K_sc,K_phas,__bushing_k__,__bushing_c__ --workers 8
```

## Outputs (`sensitivity_tests/results/`)
- `sensitivity_<axis>.json`: Full per-frequency Gain/Phase error, upper/lower joint split (`q1`/`q2`), internal C++ torque budget (`dbg_vcr_torque`, `dbg_ccr_torque`, `dbg_buckle_torque`, `dbg_qp_delivered_3dof`), SRS thixotropy, and ranked sensitivity table.
- `sensitivity_<axis>_grid.png`: Happee (2017) Fig. 5-style multi-panel Bode sensitivity plot.
- `torque_budget_<axis>.png`: Per-frequency internal torque budget & `q1` vs `q2` S-bend X-ray.
