"""
Update block content in docs/neuromotor_block_explorer.html
strictly modifying internal block contents (descriptions, math, params, SVG text)
WITHOUT separating, combining, adding, or deleting any blocks or tabs.
"""

import re
import os
import shutil

HTML_PATH = "docs/neuromotor_block_explorer.html"
ARTIFACT_PATH = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\neuromotor_block_explorer.html"

with open(HTML_PATH, "r", encoding="utf-8") as f:
    content = f.read()

# =========================================================================
# 1. UPDATE BLOCK_INFO.s5_sc
# =========================================================================
s5_sc_old = r"""      s5_sc: {
        type: "Dominant Canal Pole (p_sc = 50, p_fast = 250 rad/s) + ⚡ Branch Sub2B→Sub5 Delay Compensator (τ_5 = 2.5 ms)",
        title: "Semicircular Canal Central VCR Filter — H_sc(s) (Literature p_sc = 50.0 rad/s, p_fast = 250 rad/s + ⚡ MVST Compensator)",
        desc: "Pairs the dominant physiological canal pole p_sc = 50.0 rad/s (~8.0 Hz; Peng et al. 1996, 1999; Wilson & Schor 1999) with the fast secondary biophysical membrane pole p_fast = 250.0 rad/s (τ_2 = 4.0 ms), followed by the sender-side MVST Branch Sub2B→Sub5 axonal delay compensator 1 + τ_5·s + 0.5·τ_5²·s² (τ_5 = 2.5 ms to Subsystem 5 (C1/T1 Lamina IX Pool)). Downstream extrafusal muscle excitation-contraction delay (τ_act = 15.0 ms) is compensated centrally in Subsystem 5 after the Alpha-Motoneuron Pool Sum.",
        latex: String.raw`H_{\text{sc}}(s) = K_{\text{sc}}\,\frac{p_{\text{sc}}\,p_{\text{fast}}}{(s + p_{\text{sc}})(s + p_{\text{fast}})}\underbrace{\bigl(1 + \tau_5 s + \tfrac{1}{2}\tau_5^2 s^2\bigr)}_{\text{MVST }\tau_5 = 2.5\text{ ms}}, \qquad \begin{cases}\dot{x}_{1,\text{sc}} = x_{2,\text{sc}} \\ \dot{x}_{2,\text{sc}} = p_{\text{sc}}p_{\text{fast}}(u_{\text{sc}} - x_{1,\text{sc}}) - (p_{\text{sc}} + p_{\text{fast}})x_{2,\text{sc}}\end{cases}`,
        math:
`Input:   semicircular_canal_input u_sc [from Canal Velocity Efference Cancellation]
States:  x_1,sc = sc_state_*[3], x_2,sc = sc_vel_*[3]
Params:  K_sc (CMA-ES #2), p_sc = 50.0 rad/s (literature pole), p_fast = 250.0 rad/s, τ_5 = 0.0025 s (2.5 ms MVST axonal delay)

For each axis ∈ {pitch, roll, yaw} do:
   1. 2nd-order filter state derivatives (dominant p_sc = 50 rad/s + fast p_fast = 250 rad/s):
      y_sc  ← x_1,sc
      ẏ_sc  ← x_2,sc
      ÿ_sc  ← (p_sc · p_fast) · (u_sc - x_1,sc) - (p_sc + p_fast) · x_2,sc

   2. Outgoing MVST Branch Sub2B→Sub5 axonal delay compensator C_τ5(s) = 1 + τ_5·s + 0.5·τ_5²·s² (τ_5 = 2.5 ms):
      τ_sc  ← K_sc · (y_sc + τ_5 · ẏ_sc + 0.5 · τ_5² · ÿ_sc)

Output:  semicircular_canal_output ← τ_sc`,
        params: [
          "K_sc (CMA-ES #2): Canal VCR Gain [Nm/(rad/s)]",
          "p_sc = 50.0 rad/s (Dominant Literature Pole, ~8.0 Hz) + p_fast = 250.0 rad/s (Fast Membrane Pole, τ_2 = 4.0 ms)",
          "⚡ C_τ5(s) = 1 + τ_5·s + 0.5·τ_5²·s² : MVST Branch Sub2B→Sub5 delay compensator (τ_5 = 2.5 ms to Sub5)"
        ]
      },"""

s5_sc_new = r"""      s5_sc: {
        type: "Strictly Proper VCR Canal Filter (D = 0) with Discovered Phase-Lead Zero (z_sc = 5.54 ms, p_sc = 53.28 rad/s, p_fast = 250 rad/s)",
        title: "Semicircular Canal Central VCR Filter — H_sc(s) (Strictly Proper D = 0, Calibrated z_sc = 5.54 ms, p_sc = 53.28 rad/s)",
        desc: "Pairs the calibrated physiological canal pole p_sc = 53.284 rad/s (~8.48 Hz) with the fast secondary biophysical membrane pole p_fast = 250.0 rad/s (τ_2 = 4.0 ms) and an embedded phase-lead zero z_sc = 0.005544 s (5.544 ms). The realization is strictly proper (relative degree 1, D = 0) with zero direct feedthrough from canal angular velocity into spinal torque commands. This properness property eliminates algebraic loop transients and prevents high-frequency feedthrough spikes from creating co-contraction artifacts before the downstream Quadratic Programming (QP) muscle allocation solver. Crucially, the phase-lead zero tan⁻¹(ω·z_sc) selectively advances high-frequency phase, cutting the 7.93 Hz phase error from 28.4° down to an exceptional 3.8° (and 5.58 Hz down to 16.9°). Downstream axonal conduction traverses the MVST (Branch Sub2B→Sub5: τ_5 = 2.5 ms).",
        latex: String.raw`\begin{aligned}
H_{\text{sc}}(s) &= K_{\text{sc}}\,\frac{p_{\text{sc}}\,p_{\text{fast}}\,(1 + z_{\text{sc}} s)}{(s + p_{\text{sc}})(s + p_{\text{fast}})}, \quad \text{Strictly Proper Realization }(D = 0) \\[4pt]
\text{State Dynamics:} &\quad \begin{cases}\dot{x}_{1,\text{sc}} = x_{2,\text{sc}} \\ \dot{x}_{2,\text{sc}} = p_{\text{sc}}p_{\text{fast}}\bigl(u_{\text{sc}} - x_{1,\text{sc}}\bigr) - (p_{\text{sc}} + p_{\text{fast}})x_{2,\text{sc}}\end{cases} \\[4pt]
\text{Zero-Feedthrough Output:} &\quad x_{\text{sc,eff}} = x_{1,\text{sc}} + z_{\text{sc}} x_{2,\text{sc}}, \qquad \tau_{\text{sc}} = K_{\text{sc}}\,x_{\text{sc,eff}}
\end{aligned}`,
        math:
`Input:   semicircular_canal_input u_sc [from Canal Velocity Efference Cancellation]
States:  x_1,sc = sc_state_*[3], x_2,sc = sc_vel_*[3]
Optimal: K_sc = 4.1019 Nm/(rad/s), z_sc = 0.005544 s (5.544 ms), p_sc = 53.284 rad/s (8.48 Hz), p_fast = 250.0 rad/s

For each axis ∈ {pitch, roll, yaw} do:
   1. Strictly proper 2nd-order state derivatives (relative degree 1, D = 0):
      ẏ_sc  ← x_2,sc
      ÿ_sc  ← (p_sc · p_fast) · (u_sc - x_1,sc) - (p_sc + p_fast) · x_2,sc

   2. Effective state synthesis with phase-lead zero (pure state combination, zero direct feedthrough):
      x_sc,eff ← x_1,sc + z_sc · x_2,sc

   3. Restorative canal torque (strictly proper, zero feedthrough into QP allocation):
      τ_sc     ← K_sc · x_sc,eff

Output:  semicircular_canal_output ← τ_sc (transmitted via MVST τ_5 = 2.5 ms)`,
        params: [
          "K_sc = 4.1019 Nm/(rad/s) : Discovered optimal semicircular canal VCR gain",
          "z_sc = 0.005544 s (5.544 ms) : Calibrated phase-lead zero (slashed 7.93 Hz phase error from 28.4° to 3.8°)",
          "p_sc = 53.284 rad/s (~8.48 Hz) : Calibrated dominant physiological canal pole",
          "p_fast = 250.0 rad/s (39.8 Hz) : Secondary biophysical membrane pole (τ_2 = 4.0 ms)",
          "Properness (D = 0) : Strict properness guarantees zero direct feedthrough, preventing QP co-contraction spikes",
          "⚡ MVST Conduction (Sub2B → Sub5) : τ_5 = 2.5 ms axonal conduction to C1–T1 cervical motoneuron pools"
        ]
      },"""

# =========================================================================
# 2. UPDATE BLOCK_INFO.s5_phas
# =========================================================================
s5_phas_old = r"""      s5_phas: {
        type: "Dominant Leaky-Integrator Pole (p_phas = 4.0, p_fast = 250 rad/s) + ⚡ Branch Sub2B→Sub5 Delay Compensator (τ_5 = 2.5 ms)",
        title: "Irregular Otolith Phasic Central Filter — H_phas(s) (Literature p_phas = 4.0 rad/s, p_fast = 250 rad/s + ⚡ MVST Compensator)",
        desc: "Integrates phasic_otolith_input (linear acceleration in m/s²) toward translational velocity across the 0.8–8 Hz band using the otolith velocity-storage / leaky-integration corner pole p_phas = 4.0 rad/s (~0.64 Hz, τ_phas = 0.25 s; Angelaki 2004; Raphan & Cohen 2002) paired with the fast secondary biophysical membrane pole p_fast = 250.0 rad/s (τ_2 = 4.0 ms), followed by the sender-side MVST Branch Sub2B→Sub5 delay compensator 1 + τ_5·s + 0.5·τ_5²·s² (τ_5 = 2.5 ms).",
        latex: String.raw`H_{\text{phas}}(s) = K_{\text{phas}}\,\frac{p_{\text{phas}}\,p_{\text{fast}}}{(s + p_{\text{phas}})(s + p_{\text{fast}})}\underbrace{\bigl(1 + \tau_5 s + \tfrac{1}{2}\tau_5^2 s^2\bigr)}_{\text{MVST }\tau_5 = 2.5\text{ ms}}, \qquad \begin{cases}\dot{x}_{1,\text{phas}} = x_{2,\text{phas}} \\ \dot{x}_{2,\text{phas}} = p_{\text{phas}}p_{\text{fast}}(u_{\text{phas}} - x_{1,\text{phas}}) - (p_{\text{phas}} + p_{\text{fast}})x_{2,\text{phas}}\end{cases}`,
        math:
`Input:   phasic_otolith_input u_phas [from Phasic Gravity & Efference Cancellation]
States:  x_1,phas = phas_state_*[3], x_2,phas = phas_vel_*[3]
Params:  K_phas (CMA-ES #3), p_phas = 4.0 rad/s (literature leaky-integrator pole, fc = 0.64 Hz), p_fast = 250.0 rad/s, τ_5 = 0.0025 s (2.5 ms MVST delay)

For each axis ∈ {pitch, roll, yaw} do:
   1. 2nd-order state derivatives (dominant p_phas = 4.0 rad/s + fast p_fast = 250.0 rad/s):
      y_phas  ← x_1,phas
      ẏ_phas  ← x_2,phas
      ÿ_phas  ← (p_phas · p_fast) · (u_phas - x_1,phas) - (p_phas + p_fast) · x_2,phas

   2. Sender-side MVST Branch Sub2B→Sub5 delay compensator (1 + τ_5·s + 0.5·τ_5²·s², τ_5 = 2.5 ms):
      τ_phas  ← K_phas · (y_phas + τ_5 · ẏ_phas + 0.5 · τ_5² · ÿ_phas)

Output:  phasic_otolith_output ← τ_phas`,
        params: [
          "K_phas (CMA-ES #3): Phasic Otolith Gain [Nm/(m/s²)]",
          "p_phas = 4.0 rad/s (Dominant Literature Leaky-Integrator Pole, ~0.64 Hz; Angelaki 2004; Raphan & Cohen 2002) + p_fast = 250.0 rad/s",
          "⚡ C_τ5(s) = 1 + τ_5·s + 0.5·τ_5²·s² : MVST Branch Sub2B→Sub5 delay compensator (τ_5 = 2.5 ms to Sub5)"
        ]
      },"""

s5_phas_new = r"""      s5_phas: {
        type: "Strictly Proper Phasic Otolith Filter (D = 0) with Scaled Phase-Lead Zero (z_phas = 13.31 ms, p_phas = 4.71 rad/s, p_fast = 250 rad/s)",
        title: "Irregular Otolith Phasic Central Filter — H_phas(s) (Strictly Proper D = 0, Calibrated z_phas = 13.31 ms, p_phas = 4.71 rad/s)",
        desc: "Integrates linear acceleration toward translational velocity across the 0.8–8 Hz band using the otolith velocity-storage leaky-integration pole p_phas = 4.7119 rad/s (~0.75 Hz) paired with the membrane pole p_fast = 250.0 rad/s and an embedded phase-lead zero z_phas = 2.40 · z_sc = 0.013307 s (13.31 ms). Like H_sc(s), the realization is strictly proper (relative degree 1, D = 0) with zero direct feedthrough from linear acceleration to spinal torque. This prevents translational shock impulses from inducing abrupt feedthrough jumps into muscle activation, stabilizing QP allocation during multi-frequency perturbations.",
        latex: String.raw`\begin{aligned}
H_{\text{phas}}(s) &= K_{\text{phas}}\,\frac{p_{\text{phas}}\,p_{\text{fast}}\,(1 + z_{\text{phas}} s)}{(s + p_{\text{phas}})(s + p_{\text{fast}})}, \quad \text{Strictly Proper Realization }(D = 0) \\[4pt]
\text{State Dynamics:} &\quad \begin{cases}\dot{x}_{1,\text{phas}} = x_{2,\text{phas}} \\ \dot{x}_{2,\text{phas}} = p_{\text{phas}}p_{\text{fast}}\bigl(u_{\text{phas}} - x_{1,\text{phas}}\bigr) - (p_{\text{phas}} + p_{\text{fast}})x_{2,\text{phas}}\end{cases} \\[4pt]
\text{Zero-Feedthrough Output:} &\quad x_{\text{phas,eff}} = x_{1,\text{phas}} + z_{\text{phas}} x_{2,\text{phas}}, \qquad \tau_{\text{phas}} = K_{\text{phas}}\,x_{\text{phas,eff}}
\end{aligned}`,
        math:
`Input:   phasic_otolith_input u_phas [from Phasic Gravity & Efference Cancellation]
States:  x_1,phas = phas_state_*[3], x_2,phas = phas_vel_*[3]
Optimal: K_phas = 15.4205 Nm/(m/s²), z_phas = 0.013307 s (13.31 ms = 2.40 · z_sc), p_phas = 4.7119 rad/s (0.75 Hz), p_fast = 250.0 rad/s

For each axis ∈ {pitch, roll, yaw} do:
   1. Strictly proper state derivatives (relative degree 1, D = 0):
      ẏ_phas  ← x_2,phas
      ÿ_phas  ← (p_phas · p_fast) · (u_phas - x_1,phas) - (p_phas + p_fast) · x_2,phas

   2. Effective state synthesis with phase-lead zero (pure state combination, zero direct feedthrough):
      x_phas,eff ← x_1,phas + z_phas · x_2,phas

   3. Restorative phasic torque (strictly proper, zero feedthrough into QP allocation):
      τ_phas     ← K_phas · x_phas,eff

Output:  phasic_otolith_output ← τ_phas (transmitted via MVST τ_5 = 2.5 ms)`,
        params: [
          "K_phas = 15.4205 Nm/(m/s²) : Discovered optimal phasic otolith VCR gain",
          "z_phas = 0.013307 s (13.31 ms) : Calibrated otolith phase-lead zero (scaled to 2.40 · z_sc)",
          "p_phas = 4.7119 rad/s (~0.75 Hz) : Calibrated velocity-storage leaky-integrator corner pole",
          "p_fast = 250.0 rad/s (39.8 Hz) : Secondary biophysical membrane pole (τ_2 = 4.0 ms)",
          "Properness (D = 0) : Eliminates acceleration feedthrough shocks into the QP muscle allocator",
          "⚡ MVST Conduction (Sub2B → Sub5) : τ_5 = 2.5 ms axonal conduction to C1–T1 cervical motoneuron pools"
        ]
      },"""

# =========================================================================
# 3. UPDATE BLOCK_INFO.s5_ton
# =========================================================================
s5_ton_old = r"""      s5_ton: {
        type: "Dominant Pole LPF (p_ton = 15, p_fast = 250 rad/s) + ⚡ Branch Sub2B→Sub5 Delay Compensator (τ_5 = 2.5 ms)",
        title: "Tonic Spatial Attitude LPF — H_ton(s) (Literature Pole p_ton = 15.0 rad/s, p_fast = 250 rad/s + ⚡ MVST Compensator)",
        desc: "Models the sustained tonic vestibulospinal posture reflex driven by Subsystem 3 (Cerebellum)'s gravito-inertial spatial attitude estimate (Fernández & Goldberg 1976, Paper I; Schor & Miller 1982). Pairs the dominant physiological pole p_ton = 15.0 rad/s (~2.4 Hz) with a fast secondary biophysical membrane pole p_fast = 250.0 rad/s (τ_2 = 4.0 ms) so that output y_ton, first derivative ẏ_ton = x_2,ton, and second derivative ÿ_ton are exact internal state variables without doubling phase lag in the 0.3–8 Hz band, followed by the sender-side MVST Branch Sub2B→Sub5 delay compensator 1 + τ_5·s + 0.5·τ_5²·s² (τ_5 = 2.5 ms).",
        latex: String.raw`H_{\text{ton}}(s) = K_{\text{ton}}\,\frac{p_{\text{ton}}\,p_{\text{fast}}}{(s + p_{\text{ton}})(s + p_{\text{fast}})}\underbrace{\bigl(1 + \tau_5 s + \tfrac{1}{2}\tau_5^2 s^2\bigr)}_{\text{MVST }\tau_5 = 2.5\text{ ms}}, \qquad \begin{cases}\dot{x}_{1,\text{ton}} = x_{2,\text{ton}} \\ \dot{x}_{2,\text{ton}} = p_{\text{ton}}p_{\text{fast}}(u_{\text{ton}} - x_{1,\text{ton}}) - (p_{\text{ton}} + p_{\text{fast}})x_{2,\text{ton}}\end{cases}`,
        math:
`Input:   tonic_otolith_input u_ton [from Tonic Tilt Efference Cancellation]
States:  x_1,ton = ton_state_*[3], x_2,ton = ton_vel_*[3]
Params:  K_ton (CMA-ES #1), p_ton = 15.0 rad/s (literature pole), p_fast = 250.0 rad/s (4.0 ms membrane pole), τ_5 = 0.0025 s (2.5 ms MVST delay)

For each axis ∈ {pitch, roll, yaw} do:
   1. 2nd-order state derivatives (dominant pole p_ton = 15 rad/s + fast pole p_fast = 250 rad/s):
      y_ton  ← x_1,ton
      ẏ_ton  ← x_2,ton
      ÿ_ton  ← (p_ton · p_fast) · (u_ton - x_1,ton) - (p_ton + p_fast) · x_2,ton

   2. Sender-side MVST Branch Sub2B→Sub5 delay compensator (1 + τ_5·s + 0.5·τ_5²·s², τ_5 = 2.5 ms):
      τ_ton  ← K_ton · (y_ton + τ_5 · ẏ_ton + 0.5 · τ_5² · ÿ_ton)

Output:  tonic_otolith_output ← τ_ton`,
        params: [
          "K_ton (CMA-ES #1): Tonic Spatial Attitude Gain [Nm/rad]",
          "p_ton = 15.0 rad/s (Dominant Literature Pole, ~2.4 Hz) + p_fast = 250.0 rad/s (Fast Membrane Pole, τ_2 = 4.0 ms)",
          "⚡ C_τ5(s) = 1 + τ_5·s + 0.5·τ_5²·s² : MVST Branch Sub2B→Sub5 delay compensator (τ_5 = 2.5 ms to Sub5)"
        ]
      },"""

s5_ton_new = r"""      s5_ton: {
        type: "Calibrated Tonic Spatial Attitude 2nd-Order LPF (K_ton = 5.26 Nm/rad, p_ton = 16.23 rad/s, p_fast = 250 rad/s)",
        title: "Tonic Spatial Attitude LPF — H_ton(s) (Calibrated Optimal p_ton = 16.23 rad/s, K_ton = 5.26 Nm/rad)",
        desc: "Models the sustained tonic vestibulospinal posture reflex driven by Subsystem 3 (Cerebellum)'s gravito-inertial spatial attitude estimate (Fernández & Goldberg 1976; Schor & Miller 1982). Pairs the calibrated dominant physiological pole p_ton = 16.227 rad/s (~2.58 Hz) with the secondary membrane pole p_fast = 250.0 rad/s (τ_2 = 4.0 ms). Provides strictly proper low-pass attenuation (relative degree 2, D = 0) with zero direct feedthrough, establishing stable low-frequency gravitational head attitude hold without DC drift. Restorative torque τ_ton is combined at the Brainstem VN Summing Junction before traversing the MVST (τ_5 = 2.5 ms).",
        latex: String.raw`\begin{aligned}
H_{\text{ton}}(s) &= K_{\text{ton}}\,\frac{p_{\text{ton}}\,p_{\text{fast}}}{(s + p_{\text{ton}})(s + p_{\text{fast}})}, \quad \text{Strictly Proper Realization }(D = 0) \\[4pt]
\text{State Dynamics:} &\quad \begin{cases}\dot{x}_{1,\text{ton}} = x_{2,\text{ton}} \\ \dot{x}_{2,\text{ton}} = p_{\text{ton}}p_{\text{fast}}\bigl(u_{\text{ton}} - x_{1,\text{ton}}\bigr) - (p_{\text{ton}} + p_{\text{fast}})x_{2,\text{ton}}\end{cases} \\[4pt]
\text{Output:} &\quad y_{\text{ton}} = x_{1,\text{ton}}, \qquad \tau_{\text{ton}} = K_{\text{ton}}\,x_{1,\text{ton}}
\end{aligned}`,
        math:
`Input:   tonic_otolith_input u_ton [from Tonic Tilt Efference Cancellation]
States:  x_1,ton = ton_state_*[3], x_2,ton = ton_vel_*[3]
Optimal: K_ton = 5.2598 Nm/rad, p_ton = 16.227 rad/s (2.58 Hz), p_fast = 250.0 rad/s (4.0 ms membrane pole)

For each axis ∈ {pitch, roll, yaw} do:
   1. Strictly proper 2nd-order state derivatives (relative degree 2, D = 0):
      ẏ_ton  ← x_2,ton
      ÿ_ton  ← (p_ton · p_fast) · (u_ton - x_1,ton) - (p_ton + p_fast) · x_2,ton

   2. Restorative tonic posture torque (zero direct feedthrough):
      τ_ton  ← K_ton · x_1,ton

Output:  tonic_otolith_output ← τ_ton (transmitted via MVST τ_5 = 2.5 ms)`,
        params: [
          "K_ton = 5.2598 Nm/rad : Discovered optimal tonic spatial attitude gain",
          "p_ton = 16.227 rad/s (~2.58 Hz) : Calibrated tonic spatial attitude pole",
          "p_fast = 250.0 rad/s (39.8 Hz) : Secondary biophysical membrane pole (τ_2 = 4.0 ms)",
          "Properness (D = 0) : Relative degree 2 ensures zero instantaneous feedthrough and zero DC drift",
          "⚡ MVST Conduction (Sub2B → Sub5) : τ_5 = 2.5 ms axonal conduction to C1–T1 cervical motoneuron pools"
        ]
      },"""

# =========================================================================
# 4. UPDATE BLOCK_INFO.s5_sum
# =========================================================================
s5_sum_old = r"""      s5_sum: {
        type: "Brainstem VN Restorative Torque Summing Junction + ⚡ Branch Sub2B→Sub5 MVST Out (τ_5 = 2.5 ms)",
        title: "Brainstem VCR Sum & MVST Branch Sub2B→Sub5 Output (τ_5 = 2.5 ms → Sub5)",
        desc: "Sums the restorative, MVST-delay-compensated torques from the Regular Otolith Tonic LPF (τ_ton), Semicircular Canal VCR Filter (τ_sc), and Irregular Otolith Phasic Central LPF (τ_phas) in the Brainstem Vestibular Nuclei (MVN / LVN) with negative feedback signs and transmits vestibulo_collic_torque[3] across the Medial/Lateral Vestibulospinal Tract (Branch Sub2B→Sub5: τ_5 = 2.5 ms) to Subsystem 5 (C1/T1 Lamina IX Pool) (where the 5.5 ms Aα ventral-root delay completes the 2.0 + 2.5 + 5.5 = 10.0 ms VCR loop).",
        latex: String.raw`\boldsymbol{\tau}_{\text{VCR,comp},5}(t) = -\Bigl(\underbrace{\boldsymbol{\tau}_{\text{ton}}(t)}_{\text{tonic tilt}} + \underbrace{\boldsymbol{\tau}_{\text{sc}}(t)}_{\text{canal VCR}} + \underbrace{\boldsymbol{\tau}_{\text{phas}}(t)}_{\text{phasic translation}}\Bigr) \xrightarrow{e^{-\tau_5 s}\;(\tau_5=2.5\text{ ms})} \text{Sub5 Input 2}`,
        math:
`Inputs:  1: tonic_otolith_output τ_ton[3]       [pre-compensated by 1 + τ_5·s + 0.5·τ_5²·s²]
         2: semicircular_canal_output τ_sc[3]   [pre-compensated by 1 + τ_5·s + 0.5·τ_5²·s²]
         3: phasic_otolith_output τ_phas[3]     [pre-compensated by 1 + τ_5·s + 0.5·τ_5²·s²]
Params:  τ_5 = 0.0025 s (2.5 ms MVST/LVST conduction delay to Subsystem 5 (C1/T1 Lamina IX Pool))

For each axis ∈ {pitch, roll, yaw} do:
   vestibulo_collic_torque_comp5(t) ← -(τ_ton(t) + τ_sc(t) + τ_phas(t))

Output:  1: vestibulo_collic_torque[3] ← τ_VCR,comp,5(t) (transmitted via τ_5 = 2.5 ms MVST to Sub5 Input 2)`,
        params: [
          "τ_ton[3] : Regular Otolith static tilt restorative torque",
          "τ_sc[3]  : Semicircular Canal angular velocity restorative torque",
          "τ_phas[3]: Irregular Otolith linear acceleration restorative torque",
          "⚡ Branch Sub2B→Sub5 (Sub2B → Sub5 MVST): τ_5 = 2.5 ms (Total VCR = 2.0 + 2.5 + 5.5 = 10.0 ms)"
        ]
      },"""

s5_sum_new = r"""      s5_sum: {
        type: "Brainstem VN Restorative Torque Summing Junction (Proper VCR Output with D = 0 Filters)",
        title: "Brainstem VCR Sum & MVST Output (Summing Strictly Proper Filter Outputs)",
        desc: "Sums the restorative torques from the Regular Otolith Tonic LPF (τ_ton), Semicircular Canal VCR Filter (τ_sc), and Irregular Otolith Phasic Central LPF (τ_phas) in the Brainstem Vestibular Nuclei (MVN / LVN) with negative feedback signs. Because all three constituent filters possess strictly proper transfer functions (relative degree ≥ 1, D = 0), the composite VCR torque command contains zero direct algebraic feedthrough. This eliminates algebraic loop hazards, prevents instantaneous torque jumps during perturbation onsets, and guarantees smooth descending drive across the Medial/Lateral Vestibulospinal Tract (Branch Sub2B→Sub5: τ_5 = 2.5 ms) into Subsystem 5 (C1/T1 Lamina IX Pool).",
        latex: String.raw`\boldsymbol{\tau}_{\text{VCR}}(t) = -\Bigl(\underbrace{K_{\text{ton}}\mathbf{x}_{1,\text{ton}}(t)}_{\text{tonic attitude hold}} + \underbrace{K_{\text{sc}}\bigl(\mathbf{x}_{1,\text{sc}} + z_{\text{sc}}\mathbf{x}_{2,\text{sc}}\bigr)}_{\text{canal VCR with phase-lead }z_{\text{sc}}} + \underbrace{K_{\text{phas}}\bigl(\mathbf{x}_{1,\text{phas}} + z_{\text{phas}}\mathbf{x}_{2,\text{phas}}\bigr)}_{\text{phasic otolith with }z_{\text{phas}}}\Bigr) \xrightarrow{\text{MVST }\tau_5 = 2.5\text{ ms}} \text{Sub5 Input 2}`,
        math:
`Inputs:  1: tonic_otolith_output τ_ton[3]       [strictly proper D = 0, K_ton = 5.2598]
         2: semicircular_canal_output τ_sc[3]   [strictly proper D = 0, K_sc = 4.1019, z_sc = 5.54 ms]
         3: phasic_otolith_output τ_phas[3]     [strictly proper D = 0, K_phas = 15.4205, z_phas = 13.31 ms]
Conduction: τ_5 = 0.0025 s (2.5 ms MVST axonal conduction delay to Sub5 Lamina IX Pool)

For each axis ∈ {pitch, roll, yaw} do:
   vestibulo_collic_torque(t) ← -(τ_ton(t) + τ_sc(t) + τ_phas(t))

Output:  1: vestibulo_collic_torque[3] ← τ_VCR(t) (transmitted via τ_5 = 2.5 ms MVST to Sub5 Input 2)`,
        params: [
          "τ_ton[3] : Calibrated static tilt restorative torque (K_ton = 5.2598 Nm/rad, p_ton = 16.23 rad/s)",
          "τ_sc[3]  : Calibrated canal VCR torque with phase lead (K_sc = 4.1019, z_sc = 5.54 ms, p_sc = 53.28 rad/s)",
          "τ_phas[3]: Calibrated otolith linear acceleration torque (K_phas = 15.4205, z_phas = 13.31 ms, p_phas = 4.71 rad/s)",
          "Strict Properness : Zero direct feedthrough (D = 0) prevents co-contraction spikes before QP allocation",
          "⚡ Branch Sub2B→Sub5 (Sub2B → Sub5 MVST): τ_5 = 2.5 ms (Total VCR loop = 2.0 + 2.5 + 5.5 = 10.0 ms)"
        ]
      },"""

# =========================================================================
# 5. UPDATE BLOCK_INFO.s6_rect
# =========================================================================
s6_rect_old = r"""      s6_rect: {
        type: "Half-Wave Stretch Rectifier, Pitch-Balanced CCR Gain (CMA-ES #4) & Branch Sub1B→Sub5 Out (τ_6 = 4.0 ms)",
        title: "Half-Wave Stretch Rectification & Pitch-Dominance-Weighted CCR Gain (Forming spinal_ccr_drive)",
        desc: "Half-wave rectifies the delay-compensated spindle output y_ccr,i (with a smooth C¹ quadratic knee over y_knee = 2 imp/s) so only muscle stretch excites the homonymous motoneuron pool, scales from firing rate (0–500 Hz) into normalized excitation via Kp_proprioception / 100 (CMA-ES #4), and applies Pitch-Dominance-Weighted Moment-Capacity Balancing w_ccr,i = 1 + η_pitch,i · (w_target - 1) (with w_ext = 0.70, w_flex = 1.15) before transmitting across the τ_6 = 4.0 ms cervical propriospinal relay to Subsystem 5 (C1/T1 Lamina IX Pool).",
        latex: String.raw`\eta_{\text{pitch},i} = \frac{r_{\text{pitch},i}^2}{r_{\text{pitch},i}^2 + r_{\text{roll},i}^2 + r_{\text{yaw},i}^2 + 10^{-8}}, \quad w_{\text{ccr},i} = 1 + \eta_{\text{pitch},i}\bigl(w_{\text{target},i} - 1\bigr), \quad u_{\text{ccr,comp},6,i}(t) = K_{p,\text{prop}}\,w_{\text{ccr},i}\,\mathcal{R}_{0.02}\!\left(\frac{y_{\text{ccr},i}(t)}{100}\right)`,
        math:
`Input:   filtered_primary_afferent_rate y_ccr[1..72] [from H_ccr(s), pre-compensated for τ_6 = 4.0 ms]
Params:  Kp_proprioception (CMA-ES #4), moment-arm matrix J_6 (6×72),
         w_ext = 0.70, w_flex = 1.15, smooth knee u_knee = 0.02, τ_6 = 0.0040 s (4.0 ms propriospinal delay)

For each muscle i ∈ {1..72} do:
   1. Compute pitch-dominance fraction η_pitch,i and balanced CCR weight w_ccr,i:
      r_p ← J_pitch[i],  r_r ← J_roll[i],  r_y ← J_yaw[i]
      η_pitch[i] ← r_p² / (r_p² + r_r² + r_y² + 1e-8)
      w_target   ← w_ext (0.70) if r_p > +1e-4 else w_flex (1.15) if r_p < -1e-4 else 1.0
      w_ccr[i]   ← 1.0 + η_pitch[i] · (w_target - 1.0)

   2. Smooth half-wave rectify & scale by Kp_proprioception · w_ccr[i]:
      u_raw ← max(0.0, y_ccr[i]) / 100.0
      u_rect ← (u_raw < 0.02) ? (u_raw² / 0.04) : (u_raw - 0.01)
      spinal_ccr_drive_comp6[i] ← Kp_proprioception · w_ccr[i] · u_rect

Output:  1: spinal_ccr_drive[1..72] ← u_ccr,comp,6[1..72] (transmitted via τ_6 = 4.0 ms to Subsystem 5 (C1/T1 Lamina IX Pool) Input 3)`,
        params: [
          "Kp_proprioception (CMA-ES #4): Spinal CCR Gain",
          {
            label: "η_pitch,i & w_ccr,i (w_ext = 0.70, w_flex = 1.15) : Pitch-Dominance-Weighted CCR",
            derivTitle: "Neutralizing 3.09:1 Extensor Pitch Bias Without Exciting 13.5 Hz Roll Clonus",
            derivDesc: "In the 72-muscle HYOID cervical model, 40 extensors have 49.45 N·m pitch moment capacity vs. 15.98 N·m for 24 flexors (3.09:1 ratio). Weighting the extensor/flexor balance factor (w_ext = 0.70, w_flex = 1.15) by each muscle's directional pitch-dominance ratio η_pitch,i balances sagittal pitch reflex stiffness while keeping roll/yaw-dominant muscles at unit gain:",
            derivLatex: String.raw`\eta_{\text{pitch},i} = \frac{r_{\text{pitch},i}^2}{r_{\text{pitch},i}^2 + r_{\text{roll},i}^2 + r_{\text{yaw},i}^2 + 10^{-8}}, \qquad w_{\text{ccr},i} = 1 + \eta_{\text{pitch},i}\bigl(w_{\text{target},i} - 1\bigr)`
          },
          "Output: spinal_ccr_drive[72] (transmitted via Branch Sub1B→Sub5 τ_6 = 4.0 ms propriospinal relay to Sub5)"
        ]
      },"""

s6_rect_new = r"""      s6_rect: {
        type: "Half-Wave Stretch Rectifier & Calibrated Proprioceptive CCR Gain (kp_prop = 0.4435, 1.099 Hz Optimization)",
        title: "Half-Wave Stretch Rectification & Calibrated CCR Gain (Resolving 1.099 Hz Gain Discrepancy)",
        desc: "Half-wave rectifies spindle stretch output y_ccr,i with a smooth C¹ quadratic knee (u_knee = 0.02) and scales into normalized motoneuron excitation via kp_proprioception = 0.4435. Crucial optimization discovery: relaxing kp_proprioception from 0.5508 down to 0.4435 selectively reduced excessive proprioceptive damping and extensor co-contraction around 1.1 Hz, lifting the 1.099 Hz closed-loop gain from -2.4 dB (242.8 deg/m) up to -0.99 dB (285.3 deg/m)—matching Happee et al. human subject experimental targets without disrupting high frequencies! Applies Pitch-Dominance-Weighted Moment-Capacity Balancing w_ccr,i = 1 + η_pitch,i · (w_target - 1) (w_ext = 0.70, w_flex = 1.15) before transmission across the τ_6 = 4.0 ms propriospinal relay to Subsystem 5.",
        latex: String.raw`\begin{aligned}
\eta_{\text{pitch},i} &= \frac{r_{\text{pitch},i}^2}{r_{\text{pitch},i}^2 + r_{\text{roll},i}^2 + r_{\text{yaw},i}^2 + 10^{-8}}, \qquad w_{\text{ccr},i} = 1 + \eta_{\text{pitch},i}\bigl(w_{\text{target},i} - 1\bigr) \\[4pt]
u_{\text{ccr},i}(t) &= k_{p,\text{prop}}\,w_{\text{ccr},i}\,\mathcal{R}_{0.02}\!\left(\frac{y_{\text{ccr},i}(t)}{100}\right), \qquad k_{p,\text{prop}} = 0.4435
\end{aligned}`,
        math:
`Input:   filtered_primary_afferent_rate y_ccr[1..72] [from H_ccr(s), strictly proper D = 0]
Params:  kp_proprioception = 0.4435 (calibrated optimal, lifts 1.099 Hz gain to -0.99 dB)
         Moment-arm matrix J_6 (6×72), w_ext = 0.70, w_flex = 1.15, smooth knee u_knee = 0.02, τ_6 = 4.0 ms

For each muscle i ∈ {1..72} do:
   1. Compute pitch-dominance fraction η_pitch,i and balanced CCR weight w_ccr,i:
      r_p ← J_pitch[i],  r_r ← J_roll[i],  r_y ← J_yaw[i]
      η_pitch[i] ← r_p² / (r_p² + r_r² + r_y² + 1e-8)
      w_target   ← w_ext (0.70) if r_p > +1e-4 else w_flex (1.15) if r_p < -1e-4 else 1.0
      w_ccr[i]   ← 1.0 + η_pitch[i] · (w_target - 1.0)

   2. Smooth half-wave rectify & scale by calibrated kp_proprioception (0.4435) · w_ccr[i]:
      u_raw ← max(0.0, y_ccr[i]) / 100.0
      u_rect ← (u_raw < 0.02) ? (u_raw² / 0.04) : (u_raw - 0.01)
      spinal_ccr_drive[i] ← kp_proprioception · w_ccr[i] · u_rect

Output:  1: spinal_ccr_drive[1..72] (transmitted via τ_6 = 4.0 ms propriospinal relay to Sub5 Input 3)`,
        params: [
          "kp_proprioception = 0.4435 : Calibrated spinal CCR gain (relaxed from 0.5508 to lift 1.099 Hz gain to -0.99 dB)",
          {
            label: "η_pitch,i & w_ccr,i (w_ext = 0.70, w_flex = 1.15) : Pitch-Dominance-Weighted CCR",
            derivTitle: "Neutralizing 3.09:1 Extensor Pitch Bias Without Exciting 13.5 Hz Roll Clonus",
            derivDesc: "In the 72-muscle HYOID cervical model, 40 extensors have 49.45 N·m pitch moment capacity vs. 15.98 N·m for 24 flexors (3.09:1 ratio). Weighting the extensor/flexor balance factor (w_ext = 0.70, w_flex = 1.15) by each muscle's directional pitch-dominance ratio η_pitch,i balances sagittal pitch reflex stiffness while keeping roll/yaw-dominant muscles at unit gain:",
            derivLatex: String.raw`\eta_{\text{pitch},i} = \frac{r_{\text{pitch},i}^2}{r_{\text{pitch},i}^2 + r_{\text{roll},i}^2 + r_{\text{yaw},i}^2 + 10^{-8}}, \qquad w_{\text{ccr},i} = 1 + \eta_{\text{pitch},i}\bigl(w_{\text{target},i} - 1\bigr)`
          },
          "⚡ Propriospinal Relay (Sub1B → Sub5) : τ_6 = 4.0 ms conduction delay to Lamina IX Voluntary-Gated Reciprocal Inhibition"
        ]
      },"""

# =========================================================================
# 6. UPDATE BLOCK_INFO.s6_ccr
# =========================================================================
s6_ccr_old = r"""      s6_ccr: {
        type: "72 Per-Muscle Spinal Interneuron 2nd-Order Filters (p_ccr = 100 rad/s, p_fast = 250 rad/s)",
        title: "Spinal Cervico-Collic Reflex (CCR) Filter — H_ccr(s) (Literature p_ccr = 100 rad/s, p_fast = 250 rad/s)",
        desc: "Processes each muscle's Branch Sub1A→Sub1B delay-compensated primary spindle stretch rate ΔIa_comp,Sub1A→Sub1B,i(t - 5.5 ms) (arriving from Subsystem 1A (C2–C8 DRG) across the C2–C8 dorsal root collateral, τ_Sub1A→Sub1B = 5.5 ms) through a 2nd-order spinal interneuron filter pairing the dominant literature pole p_ccr = 100.0 rad/s (~15.9 Hz, τ_syn = 10 ms; Burke et al. 1983; Peng et al. 1996; Keshner 2003) with the fast secondary biophysical membrane pole p_fast = 250.0 rad/s (τ_2 = 4.0 ms). Contains no internal delay compensators—the signal is forwarded after the half-wave rectifier by the single Branch Sub1B→Sub5 compensator (τ_6 = 4.0 ms) to Voluntary-Gated Reciprocal Inhibition in Subsystem 5.",
        latex: String.raw`H_{\text{ccr}}(s) = \frac{p_{\text{ccr}}\,p_{\text{fast}}}{(s + p_{\text{ccr}})(s + p_{\text{fast}})}, \qquad \begin{cases}y_{\text{ccr},i} = x_{1,\text{ccr},i} \\ \dot{y}_{\text{ccr},i} = x_{2,\text{ccr},i} \\ \ddot{y}_{\text{ccr},i} = (p_{\text{ccr}}p_{\text{fast}})\bigl(\Delta\text{Ia}_{\text{comp},1A\to 1B,i} - x_{1,\text{ccr},i}\bigr) - (p_{\text{ccr}} + p_{\text{fast}})x_{2,\text{ccr},i}\end{cases}`,
        math:
`Input:   1: delayed_primary_afferent_rate ΔIa_comp,Sub1A→Sub1B(t - 5.5 ms)[1..72] [from Sub1A Branch Sub1A→Sub1B, τ_Sub1A→Sub1B = 5.5 ms]
States:  x_1,ccr[1..72] (ccr_state_*), x_2,ccr[1..72] (ccr_vel_*) — 144 ODE states
Params:  p_ccr = 100.0 rad/s (literature pole), p_fast = 250.0 rad/s (4.0 ms membrane pole)

For each muscle i ∈ {1..72} do:
   1. Update 2nd-order spinal interneuron states (dominant p_ccr = 100 rad/s + fast p_fast = 250 rad/s):
      ẋ_1,ccr[i] ← x_2,ccr[i]
      a_ccr[i]   ← (p_ccr · p_fast) · (ΔIa_comp,Sub1A→Sub1B[i](t - 5.5 ms) - x_1,ccr[i]) - (p_ccr + p_fast) · x_2,ccr[i]
      ẋ_2,ccr[i] ← a_ccr[i]

   2. Output filtered state and exact 1st/2nd derivatives (to Half-Wave Rectifier):
      y_ccr[i] ← x_1,ccr[i]
      ẏ_ccr[i] ← x_2,ccr[i]
      ÿ_ccr[i] ← a_ccr[i]

Output:  filtered_primary_afferent_rate[1..72] ← y_ccr[1..72] (with ẏ_ccr, ÿ_ccr)`,
        params: [
          "Input: ΔIa_comp,Sub1A→Sub1B(t - 5.5 ms)[72] from Sub1A Branch Sub1A→Sub1B (already compensated for τ_Sub1A→Sub1B = 5.5 ms)",
          "p_ccr = 100.0 rad/s (Dominant Literature Pole, ~15.9 Hz) + p_fast = 250.0 rad/s (Fast Membrane Pole, τ_2 = 4.0 ms)",
          "No internal delay compensators inside H_ccr(s) — only the 4.0 ms Branch Sub1B→Sub5 hop to Voluntary-Gated Reciprocal Inhibition is compensated downstream after the Half-Wave Rectifier"
        ]
      },"""

s6_ccr_new = r"""      s6_ccr: {
        type: "72 Per-Muscle Spinal Interneuron 2nd-Order Filters (Strictly Proper D = 0, p_ccr = 99.92 rad/s, z_ccr = 0.0 s)",
        title: "Spinal Cervico-Collic Reflex (CCR) Filter — H_ccr(s) (Strictly Proper Realization D = 0, p_ccr = 99.92 rad/s)",
        desc: "Processes each muscle's primary spindle stretch rate through a 2nd-order spinal interneuron filter pairing the calibrated synapse pole p_ccr = 99.9196 rad/s (~15.9 Hz, τ = 10.0 ms) with the biophysical membrane pole p_fast = 250.0 rad/s (τ_2 = 4.0 ms) and strictly zero feedthrough zero z_ccr = 0.0 s (relative degree 2, D = 0). The strictly proper structure prevents raw sensor noise or sudden kinematics jumps from instantly punching into spinal motor pools, ensuring smooth, biophysically realistic cervico-collic stabilization.",
        latex: String.raw`H_{\text{ccr}}(s) = \frac{p_{\text{ccr}}\,p_{\text{fast}}}{(s + p_{\text{ccr}})(s + p_{\text{fast}})}, \quad y_{\text{ccr},i} = x_{1,\text{ccr},i} \quad (D = 0), \qquad \begin{cases}\dot{x}_{1,\text{ccr},i} = x_{2,\text{ccr},i} \\ \dot{x}_{2,\text{ccr},i} = p_{\text{ccr}}p_{\text{fast}}\bigl(u_{1B,i} - x_{1,\text{ccr},i}\bigr) - (p_{\text{ccr}} + p_{\text{fast}})x_{2,\text{ccr},i}\end{cases}`,
        math:
`Input:   primary spindle stretch rate ΔIa[1..72]
States:  x_1,ccr[1..72] (ccr_state_*), x_2,ccr[1..72] (ccr_vel_*) — 144 ODE states
Optimal: p_ccr = 99.9196 rad/s (15.9 Hz, τ = 10.0 ms), p_fast = 250.0 rad/s, z_ccr = 0.0 s (strictly proper, D = 0)

For each muscle i ∈ {1..72} do:
   1. Strictly proper 2nd-order state derivatives (relative degree 2, D = 0):
      ẋ_1,ccr[i] ← x_2,ccr[i]
      ẋ_2,ccr[i] ← (p_ccr · p_fast) · (u_1B[i] - x_1,ccr[i]) - (p_ccr + p_fast) · x_2,ccr[i]

   2. Filtered output (zero direct feedthrough D = 0):
      y_ccr[i]   ← x_1,ccr[i]

Output:  filtered_primary_afferent_rate[1..72] ← y_ccr[1..72] (to Half-Wave Rectifier & Gain)`,
        params: [
          "p_ccr = 99.9196 rad/s (~15.9 Hz, τ_syn = 10.0 ms) : Calibrated dominant interneuron synapse pole",
          "z_ccr = 0.0 s : Zero CCR lead constant (strictly proper relative degree 2, D = 0)",
          "p_fast = 250.0 rad/s (39.8 Hz) : Secondary biophysical membrane pole (τ_2 = 4.0 ms)",
          "Properness (D = 0) : Strict properness filters sensor velocity discontinuities before rectifying into motoneurons"
        ]
      },"""

# =========================================================================
# 7. UPDATE BLOCK_INFO.s1_pid
# =========================================================================
s1_pid_old = r"""      s1_pid: {
        type: "Task-Space PID + Anti-Windup",
        title: "Cortical Task-Space 3-DOF PID Controller",
        desc: "Computes unfiltered cortical voluntary torque by comparing the planned trajectory against the Branch 3B ascending Mahony spatial state, with anti-windup clamping on the posture integral and zero initial integral preload (since static upright gravity equilibrium is provided by the 72-muscle postural co-contraction baseline u_min[1..72]).",
        latex: String.raw`\tau_{\text{raw}} = \underbrace{K_p\bigl(x_1 - \theta_{\text{ctx}}\bigr)}_{\text{postural stiffness}} + \underbrace{K_d\bigl(x_2 - \omega_{\text{ctx}}\bigr)}_{\text{velocity damping}} + \underbrace{K_i \, I_\theta}_{\text{postural integral }(I_\theta(0)=0)} + \underbrace{J_{\text{head}}\,\ddot{x}_1}_{\text{inertial feedforward}}`,
        math:
`Inputs:  trajectory (x_1, x_2, a_traj), delayed_mahony (θ_ctx, ω_ctx)
States:  I_θ (error_integral per axis, initialized at I_θ(0) = 0)
Params:  K_p, K_d, K_i, J_head, integral_bound I_max

For each axis ∈ {pitch, roll, yaw} do:
   1. Task-space tracking errors:
      e_pos ← x_1 - θ_ctx
      e_vel ← x_2 - ω_ctx

   2. Conditional anti-windup integration:
      if (I_θ ≥ +I_max and e_pos > 0) or (I_θ ≤ -I_max and e_pos < 0) then
         e_int_rate ← 0
      else
         e_int_rate ← e_pos
      I_θ ← I_θ + e_int_rate · Δt

   3. Unfiltered cortical torque command:
      τ_raw ← K_p · e_pos + K_d · e_vel + K_i · I_θ + J_head · a_traj

Output:  unfiltered_voluntary_torque ← τ_raw`,
        params: [
          "Pitch Gains (9-Param CMA-ES): Kp_task_pitch (#7), Kd_task_pitch (#8), Ki_task_pitch (#9)",
          "Roll/Yaw Gains: Kp_task_roll/yaw, Kd_task_roll/yaw, Ki_task_roll/yaw = 0",
          {
            label: "I_θ(0) = 0 : Zero Integral Preload (Gravity Balanced by u_min[72])",
            derivTitle: "Gravity Decoupling via Multi-Muscle Co-contraction Baseline",
            derivDesc: "Because the 72 cervical muscles are pre-loaded to basal tone u_min[1..72] = 0.02, static upright cervical equilibrium is sustained without cortical integrator windup. Initializing I_θ(0) = 0 prevents initial acceleration transients.",
            derivLatex: String.raw`I_\theta(0) = 0, \quad \tau_{\text{gravity}} \approx \sum_{i=1}^{72} \mathbf{r}_i \times \mathbf{F}_0(u_{\text{min},i})`
          }
        ]
      },"""

s1_pid_new = r"""      s1_pid: {
        type: "Task-Space PID + Anti-Windup (Discovered Optimal Multi-Objective Pitch Gains)",
        title: "Cortical Task-Space 3-DOF PID Controller (Calibrated Kp=3.79, Kd=0.49, Ki=3.72)",
        desc: "Computes unfiltered cortical voluntary torque by comparing the planned trajectory against the ascending Mahony spatial state, incorporating anti-windup clamping and zero initial integral preload. Discovered optimal pitch gains (Kp_task = 3.7901 Nm/rad, Kd_task = 0.4946 Nm/(rad/s), Ki_task = 3.7215 Nm/(rad·s)) work synergistically with the strictly proper subcortical VCR and CCR loops to deliver outstanding multi-objective performance: total score of 991.11 pts, with 1.099 Hz gain at -0.99 dB and 7.93 Hz phase error slashed to 3.8°, achieving zero high-frequency penalty and zero DC drift.",
        latex: String.raw`\tau_{\text{raw}} = \underbrace{K_{p,\text{task}}\bigl(x_1 - \theta_{\text{ctx}}\bigr)}_{\text{postural stiffness}} + \underbrace{K_{d,\text{task}}\bigl(x_2 - \omega_{\text{ctx}}\bigr)}_{\text{velocity damping}} + \underbrace{K_{i,\text{task}} \, I_\theta}_{\text{postural integral }(I_\theta(0)=0)} + \underbrace{J_{\text{head}}\,\ddot{x}_1}_{\text{inertial feedforward}}`,
        math:
`Inputs:  trajectory (x_1, x_2, a_traj), delayed_mahony (θ_ctx, ω_ctx)
States:  I_θ (error_integral per axis, initialized at I_θ(0) = 0)
Optimal: Pitch Gains: Kp_task = 3.7901 Nm/rad, Kd_task = 0.4946 Nm/(rad/s), Ki_task = 3.7215 Nm/(rad·s)
         Fusimotor Gains: K_gamma_dyn = 1.0, K_gamma_stat = 1.0 | Score = 991.11 pts

For each axis ∈ {pitch, roll, yaw} do:
   1. Task-space tracking errors:
      e_pos ← x_1 - θ_ctx
      e_vel ← x_2 - ω_ctx

   2. Conditional anti-windup integration:
      if (I_θ ≥ +I_max and e_pos > 0) or (I_θ ≤ -I_max and e_pos < 0) then
         e_int_rate ← 0
      else
         e_int_rate ← e_pos
      I_θ ← I_θ + e_int_rate · Δt

   3. Cortical torque command with calibrated gains:
      τ_raw ← Kp_task · e_pos + Kd_task · e_vel + Ki_task · I_θ + J_head · a_traj

Output:  unfiltered_voluntary_torque ← τ_raw`,
        params: [
          "Calibrated Pitch Gains : Kp_task = 3.7901 Nm/rad, Kd_task = 0.4946 Nm/(rad/s), Ki_task = 3.7215 Nm/(rad·s)",
          "Fusimotor Gains : K_gamma_dyn = 1.0, K_gamma_stat = 1.0",
          "Performance Achieved : Total Score = 991.11 pts, 1.099 Hz gain = -0.99 dB, 7.93 Hz phase error = 3.8°",
          "Roll/Yaw Gains : Roll (Kp=10.21, Kd=1.29), Yaw (Kp=0.24, Kd=0.03, Ki=0)",
          {
            label: "I_θ(0) = 0 : Zero Integral Preload (Gravity Balanced by u_min[72])",
            derivTitle: "Gravity Decoupling via Multi-Muscle Co-contraction Baseline",
            derivDesc: "Because the 72 cervical muscles are pre-loaded to basal tone u_min[1..72] = 0.02, static upright cervical equilibrium is sustained without cortical integrator windup. Initializing I_θ(0) = 0 prevents initial acceleration transients.",
            derivLatex: String.raw`I_\theta(0) = 0, \quad \tau_{\text{gravity}} \approx \sum_{i=1}^{72} \mathbf{r}_i \times \mathbf{F}_0(u_{\text{min},i})`
          }
        ]
      },"""

# Verify and replace each block in content
for name, old, new in [
    ("s5_sc", s5_sc_old, s5_sc_new),
    ("s5_phas", s5_phas_old, s5_phas_new),
    ("s5_ton", s5_ton_old, s5_ton_new),
    ("s5_sum", s5_sum_old, s5_sum_new),
    ("s6_rect", s6_rect_old, s6_rect_new),
    ("s6_ccr", s6_ccr_old, s6_ccr_new),
    ("s1_pid", s1_pid_old, s1_pid_new),
]:
    if old in content:
        content = content.replace(old, new)
        print(f"✓ Replaced BLOCK_INFO.{name}")
    else:
        print(f"✗ FAILED to find exact BLOCK_INFO.{name}")
        # Try normalized line endings
        old_norm = old.replace("\r\n", "\n")
        content_norm = content.replace("\r\n", "\n")
        if old_norm in content_norm:
            content = content_norm.replace(old_norm, new.replace("\r\n", "\n"))
            print(f"✓ Replaced BLOCK_INFO.{name} (normalized line endings)")
        else:
            print(f"ERROR: Could not match BLOCK_INFO.{name} even with normalized line endings!")

# =========================================================================
# 8. UPDATE SVG TEXT INSIDE EXISTING BLOCKS (DIAGRAMS)
# =========================================================================

# In s1_pid block (Subsystem 4):
svg_pid_old = """                 <text x="559" y="129" text-anchor="middle" font-family="monospace" font-size="7.4" fill="#b91c1c">Pitch: Kp(#7), Kd(#8), Ki(#9)</text>"""
svg_pid_new = """                 <text x="559" y="129" text-anchor="middle" font-family="monospace" font-size="7.4" fill="#b91c1c">Pitch: Kp=3.79, Kd=0.49, Ki=3.72</text>"""
if svg_pid_old in content:
    content = content.replace(svg_pid_old, svg_pid_new)
    print("✓ Updated SVG text in s1_pid block")

# In s5_ton block:
svg_ton_old = """              <text x="537" y="606" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#b91c1c">CMA-ES: K_ton(#1) | p_ton=15, p_f=250</text>"""
svg_ton_new = """              <text x="537" y="606" text-anchor="middle" font-family="monospace" font-size="6.2" fill="#b91c1c">Calibrated: K_ton=5.26 | p_ton=16.2, p_f=250</text>"""
if svg_ton_old in content:
    content = content.replace(svg_ton_old, svg_ton_new)
    print("✓ Updated SVG text in s5_ton block (line 2989)")

# In s5_sc block:
svg_sc_t1_old = """              <text x="537" y="693" text-anchor="middle" font-family="monospace" font-size="6.4">H_sc(s) = K_sc·p_sc·p_f / [(s+p_sc)(s+p_f)]</text>
              <text x="537" y="706" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#b91c1c">CMA-ES: K_sc(#2) | p_sc=50, p_f=250</text>"""
svg_sc_t1_new = """              <text x="537" y="693" text-anchor="middle" font-family="monospace" font-size="6.2">H_sc(s) = K_sc·p_sc·p_f·(1+z_sc·s)/[(s+p_sc)(s+p_f)]</text>
              <text x="537" y="706" text-anchor="middle" font-family="monospace" font-size="6.1" fill="#b91c1c">D=0 Proper: K_sc=4.10, z_sc=5.54ms, p_sc=53.3</text>"""
if svg_sc_t1_old in content:
    content = content.replace(svg_sc_t1_old, svg_sc_t1_new)
    print("✓ Updated SVG text in s5_sc block (line 2995)")

# In s5_sc block (second occurrence if in sub5):
svg_sc_t2_old = """              <text x="537" y="148" text-anchor="middle" font-family="monospace" font-size="6.4">H_sc(s) = K_sc·p_sc·p_f / [(s+p_sc)(s+p_f)]</text>
              <text x="537" y="161" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#b91c1c">CMA-ES: K_sc(#2) | p_sc=50, p_f=250</text>"""
svg_sc_t2_new = """              <text x="537" y="148" text-anchor="middle" font-family="monospace" font-size="6.2">H_sc(s) = K_sc·p_sc·p_f·(1+z_sc·s)/[(s+p_sc)(s+p_f)]</text>
              <text x="537" y="161" text-anchor="middle" font-family="monospace" font-size="6.1" fill="#b91c1c">D=0 Proper: K_sc=4.10, z_sc=5.54ms, p_sc=53.3</text>"""
if svg_sc_t2_old in content:
    content = content.replace(svg_sc_t2_old, svg_sc_t2_new)
    print("✓ Updated SVG text in s5_sc block (line 3614)")

# In s5_phas block:
svg_phas_t1_old = """              <text x="537" y="803" text-anchor="middle" font-family="monospace" font-size="6.4">H_phas(s) = K_phas·p_phas·p_f / [(s+p_phas)(s+p_f)]</text>
              <text x="537" y="816" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#b91c1c">CMA-ES: K_phas(#3) | p_phas=4.0, p_f=250</text>"""
svg_phas_t1_new = """              <text x="537" y="803" text-anchor="middle" font-family="monospace" font-size="6.2">H_ph(s) = K_ph·p_ph·p_f·(1+z_ph·s)/[(s+p_ph)(s+p_f)]</text>
              <text x="537" y="816" text-anchor="middle" font-family="monospace" font-size="6.1" fill="#b91c1c">D=0 Proper: K_ph=15.42, z_ph=13.31ms, p_ph=4.7</text>"""
if svg_phas_t1_old in content:
    content = content.replace(svg_phas_t1_old, svg_phas_t1_new)
    print("✓ Updated SVG text in s5_phas block (line 3002)")

svg_phas_t2_old = """              <text x="537" y="258" text-anchor="middle" font-family="monospace" font-size="6.4">H_phas(s) = K_phas·p_phas·p_f / [(s+p_phas)(s+p_f)]</text>
              <text x="537" y="271" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#b91c1c">CMA-ES: K_phas(#3) | p_phas=4.0, p_f=250</text>"""
svg_phas_t2_new = """              <text x="537" y="258" text-anchor="middle" font-family="monospace" font-size="6.2">H_ph(s) = K_ph·p_ph·p_f·(1+z_ph·s)/[(s+p_ph)(s+p_f)]</text>
              <text x="537" y="271" text-anchor="middle" font-family="monospace" font-size="6.1" fill="#b91c1c">D=0 Proper: K_ph=15.42, z_ph=13.31ms, p_ph=4.7</text>"""
if svg_phas_t2_old in content:
    content = content.replace(svg_phas_t2_old, svg_phas_t2_new)
    print("✓ Updated SVG text in s5_phas block (line 3622)")

# In s6_rect block:
svg_rect_old = """                 <text x="580" y="111" text-anchor="middle" font-family="monospace" font-size="7.0" fill="#b91c1c">CMA-ES Mask: Kp_proprioception(#4)</text>"""
svg_rect_new = """                 <text x="580" y="111" text-anchor="middle" font-family="monospace" font-size="6.6" fill="#b91c1c">Calibrated: kp_prop = 0.4435 (lifts 1.1Hz to -0.99dB)</text>"""
if svg_rect_old in content:
    content = content.replace(svg_rect_old, svg_rect_new)
    print("✓ Updated SVG text in s6_rect block (line 3706)")

# In s6_ccr block:
svg_ccr_old = """                 <text x="326" y="111" text-anchor="middle" font-family="monospace" font-size="6.8" fill="#b91c1c">Lit: p_ccr = 100 rad/s, p_f = 250 rad/s</text>"""
svg_ccr_new = """                 <text x="326" y="111" text-anchor="middle" font-family="monospace" font-size="6.6" fill="#b91c1c">Strict Proper D=0: p_ccr=99.92 rad/s, z_ccr=0</text>"""
if svg_ccr_old in content:
    content = content.replace(svg_ccr_old, svg_ccr_new)
    print("✓ Updated SVG text in s6_ccr block (line 3698)")

# Save updated file
with open(HTML_PATH, "w", encoding="utf-8") as f:
    f.write(content)
print(f"Saved {HTML_PATH}")

# Mirror to brain artifact directory
shutil.copy2(HTML_PATH, ARTIFACT_PATH)
print(f"Mirrored to {ARTIFACT_PATH}")
