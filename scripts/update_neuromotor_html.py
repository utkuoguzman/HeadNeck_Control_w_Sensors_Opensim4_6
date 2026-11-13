import os
import re

html_path = r"d:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\docs\neuromotor_block_explorer.html"
art_path  = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\neuromotor_block_explorer.html"

with open(html_path, "r", encoding="utf-8") as f:
    text = f.read()

print("Original size:", len(text))

# 1. Update navigation bar to include the new Tab 7 button
nav_search = '<button onclick="selectTab(\'atlas\')" id="tab-atlas" class="px-2 py-0.5 text-[11px] font-bold rounded border border-sky-600 bg-sky-50 text-sky-800 hover:bg-sky-100 shadow-sm">⚡ 6. Anatomical Delay &amp; Prospective Coding Atlas</button>'
nav_replace = nav_search + '\n            <button onclick="selectTab(\'optimal\')" id="tab-optimal" class="px-2 py-0.5 text-[11px] font-bold rounded border border-emerald-600 bg-emerald-50 text-emerald-800 hover:bg-emerald-100 shadow-sm">🎯 7. Calibrated Optimal Parameters &amp; Strictly Proper TF Architecture (991 pts)</button>'

if nav_search in text:
    text = text.replace(nav_search, nav_replace)
    print("[OK] Added tab-optimal button to navigation bar")
else:
    print("[WARN] nav_search not found!")

# 2. Update selectTab active array
tab_array_search = "['top', 'sub1', 'sub2', 'sub3', 'sub4', 'sub5', 'sub6', 'sub7', 'atlas'].forEach(k => {"
tab_array_replace = "['top', 'sub1', 'sub2', 'sub3', 'sub4', 'sub5', 'sub6', 'sub7', 'atlas', 'optimal'].forEach(k => {"
if tab_array_search in text:
    text = text.replace(tab_array_search, tab_array_replace)
    print("[OK] Updated selectTab tab list")
else:
    print("[WARN] tab_array_search not found!")

# 3. Update BLOCK_INFO entries: s5_ton, s5_sc, s5_phas, s6_rect, s6_ccr, s1_pid
# s5_sc replacement
old_s5_sc = """      s5_sc: {
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

new_s5_sc = """      s5_sc: {
        type: "Strictly-Proper Canal VCR Filter (p_sc = 53.28 rad/s, z_sc = 5.54 ms, p_fast = 250 rad/s) + ⚡ Branch Sub2B→Sub5 MVST Compensator (τ_5 = 2.5 ms)",
        title: "Semicircular Canal Central VCR Filter — H_sc(s) (Strictly-Proper Zero z_sc = 5.54 ms, p_sc = 53.28 rad/s, p_fast = 250 rad/s)",
        desc: "Models the semicircular canal velocity-driven VCR reflex in the medial vestibular nucleus (MVN). Incorporates a strictly-proper 1st-order state-space transfer function zero (1 + z_sc·s) realized via internal state coordinates (x_sc_eff = x_1,sc + z_sc·x_2,sc) with zero direct feedthrough (D = 0). This strictly proper formulation eliminates instantaneous algebraic feedthrough loops, completely prevents DC co-contraction transients during active torque allocation, and provides the vital high-frequency phase lead (cutting 7.93 Hz phase error to 3.8°). Pairs the dominant physiological canal pole p_sc = 53.28 rad/s (~8.48 Hz) with the fast secondary biophysical membrane pole p_fast = 250.0 rad/s (τ_2 = 4.0 ms) and optimal gain K_sc = 4.1019 Nm/(rad/s), followed by the sender-side MVST Branch Sub2B→Sub5 axonal delay compensator (τ_5 = 2.5 ms).",
        latex: String.raw`H_{\text{sc}}(s) = K_{\text{sc}}\,\frac{p_{\text{sc}}\,p_{\text{fast}}\,(1 + z_{\text{sc}}\,s)}{(s + p_{\text{sc}})(s + p_{\text{fast}})}\underbrace{\bigl(1 + \tau_5 s + \tfrac{1}{2}\tau_5^2 s^2\bigr)}_{\text{MVST }\tau_5 = 2.5\text{ ms}}, \qquad \begin{cases}\dot{x}_{1,\text{sc}} = x_{2,\text{sc}} \\ \dot{x}_{2,\text{sc}} = p_{\text{sc}}p_{\text{fast}}(u_{\text{sc}} - x_{1,\text{sc}}) - (p_{\text{sc}} + p_{\text{fast}})x_{2,\text{sc}} \\ x_{\text{sc,eff}} = x_{1,\text{sc}} + z_{\text{sc}}\,x_{2,\text{sc}} \quad (D=0\text{ strictly proper})\end{cases}`,
        math:
`Input:   semicircular_canal_input u_sc [from Canal Velocity Efference Cancellation]
States:  x_1,sc = sc_state_*[3], x_2,sc = sc_vel_*[3]
Discovered Optimal Params:
         K_sc   = 4.1019 Nm/(rad/s) [Calibrated Canal Gain, Run 4]
         z_sc   = 0.005544 s (5.54 ms strictly-proper vestibular zero)
         p_sc   = 53.284 rad/s (dominant canal pole, ~8.48 Hz)
         p_fast = 250.0 rad/s (4.0 ms fast membrane pole)
         τ_5    = 0.0025 s (2.5 ms MVST axonal delay)

For each axis ∈ {pitch, roll, yaw} do:
   1. 2nd-order state derivatives (strictly proper state-space realization):
      a_sc   ← (p_sc · p_fast) · (u_sc - x_1,sc) - (p_sc + p_fast) · x_2,sc
      ẋ_1,sc ← x_2,sc
      ẋ_2,sc ← a_sc

   2. Strictly-proper 1st-order vestibular TF zero (zero direct feedthrough, D = 0):
      z_sc_lead ← (axis == pitch) ? z_sc : 0.0
      x_sc_eff  ← x_1,sc + z_sc_lead · x_2,sc
      ẋ_sc_eff  ← x_2,sc + z_sc_lead · a_sc

   3. Outgoing MVST Branch Sub2B→Sub5 axonal delay compensator C_τ5(s) = 1 + τ_5·s + 0.5·τ_5²·s² (τ_5 = 2.5 ms):
      τ_sc  ← K_sc · (x_sc_eff + τ_5 · ẋ_sc_eff + 0.5 · τ_5² · a_sc)

Output:  semicircular_canal_output ← τ_sc`,
        params: [
          "K_sc = 4.1019 Nm/(rad/s) (Discovered Optimal Canal VCR Gain, Run 4)",
          "z_sc = 0.005544 s (5.54 ms Strictly-Proper Zero Time Constant, phase lead @ 7.93 Hz)",
          "p_sc = 53.284 rad/s (~8.48 Hz dominant pole) + p_fast = 250.0 rad/s (Fast Membrane Pole, τ_2 = 4.0 ms)",
          "Strictly Proper (D = 0): Realized via internal state coordinates x_1 + z_sc·x_2, preventing algebraic loops and DC co-contraction transients",
          "⚡ C_τ5(s) = 1 + τ_5·s + 0.5·τ_5²·s² : MVST Branch Sub2B→Sub5 delay compensator (τ_5 = 2.5 ms to Sub5)"
        ]
      },"""

if old_s5_sc in text:
    text = text.replace(old_s5_sc, new_s5_sc)
    print("[OK] Replaced s5_sc in BLOCK_INFO")
else:
    print("[WARN] old_s5_sc not found!")

# s5_phas replacement
old_s5_phas = """      s5_phas: {
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

new_s5_phas = """      s5_phas: {
        type: "Strictly-Proper Phasic Otolith VCR Filter (p_phas = 4.71 rad/s, z_phas = 13.31 ms, p_fast = 250 rad/s) + ⚡ MVST Compensator (τ_5 = 2.5 ms)",
        title: "Irregular Otolith Phasic Central Filter — H_phas(s) (Strictly-Proper Zero z_phas = 13.31 ms, p_phas = 4.71 rad/s)",
        desc: "Integrates linear acceleration toward translational velocity using the otolith velocity-storage leaky-integrator pole p_phas = 4.71 rad/s (~0.75 Hz; Angelaki 2004) paired with a strictly-proper state-space TF zero z_phas = 2.40·z_sc = 0.013307 s (13.31 ms). This zero differentiates feedforward translational acceleration to recover high-frequency phase lead, while matched canal damping preserves 7.93 Hz gain at +0.07 dB. Realized strictly-properly via internal state coordinates (x_phas_eff = x_1,phas + z_phas·x_2,phas, D = 0) with membrane pole p_fast = 250.0 rad/s and optimal gain K_phas = 15.4205 Nm/(m/s²), followed by the MVST Branch Sub2B→Sub5 delay compensator (τ_5 = 2.5 ms).",
        latex: String.raw`H_{\text{phas}}(s) = K_{\text{phas}}\,\frac{p_{\text{phas}}\,p_{\text{fast}}\,(1 + z_{\text{phas}}\,s)}{(s + p_{\text{phas}})(s + p_{\text{fast}})}\underbrace{\bigl(1 + \tau_5 s + \tfrac{1}{2}\tau_5^2 s^2\bigr)}_{\text{MVST }\tau_5 = 2.5\text{ ms}}, \qquad \begin{cases}\dot{x}_{1,\text{phas}} = x_{2,\text{phas}} \\ \dot{x}_{2,\text{phas}} = p_{\text{phas}}p_{\text{fast}}(u_{\text{phas}} - x_{1,\text{phas}}) - (p_{\text{phas}} + p_{\text{fast}})x_{2,\text{phas}} \\ x_{\text{phas,eff}} = x_{1,\text{phas}} + z_{\text{phas}}\,x_{2,\text{phas}} \quad (D=0)\end{cases}`,
        math:
`Input:   phasic_otolith_input u_phas [from Phasic Gravity & Efference Cancellation]
States:  x_1,phas = phas_state_*[3], x_2,phas = phas_vel_*[3]
Discovered Optimal Params:
         K_phas = 15.4205 Nm/(m/s²) [Calibrated Phasic Gain, Run 4]
         z_phas = 0.013307 s (13.31 ms = 2.40 · z_sc strictly-proper zero)
         p_phas = 4.7119 rad/s (dominant leaky-integrator pole, ~0.75 Hz)
         p_fast = 250.0 rad/s (4.0 ms membrane pole)
         τ_5    = 0.0025 s (2.5 ms MVST delay)

For each axis ∈ {pitch, roll, yaw} do:
   1. 2nd-order state derivatives (strictly proper state-space realization):
      a_phas   ← (p_phas · p_fast) · (u_phas - x_1,phas) - (p_phas + p_fast) · x_2,phas
      ẋ_1,phas ← x_2,phas
      ẋ_2,phas ← a_phas

   2. Strictly-proper 1st-order phasic vestibular TF zero (zero direct feedthrough, D = 0):
      z_phas_lead ← (axis == pitch) ? (2.40 · z_sc) : 0.0
      x_phas_eff  ← x_1,phas + z_phas_lead · x_2,phas
      ẋ_phas_eff  ← x_2,phas + z_phas_lead · a_phas

   3. Sender-side MVST Branch Sub2B→Sub5 delay compensator (1 + τ_5·s + 0.5·τ_5²·s², τ_5 = 2.5 ms):
      τ_phas  ← K_phas · (x_phas_eff + τ_5 · ẋ_phas_eff + 0.5 · τ_5² · a_phas)

Output:  phasic_otolith_output ← τ_phas`,
        params: [
          "K_phas = 15.4205 Nm/(m/s²) (Discovered Optimal Phasic Otolith Gain, Run 4)",
          "z_phas = 13.31 ms (2.40 · z_sc strictly-proper zero, feedforward acceleration differentiation)",
          "p_phas = 4.7119 rad/s (~0.75 Hz leaky-integrator pole) + p_fast = 250.0 rad/s",
          "Strictly Proper (D = 0): Solves high-frequency phase lag without feedthrough spikes into QP allocation",
          "⚡ C_τ5(s) = 1 + τ_5·s + 0.5·τ_5²·s² : MVST Branch Sub2B→Sub5 delay compensator (τ_5 = 2.5 ms to Sub5)"
        ]
      },"""

if old_s5_phas in text:
    text = text.replace(old_s5_phas, new_s5_phas)
    print("[OK] Replaced s5_phas in BLOCK_INFO")
else:
    print("[WARN] old_s5_phas not found!")

# s5_sum replacement
old_s5_sum = """    For each axis ∈ {pitch, roll, yaw} do:
   vestibulo_collic_torque_comp5(t) ← -(τ_ton(t) + τ_sc(t) + τ_phas(t))"""

new_s5_sum = """    For each axis ∈ {pitch, roll, yaw} do:
   vestibulo_collic_torque_comp5(t) ← -(K_ton · x_ton + K_sc · x_sc_eff + K_phas · x_phas_eff) [all pre-compensated for τ_5 = 2.5 ms]"""

if old_s5_sum in text:
    text = text.replace(old_s5_sum, new_s5_sum)
    print("[OK] Updated s5_sum equation in BLOCK_INFO")
else:
    print("[WARN] old_s5_sum not found!")

# 4. Update SVG text in DIAGRAMS.sub2
old_svg_sc = """              <text x="537" y="680" text-anchor="middle" font-family="sans-serif" font-size="7.9" font-weight="bold" fill="#111827">Canal Velocity VCR Filter H_sc(s)</text>
              <text x="537" y="693" text-anchor="middle" font-family="monospace" font-size="6.4">H_sc(s) = K_sc·p_sc·p_f / [(s+p_sc)(s+p_f)]</text>
              <text x="537" y="706" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#b91c1c">CMA-ES: K_sc(#2) | p_sc=50, p_f=250</text>"""

new_svg_sc = """              <text x="537" y="680" text-anchor="middle" font-family="sans-serif" font-size="7.9" font-weight="bold" fill="#111827">Canal VCR Filter H_sc(s) (Strictly Proper)</text>
              <text x="537" y="693" text-anchor="middle" font-family="monospace" font-size="6.3">H_sc(s) = K_sc·p_sc·p_f·(1+z_sc·s) / [(s+p_sc)(s+p_f)]</text>
              <text x="537" y="706" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#047857">Opt: K_sc=4.10, z_sc=5.54ms | p_sc=53.3, p_f=250</text>"""

if old_svg_sc in text:
    text = text.replace(old_svg_sc, new_svg_sc)
    print("[OK] Updated s5_sc in DIAGRAMS.sub2 SVG")
else:
    print("[WARN] old_svg_sc not found!")

old_svg_phas = """              <text x="537" y="790" text-anchor="middle" font-family="sans-serif" font-size="7.9" font-weight="bold" fill="#111827">Otolith Phasic Velocity-Storage LPF</text>
              <text x="537" y="803" text-anchor="middle" font-family="monospace" font-size="6.4">H_phas(s) = K_phas·p_phas·p_f / [(s+p_phas)(s+p_f)]</text>
              <text x="537" y="816" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#b91c1c">CMA-ES: K_phas(#3) | p_phas=4.0, p_f=250</text>"""

new_svg_phas = """              <text x="537" y="790" text-anchor="middle" font-family="sans-serif" font-size="7.9" font-weight="bold" fill="#111827">Otolith Phasic VCR Filter H_phas(s) (Strictly Proper)</text>
              <text x="537" y="803" text-anchor="middle" font-family="monospace" font-size="6.3">H_phas(s) = K_phas·p_ph·p_f·(1+2.4z_sc·s) / [(s+p_ph)(s+p_f)]</text>
              <text x="537" y="816" text-anchor="middle" font-family="monospace" font-size="6.3" fill="#047857">Opt: K_phas=15.42, z_phas=13.31ms | p_ph=4.71</text>"""

if old_svg_phas in text:
    text = text.replace(old_svg_phas, new_svg_phas)
    print("[OK] Updated s5_phas in DIAGRAMS.sub2 SVG")
else:
    print("[WARN] old_svg_phas not found!")

# 5. Insert DIAGRAMS.optimal dashboard into DIAGRAMS
optimal_html = """
      optimal: {
        breadcrumb: "Top_Level_System / 🎯 7. Calibrated Optimal Parameters & Strictly Proper TF Architecture (991 pts)",
        svg: `
          <div class="p-4 bg-white text-slate-900 font-sans space-y-5 max-w-6xl mx-auto">
            <!-- Header Banner -->
            <div class="bg-gradient-to-r from-emerald-900 via-slate-900 to-sky-950 text-white rounded-xl p-5 shadow-md border border-emerald-700/40">
              <div class="flex flex-wrap items-center justify-between gap-3">
                <div>
                  <div class="flex items-center gap-2 mb-1">
                    <span class="px-2 py-0.5 rounded bg-emerald-500 text-slate-950 font-mono text-[10px] font-black uppercase tracking-wider">Sub-1,000 pt Milestone</span>
                    <span class="px-2 py-0.5 rounded bg-sky-500/20 text-sky-200 border border-sky-400/40 font-mono text-[10px] font-bold">Happee (2017) B8EC Benchmark</span>
                    <span class="px-2 py-0.5 rounded bg-purple-500/20 text-purple-200 border border-purple-400/40 font-mono text-[10px] font-bold">Strictly Proper State-Space (D = 0)</span>
                  </div>
                  <h1 class="text-xl md:text-2xl font-black tracking-tight text-white flex items-center gap-2">
                    🎯 Neuromotor Controller Calibration &amp; Strictly Proper TF Architecture
                  </h1>
                  <p class="text-xs text-slate-300 mt-1 max-w-3xl leading-relaxed">
                    Integration of strictly-proper 1st-order state-space transfer function zeros (<span class="font-mono text-emerald-300">1 + τ_z·s</span>) on Subsystem 2(B) VCR filters, spinal CCR velocity damping relaxation (<span class="font-mono text-emerald-300">k_p,prop = 0.4435</span>), and multi-objective real-plant adaptive calibration matching the complete 21.52s (6-cycle) experimental profile.
                  </p>
                </div>
                <div class="bg-slate-800/80 border border-slate-700 rounded-lg p-3 text-right shrink-0">
                  <div class="text-[10px] text-slate-400 font-mono uppercase tracking-wider font-bold">21.52s Pure Stationary Score</div>
                  <div class="text-2xl font-black text-emerald-400 font-mono">991.11 pts</div>
                  <div class="text-[10px] text-slate-300 font-mono mt-0.5">hf_penalty: 0.0 &bull; DC drift: 0.0</div>
                </div>
              </div>
            </div>

            <!-- Key Metric Cards Grid -->
            <div class="grid grid-cols-2 md:grid-cols-4 gap-3 text-xs">
              <div class="bg-emerald-50 border border-emerald-200 rounded-lg p-3">
                <span class="text-[10px] font-bold uppercase tracking-wider text-emerald-800 block">1.099 Hz Gain Discrepancy</span>
                <span class="text-lg font-black text-emerald-700 font-mono block mt-0.5">285.26 deg/m (-0.99 dB)</span>
                <span class="text-[11px] text-slate-600 block mt-1">Target: 319.74 deg/m &bull; Surpassed -1.2 dB goal! Recovered from -2.44 dB (Run 1) via Ia damping relaxation.</span>
              </div>
              <div class="bg-purple-50 border border-purple-200 rounded-lg p-3">
                <span class="text-[10px] font-bold uppercase tracking-wider text-purple-800 block">High-Freq Phase Lead</span>
                <span class="text-lg font-black text-purple-700 font-mono block mt-0.5">7.93 Hz: 3.8° | 5.58 Hz: 16.9°</span>
                <span class="text-[11px] text-slate-600 block mt-1">Slashed from 28.4° &amp; 28.7° via strictly-proper vestibular zero (z_sc = 5.54 ms, z_phas = 13.31 ms).</span>
              </div>
              <div class="bg-blue-50 border border-blue-200 rounded-lg p-3">
                <span class="text-[10px] font-bold uppercase tracking-wider text-blue-800 block">Mid &amp; High Gain Precision</span>
                <span class="text-lg font-black text-blue-700 font-mono block mt-0.5">0.78 Hz: -0.06 dB | 7.93 Hz: +0.07 dB</span>
                <span class="text-[11px] text-slate-600 block mt-1">0.783 Hz phase error 0.9° &bull; Matched canal inertia prevents high-frequency gain bulging.</span>
              </div>
              <div class="bg-amber-50 border border-amber-200 rounded-lg p-3">
                <span class="text-[10px] font-bold uppercase tracking-wider text-amber-800 block">6-DOF Cross-Axis Stability</span>
                <span class="text-lg font-black text-amber-700 font-mono block mt-0.5">Max |Roll|: 0.80° | |Yaw|: 0.98°</span>
                <span class="text-[11px] text-slate-600 block mt-1">Pitch velocity RMS: 34.1°/s vs 31.2°/s ref (r = +0.942) &bull; Zero high-frequency tremor penalty.</span>
              </div>
            </div>

            <!-- Mathematical Architecture: The Properness Breakthrough -->
            <div class="bg-slate-50 border border-slate-200 rounded-xl p-4 space-y-3">
              <div class="flex items-center justify-between border-b border-slate-200 pb-2">
                <div class="flex items-center gap-2">
                  <span class="px-2 py-0.5 rounded bg-purple-700 text-white font-mono text-[10px] font-bold uppercase">Mathematical Core</span>
                  <h2 class="text-sm font-bold text-slate-800">1. The Strictly-Proper State-Space Zero Architecture (D = 0)</h2>
                </div>
                <span class="text-[11px] text-slate-500 font-mono">HeadNeckNeuralController.cpp (lines 2792–2812)</span>
              </div>
              <div class="grid grid-cols-1 md:grid-cols-2 gap-4 text-xs text-slate-700">
                <div class="space-y-2">
                  <p class="leading-relaxed">
                    <strong>Why Standard Post-QP Differentiation Failed:</strong> Inserting a derivative or zero block after the Active-Set Quadratic Program (QP) or as direct feedthrough ($D \\neq 0$) created high-frequency algebraic spikes ($s \\cdot u(t)$) during muscle allocation. In OpenSim's continuous integrator, these feedthrough spikes caused severe DC co-contraction transients, numerical instability, and lateral roll/yaw jerking.
                  </p>
                  <p class="leading-relaxed">
                    <strong>The Strictly-Proper Solution ($D = 0$):</strong> By realizing the 1st-order transfer function zero $(1 + \\tau_z s)$ pre-QP inside the 2nd-order VCR state-space filters via linear combination of internal state variables ($x_1$ and $x_2 = \\dot{x}_1$), the transfer function from afferent acceleration/velocity to reflex output has <strong>degree of denominator (2) strictly greater than numerator (1)</strong>:
                  </p>
                  <div class="bg-white border border-slate-300 rounded p-2.5 font-mono text-[11px] text-slate-900 overflow-x-auto">
                    H_sc(s)   = K_sc · [p_sc · p_f · (1 + z_sc · s)] / [(s + p_sc)(s + p_f)]<br/>
                    H_phas(s) = K_phas · [p_ph · p_f · (1 + z_phas · s)] / [(s + p_ph)(s + p_f)]
                  </div>
                </div>
                <div class="space-y-2">
                  <p class="leading-relaxed">
                    <strong>Exact State-Space Implementation:</strong> The state derivatives remain strictly proper 2nd-order ODEs driven solely by delayed sensory afferents $u(t)$, completely free of algebraic loops:
                  </p>
                  <div class="bg-white border border-slate-300 rounded p-2.5 font-mono text-[10.5px] text-slate-900 overflow-x-auto leading-relaxed">
                    // 1. Classical 2nd-order filter state derivatives (p_fast = 250 rad/s):<br/>
                    a_sc   = (p_sc * p_fast) * (u_sc - x_sc) - (p_sc + p_fast) * v_sc;<br/>
                    a_phas = (p_phas * p_fast) * (u_phas - x_phas) - (p_phas + p_fast) * v_phas;<br/>
                    <br/>
                    // 2. Strictly-proper state-space TF zeros (D = 0, pitch axis):<br/>
                    x_sc_eff   = x_sc   + z_sc_lead   * v_sc;         // z_sc = 5.54 ms<br/>
                    x_phas_eff = x_phas + z_phas_lead * v_phas;       // z_phas = 13.31 ms<br/>
                    <br/>
                    // 3. Restorative VCR Torque (Clean, zero direct feedthrough):<br/>
                    tau_VCR = -(K_ton * x_ton + K_sc * x_sc_eff + K_phas * x_phas_eff);
                  </div>
                  <p class="text-[11px] text-slate-500 leading-relaxed">
                    <strong>Biophysical Interpretation:</strong> The phasic otolith zero ($z_{\\text{phas}} = 13.31\\text{ ms}$) acts as prospective predictive lead from T1 sled linear acceleration, compensating for the 10 ms physical VCR conduction delay, while the matched canal zero ($z_{\\text{sc}} = 5.54\\text{ ms}$) balances virtual head inertia to hold 7.93 Hz gain flat ($+0.07\\text{ dB}$).
                  </p>
                </div>
              </div>
            </div>

            <!-- Discovered Optimal 15-Parameter Set Table -->
            <div class="bg-white border border-slate-200 rounded-xl p-4 space-y-3">
              <div class="flex items-center justify-between border-b border-slate-200 pb-2">
                <div class="flex items-center gap-2">
                  <span class="px-2 py-0.5 rounded bg-emerald-700 text-white font-mono text-[10px] font-bold uppercase">Optimal Parameters</span>
                  <h2 class="text-sm font-bold text-slate-800">2. Discovered 15-Parameter Calibration Set (Sub-1,000 pt Performance)</h2>
                </div>
                <span class="text-[11px] text-emerald-700 font-bold font-mono">Calibrated in Run 4 &bull; Verified on 21.52s Pure Simulation</span>
              </div>
              <div class="overflow-x-auto">
                <table class="w-full text-left font-mono text-[11px] border-collapse">
                  <thead>
                    <tr class="bg-slate-100 text-slate-700 border-b border-slate-300">
                      <th class="py-1.5 px-2">#</th>
                      <th class="py-1.5 px-2">Parameter</th>
                      <th class="py-1.5 px-2">Variable Name</th>
                      <th class="py-1.5 px-2 font-bold text-emerald-800">Discovered Optimal Value</th>
                      <th class="py-1.5 px-2">Literature / Baseline</th>
                      <th class="py-1.5 px-2">Units</th>
                      <th class="py-1.5 px-2">Subsystem &amp; Role</th>
                    </tr>
                  </thead>
                  <tbody class="divide-y divide-slate-200 text-slate-800">
                    <tr class="hover:bg-slate-50">
                      <td class="py-1 px-2 font-bold text-slate-500">0</td>
                      <td class="py-1 px-2 font-bold text-emerald-900">K_ton</td>
                      <td class="py-1 px-2 text-slate-600">adapt_K_ton</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">5.259843</td>
                      <td class="py-1 px-2 text-slate-500">5.262089</td>
                      <td class="py-1 px-2 text-slate-500">N&middot;m/rad</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Sub2B Regular Otolith static tilt attitude stiffness</td>
                    </tr>
                    <tr class="hover:bg-slate-50 bg-slate-50/50">
                      <td class="py-1 px-2 font-bold text-slate-500">1</td>
                      <td class="py-1 px-2">p_ton</td>
                      <td class="py-1 px-2 text-slate-600">adapt_p_ton</td>
                      <td class="py-1 px-2 font-mono">16.227225</td>
                      <td class="py-1 px-2 text-slate-500">16.227225</td>
                      <td class="py-1 px-2 text-slate-500">rad/s</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Dominant tonic posture filter pole (~2.58 Hz; fixed)</td>
                    </tr>
                    <tr class="hover:bg-slate-50">
                      <td class="py-1 px-2 font-bold text-slate-500">2</td>
                      <td class="py-1 px-2 font-bold text-emerald-900">K_sc</td>
                      <td class="py-1 px-2 text-slate-600">adapt_K_sc</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">4.101869</td>
                      <td class="py-1 px-2 text-slate-500">4.335589</td>
                      <td class="py-1 px-2 text-slate-500">N&middot;m/(rad/s)</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Sub2B Semicircular Canal angular velocity VCR gain</td>
                    </tr>
                    <tr class="hover:bg-purple-50/60 bg-purple-50/30">
                      <td class="py-1 px-2 font-bold text-purple-700">3</td>
                      <td class="py-1 px-2 font-bold text-purple-900">z_sc</td>
                      <td class="py-1 px-2 text-purple-700">adapt_z_sc</td>
                      <td class="py-1 px-2 font-bold text-purple-800">0.005544 (5.54 ms)</td>
                      <td class="py-1 px-2 text-slate-500">0.000000</td>
                      <td class="py-1 px-2 text-slate-500">s</td>
                      <td class="py-1 px-2 font-sans text-[11px] font-bold text-purple-900">⚡ Strictly-proper vestibular TF zero; recovers high-freq phase lead</td>
                    </tr>
                    <tr class="hover:bg-slate-50 bg-slate-50/50">
                      <td class="py-1 px-2 font-bold text-slate-500">4</td>
                      <td class="py-1 px-2">p_sc</td>
                      <td class="py-1 px-2 text-slate-600">adapt_p_sc</td>
                      <td class="py-1 px-2 font-mono">53.284141</td>
                      <td class="py-1 px-2 text-slate-500">53.284141</td>
                      <td class="py-1 px-2 text-slate-500">rad/s</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Dominant semicircular canal corner pole (~8.48 Hz; fixed)</td>
                    </tr>
                    <tr class="hover:bg-slate-50">
                      <td class="py-1 px-2 font-bold text-slate-500">5</td>
                      <td class="py-1 px-2 font-bold text-emerald-900">K_phas</td>
                      <td class="py-1 px-2 text-slate-600">adapt_K_phas</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">15.420487</td>
                      <td class="py-1 px-2 text-slate-500">14.324995</td>
                      <td class="py-1 px-2 text-slate-500">N&middot;m/(m/s²)</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Sub2B Irregular Otolith translational acceleration reflex gain</td>
                    </tr>
                    <tr class="hover:bg-slate-50 bg-slate-50/50">
                      <td class="py-1 px-2 font-bold text-slate-500">6</td>
                      <td class="py-1 px-2">p_phas</td>
                      <td class="py-1 px-2 text-slate-600">adapt_p_phas</td>
                      <td class="py-1 px-2 font-mono">4.711937</td>
                      <td class="py-1 px-2 text-slate-500">4.711937</td>
                      <td class="py-1 px-2 text-slate-500">rad/s</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Otolith velocity-storage leaky integrator pole (~0.75 Hz; fixed)</td>
                    </tr>
                    <tr class="hover:bg-amber-50/60 bg-amber-50/30">
                      <td class="py-1 px-2 font-bold text-amber-700">7</td>
                      <td class="py-1 px-2 font-bold text-amber-900">kp_prop</td>
                      <td class="py-1 px-2 text-amber-700">adapt_kp_prop</td>
                      <td class="py-1 px-2 font-bold text-amber-800">0.443511</td>
                      <td class="py-1 px-2 text-slate-500">0.550781</td>
                      <td class="py-1 px-2 text-slate-500">&mdash;</td>
                      <td class="py-1 px-2 font-sans text-[11px] font-bold text-amber-900">⚡ Relaxed Ia CCR damping; unlocked 1.099 Hz gain to -0.99 dB</td>
                    </tr>
                    <tr class="hover:bg-slate-50 bg-slate-50/50">
                      <td class="py-1 px-2 font-bold text-slate-500">8</td>
                      <td class="py-1 px-2">z_ccr</td>
                      <td class="py-1 px-2 text-slate-600">adapt_z_ccr</td>
                      <td class="py-1 px-2 font-mono">0.000000</td>
                      <td class="py-1 px-2 text-slate-500">0.000000</td>
                      <td class="py-1 px-2 text-slate-500">s</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Spinal CCR 1st-order zero time constant (fixed at 0.0)</td>
                    </tr>
                    <tr class="hover:bg-slate-50">
                      <td class="py-1 px-2 font-bold text-slate-500">9</td>
                      <td class="py-1 px-2">p_ccr</td>
                      <td class="py-1 px-2 text-slate-600">adapt_p_ccr</td>
                      <td class="py-1 px-2 font-mono">99.919586</td>
                      <td class="py-1 px-2 text-slate-500">99.919586</td>
                      <td class="py-1 px-2 text-slate-500">rad/s</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Spinal interneuron CCR corner pole (~15.9 Hz; fixed)</td>
                    </tr>
                    <tr class="hover:bg-slate-50 bg-slate-50/50">
                      <td class="py-1 px-2 font-bold text-slate-500">10</td>
                      <td class="py-1 px-2">K_gamma_dyn</td>
                      <td class="py-1 px-2 text-slate-600">adapt_K_gamma_dyn</td>
                      <td class="py-1 px-2 font-mono">1.000000</td>
                      <td class="py-1 px-2 text-slate-500">1.000000</td>
                      <td class="py-1 px-2 text-slate-500">&mdash;</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Dynamic fusimotor Bag1 voluntary sensitivity (biological cap 1.0)</td>
                    </tr>
                    <tr class="hover:bg-slate-50">
                      <td class="py-1 px-2 font-bold text-slate-500">11</td>
                      <td class="py-1 px-2">K_gamma_stat</td>
                      <td class="py-1 px-2 text-slate-600">adapt_K_gamma_stat</td>
                      <td class="py-1 px-2 font-mono">1.000000</td>
                      <td class="py-1 px-2 text-slate-500">1.000000</td>
                      <td class="py-1 px-2 text-slate-500">&mdash;</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Static fusimotor Bag2/Chain voluntary sensitivity (biological cap 1.0)</td>
                    </tr>
                    <tr class="hover:bg-slate-50 bg-slate-50/50">
                      <td class="py-1 px-2 font-bold text-slate-500">12</td>
                      <td class="py-1 px-2 font-bold text-emerald-900">Kp_task_pitch</td>
                      <td class="py-1 px-2 text-slate-600">adapt_Kp_task_pitch</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">3.790117</td>
                      <td class="py-1 px-2 text-slate-500">3.835668</td>
                      <td class="py-1 px-2 text-slate-500">N&middot;m/rad</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Cortical voluntary task-space proportional pitch stiffness</td>
                    </tr>
                    <tr class="hover:bg-slate-50">
                      <td class="py-1 px-2 font-bold text-slate-500">13</td>
                      <td class="py-1 px-2 font-bold text-emerald-900">Kd_task_pitch</td>
                      <td class="py-1 px-2 text-slate-600">adapt_Kd_task_pitch</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">0.494580</td>
                      <td class="py-1 px-2 text-slate-500">0.540494</td>
                      <td class="py-1 px-2 text-slate-500">N&middot;m/(rad/s)</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Cortical voluntary task-space derivative pitch damping</td>
                    </tr>
                    <tr class="hover:bg-slate-50 bg-slate-50/50">
                      <td class="py-1 px-2 font-bold text-slate-500">14</td>
                      <td class="py-1 px-2 font-bold text-emerald-900">Ki_task_pitch</td>
                      <td class="py-1 px-2 text-slate-600">adapt_Ki_task_pitch</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">3.721492</td>
                      <td class="py-1 px-2 text-slate-500">3.790829</td>
                      <td class="py-1 px-2 text-slate-500">N&middot;m/(rad&middot;s)</td>
                      <td class="py-1 px-2 font-sans text-[11px]">Cortical voluntary integral gain (I_max = 0.12 rad&middot;s)</td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </div>

            <!-- Happee 2017 B8EC 6-Frequency Validation Table -->
            <div class="bg-slate-50 border border-slate-200 rounded-xl p-4 space-y-3">
              <div class="flex items-center justify-between border-b border-slate-200 pb-2">
                <div class="flex items-center gap-2">
                  <span class="px-2 py-0.5 rounded bg-sky-700 text-white font-mono text-[10px] font-bold uppercase">Empirical Benchmark</span>
                  <h2 class="text-sm font-bold text-slate-800">3. Full 21.52s Pure Stationary 6-Frequency Bode Response (Happee 2017 B8EC)</h2>
                </div>
                <span class="text-[11px] text-slate-500 font-mono">6 Full Fundamental Cycles &bull; MULTISINE_CYCLES = 6.0</span>
              </div>
              <div class="overflow-x-auto">
                <table class="w-full text-left font-mono text-[11px] border-collapse">
                  <thead>
                    <tr class="bg-slate-200/70 text-slate-700 border-b border-slate-300">
                      <th class="py-1.5 px-2">Frequency</th>
                      <th class="py-1.5 px-2">Target Gain</th>
                      <th class="py-1.5 px-2 font-bold text-emerald-800">Sim Gain</th>
                      <th class="py-1.5 px-2">Gain Discrepancy</th>
                      <th class="py-1.5 px-2">Target Phase</th>
                      <th class="py-1.5 px-2 font-bold text-purple-800">Sim Phase</th>
                      <th class="py-1.5 px-2">Phase Error</th>
                      <th class="py-1.5 px-2 text-right">Frequency Score</th>
                    </tr>
                  </thead>
                  <tbody class="divide-y divide-slate-200 text-slate-800">
                    <tr class="hover:bg-slate-100/50">
                      <td class="py-1 px-2 font-bold">0.324 Hz</td>
                      <td class="py-1 px-2 text-slate-600">86.55 deg/m</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">102.82 deg/m</td>
                      <td class="py-1 px-2 text-amber-700">+1.50 dB</td>
                      <td class="py-1 px-2 text-slate-600">-7.03°</td>
                      <td class="py-1 px-2 font-bold text-purple-700">-16.81°</td>
                      <td class="py-1 px-2">9.8°</td>
                      <td class="py-1 px-2 text-right">319.4 pts</td>
                    </tr>
                    <tr class="hover:bg-slate-100/50 bg-emerald-50/40">
                      <td class="py-1 px-2 font-bold">0.783 Hz</td>
                      <td class="py-1 px-2 text-slate-600">236.93 deg/m</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">235.30 deg/m</td>
                      <td class="py-1 px-2 font-bold text-emerald-800">-0.06 dB</td>
                      <td class="py-1 px-2 text-slate-600">-96.96°</td>
                      <td class="py-1 px-2 font-bold text-purple-700">-96.08°</td>
                      <td class="py-1 px-2 font-bold text-emerald-800">0.9°</td>
                      <td class="py-1 px-2 text-right font-bold text-emerald-800">1.1 pts</td>
                    </tr>
                    <tr class="hover:bg-slate-100/50 bg-emerald-50/60 font-bold">
                      <td class="py-1 px-2">1.099 Hz</td>
                      <td class="py-1 px-2 text-slate-600">319.74 deg/m</td>
                      <td class="py-1 px-2 text-emerald-800">285.26 deg/m</td>
                      <td class="py-1 px-2 text-emerald-800">-0.99 dB (Goal Met!)</td>
                      <td class="py-1 px-2 text-slate-600">-109.23°</td>
                      <td class="py-1 px-2 text-purple-800">-117.23°</td>
                      <td class="py-1 px-2">8.0°</td>
                      <td class="py-1 px-2 text-right text-emerald-800">162.3 pts</td>
                    </tr>
                    <tr class="hover:bg-slate-100/50">
                      <td class="py-1 px-2 font-bold">2.446 Hz</td>
                      <td class="py-1 px-2 text-slate-600">423.34 deg/m</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">483.30 deg/m</td>
                      <td class="py-1 px-2 text-amber-700">+1.15 dB</td>
                      <td class="py-1 px-2 text-slate-600">-161.34°</td>
                      <td class="py-1 px-2 font-bold text-purple-700">-160.35°</td>
                      <td class="py-1 px-2 font-bold text-emerald-800">1.0°</td>
                      <td class="py-1 px-2 text-right">133.4 pts</td>
                    </tr>
                    <tr class="hover:bg-slate-100/50">
                      <td class="py-1 px-2 font-bold">5.583 Hz</td>
                      <td class="py-1 px-2 text-slate-600">503.30 deg/m</td>
                      <td class="py-1 px-2 font-bold text-emerald-700">556.07 deg/m</td>
                      <td class="py-1 px-2 text-slate-700">+0.87 dB</td>
                      <td class="py-1 px-2 text-slate-600">-201.38°</td>
                      <td class="py-1 px-2 font-bold text-purple-700">-218.27° (+141.73°)</td>
                      <td class="py-1 px-2 font-bold text-purple-700">16.9° (down from 28.7°)</td>
                      <td class="py-1 px-2 text-right">360.3 pts</td>
                    </tr>
                    <tr class="hover:bg-slate-100/50 bg-purple-50/60 font-bold">
                      <td class="py-1 px-2">7.927 Hz</td>
                      <td class="py-1 px-2 text-slate-600">572.69 deg/m</td>
                      <td class="py-1 px-2 text-emerald-800">577.16 deg/m</td>
                      <td class="py-1 px-2 text-emerald-800">+0.07 dB</td>
                      <td class="py-1 px-2 text-slate-600">-247.62°</td>
                      <td class="py-1 px-2 text-purple-800">-251.38° (+108.62°)</td>
                      <td class="py-1 px-2 text-purple-800">3.8° (down from 28.4°!)</td>
                      <td class="py-1 px-2 text-right text-emerald-800">14.6 pts</td>
                    </tr>
                  </tbody>
                  <tfoot>
                    <tr class="bg-slate-200/80 font-bold text-slate-900 border-t-2 border-slate-300">
                      <td colspan="7" class="py-2 px-2 text-right">Total Multisine Score (6 Frequencies, hf_penalty = 0.0):</td>
                      <td class="py-2 px-2 text-right text-emerald-800 font-mono text-xs">991.11 pts</td>
                    </tr>
                  </tfoot>
                </table>
              </div>
            </div>

            <!-- Iterative Adaptation Progression -->
            <div class="bg-white border border-slate-200 rounded-xl p-4 space-y-3">
              <div class="flex items-center justify-between border-b border-slate-200 pb-2">
                <div class="flex items-center gap-2">
                  <span class="px-2 py-0.5 rounded bg-slate-700 text-white font-mono text-[10px] font-bold uppercase">Progression</span>
                  <h2 class="text-sm font-bold text-slate-800">4. Multi-Stage Adaptation Progression (21.52s Pure Stationary Simulations)</h2>
                </div>
                <span class="text-[11px] text-slate-500 font-mono">Comparing Seed Champion &rarr; Runs 1 to 4</span>
              </div>
              <div class="overflow-x-auto">
                <table class="w-full text-left font-mono text-[11px] border-collapse">
                  <thead>
                    <tr class="bg-slate-100 text-slate-700 border-b border-slate-300">
                      <th class="py-1.5 px-2">Evaluation Stage</th>
                      <th class="py-1.5 px-2">Total Score</th>
                      <th class="py-1.5 px-2">1.099 Hz Gain</th>
                      <th class="py-1.5 px-2">5.58 Hz Phase Err</th>
                      <th class="py-1.5 px-2">7.93 Hz Phase Err</th>
                      <th class="py-1.5 px-2">Key Mechanism Added</th>
                    </tr>
                  </thead>
                  <tbody class="divide-y divide-slate-200 text-slate-800">
                    <tr class="hover:bg-slate-50">
                      <td class="py-1.5 px-2 font-bold">Pre-Adaptive Champion (W04_S4_G02)</td>
                      <td class="py-1.5 px-2 font-mono">1,638.71 pts</td>
                      <td class="py-1.5 px-2">273.21 deg/m (-1.37 dB)</td>
                      <td class="py-1.5 px-2">23.5°</td>
                      <td class="py-1.5 px-2">24.4°</td>
                      <td class="py-1.5 px-2 font-sans text-[11px]">Static seed parameters (9.17s baseline)</td>
                    </tr>
                    <tr class="hover:bg-slate-50">
                      <td class="py-1.5 px-2 font-bold">Adaptation Run 1 (Pure 21.5s)</td>
                      <td class="py-1.5 px-2 font-mono">3,234.15 pts</td>
                      <td class="py-1.5 px-2 text-red-600">241.41 deg/m (-2.44 dB)</td>
                      <td class="py-1.5 px-2">27.9°</td>
                      <td class="py-1.5 px-2">24.6°</td>
                      <td class="py-1.5 px-2 font-sans text-[11px]">Over-damped Ia feedback; 1.099 Hz depressed</td>
                    </tr>
                    <tr class="hover:bg-slate-50">
                      <td class="py-1.5 px-2 font-bold">Adaptation Run 2 (Pure 21.5s)</td>
                      <td class="py-1.5 px-2 font-mono">3,790.61 pts</td>
                      <td class="py-1.5 px-2">262.98 deg/m (-1.70 dB)</td>
                      <td class="py-1.5 px-2">28.5°</td>
                      <td class="py-1.5 px-2">25.9°</td>
                      <td class="py-1.5 px-2 font-sans text-[11px]">Began damping relaxation; 1.099 Hz gain recovered +0.74 dB</td>
                    </tr>
                    <tr class="hover:bg-slate-50">
                      <td class="py-1.5 px-2 font-bold">Adaptation Run 3 (Pure 21.5s)</td>
                      <td class="py-1.5 px-2 font-mono">2,713.21 pts</td>
                      <td class="py-1.5 px-2">272.01 deg/m (-1.40 dB)</td>
                      <td class="py-1.5 px-2 text-amber-700">28.7°</td>
                      <td class="py-1.5 px-2 text-amber-700">28.4°</td>
                      <td class="py-1.5 px-2 font-sans text-[11px]">1.099 Hz recovered to -1.40 dB; revealed high-freq phase lag gap</td>
                    </tr>
                    <tr class="hover:bg-emerald-50 bg-emerald-50/70 font-bold text-emerald-950">
                      <td class="py-1.5 px-2">★ Adaptation Run 4 (Pure 21.5s)</td>
                      <td class="py-1.5 px-2 font-mono text-emerald-700">991.11 pts</td>
                      <td class="py-1.5 px-2 text-emerald-700">285.26 deg/m (-0.99 dB)</td>
                      <td class="py-1.5 px-2 text-purple-700">16.9°</td>
                      <td class="py-1.5 px-2 text-purple-700">3.8°</td>
                      <td class="py-1.5 px-2 font-sans text-[11px] text-emerald-800">Strictly-proper zero (z_sc=5.54ms) + relaxed kp_prop (0.4435)</td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </div>
          </div>
        `
      },
"""

diag_marker = "const DIAGRAMS = {"
if diag_marker in text:
    text = text.replace(diag_marker, diag_marker + optimal_html)
    print("[OK] Inserted DIAGRAMS.optimal dashboard")
else:
    print("[WARN] diag_marker not found!")

with open(html_path, "w", encoding="utf-8") as f:
    f.write(text)
print("[OK] Successfully updated", html_path)

with open(art_path, "w", encoding="utf-8") as f:
    f.write(text)
print("[OK] Successfully mirrored to brain artifact", art_path)
