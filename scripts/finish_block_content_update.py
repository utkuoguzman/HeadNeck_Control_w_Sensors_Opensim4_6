import re
import shutil

HTML_PATH = "docs/neuromotor_block_explorer.html"
ARTIFACT_PATH = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\neuromotor_block_explorer.html"

with open(HTML_PATH, "r", encoding="utf-8") as f:
    text = f.read()

# 1. Update BLOCK_INFO.s1_pid
s1_pid_pattern = r's1_pid:\s*\{.*?derivLatex:[^\n]*?\n\s*\}\,\n\s*"J_head : head_inertia_pitch/roll/yaw"\n\s*\]\n\s*\},'

s1_pid_replacement = r"""s1_pid: {
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
            derivTitle: "Zero Initial Cortical Integrator State via 6-DOF Muscle Co-Contraction Equilibrium",
            derivDesc: "Because the 72 individual muscle co-contraction baselines u_min[1..72] are solved to balance all 6 cervical joint accelerations at t = 0 (∑ A_{:,i} u_min,i = τ_grav), the required corrective task torque at upright rest (e_pos = 0, e_vel = 0) is zero:",
            derivLatex: String.raw`\sum_{i=1}^{72} A_{:,i}\,u_{\min,i} = \boldsymbol{\tau}_{\text{grav}} \implies \boldsymbol{\tau}_{\text{raw}}(0) = \mathbf{0}, \qquad I_\theta(0) = 0`
          },
          "J_head : head_inertia_pitch/roll/yaw"
        ]
      },"""

new_text, count = re.subn(s1_pid_pattern, lambda m: s1_pid_replacement, text, count=1, flags=re.DOTALL)
if count == 1:
    print("✓ Successfully replaced BLOCK_INFO.s1_pid via regex")
    text = new_text
else:
    print(f"✗ Failed regex match for BLOCK_INFO.s1_pid (count={count})")

# 2. SVG Text updates:
# s1_pid SVG text:
old_svg_pid = 'Pitch: Kp(#7), Kd(#8), Ki(#9)'
new_svg_pid = 'Pitch: Kp=3.79, Kd=0.49, Ki=3.72'
if old_svg_pid in text:
    text = text.replace(old_svg_pid, new_svg_pid)
    print(f"✓ Replaced SVG '{old_svg_pid}' -> '{new_svg_pid}'")

# s5_ton SVG text (line 3632):
old_svg_ton = 'CMA-ES: K_ton(#1) | p_ton=15, p_f=250'
new_svg_ton = 'Calibrated: K_ton=5.26 | p_ton=16.2, p_f=250'
if old_svg_ton in text:
    text = text.replace(old_svg_ton, new_svg_ton)
    print(f"✓ Replaced SVG '{old_svg_ton}' -> '{new_svg_ton}'")

# s6_rect SVG text (both occurrences):
old_svg_rect = 'CMA-ES Mask: Kp_proprioception(#4)'
new_svg_rect = 'Calibrated: kp_prop = 0.4435 (lifts 1.1Hz to -0.99dB)'
if old_svg_rect in text:
    cnt_rect = text.count(old_svg_rect)
    text = text.replace(old_svg_rect, new_svg_rect)
    print(f"✓ Replaced {cnt_rect} occurrences of SVG '{old_svg_rect}'")

# s6_ccr SVG text (both occurrences):
old_svg_ccr = 'Lit: p_ccr = 100 rad/s, p_f = 250 rad/s'
new_svg_ccr = 'Strict Proper D=0: p_ccr=99.92 rad/s, z_ccr=0'
if old_svg_ccr in text:
    cnt_ccr = text.count(old_svg_ccr)
    text = text.replace(old_svg_ccr, new_svg_ccr)
    print(f"✓ Replaced {cnt_ccr} occurrences of SVG '{old_svg_ccr}'")

with open(HTML_PATH, "w", encoding="utf-8") as f:
    f.write(text)
print(f"Saved {HTML_PATH}")

shutil.copy2(HTML_PATH, ARTIFACT_PATH)
print(f"Mirrored to {ARTIFACT_PATH}")
