path = r'C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\neuromotor_control_architecture_c4.md'
with open(path, encoding='utf-8') as f:
    text = f.read()

# Find and replace section 3.3 entirely (from heading to the --- divider after QP)
start_marker = '### 3.3 Cervical Spinal Cord (CCR Reflexes & Muscle Allocation)\n'
end_marker = '\n---\n\n## 4. Reflex Integration Summary Table'

start_idx = text.find(start_marker)
end_idx = text.find(end_marker)

if start_idx == -1 or end_idx == -1:
    print(f"Markers not found! start={start_idx}, end={end_idx}")
else:
    old_section = text[start_idx:end_idx]
    print(f"Found section 3.3 ({len(old_section)} chars), replacing...")

    new_section = r"""### 3.3 Cervical Spinal Cord (CCR Reflexes & Muscle Allocation)
* **Biophysical Muscle Spindles (Mileusnic et al. 2006):**
  Computes primary (Ia) nerve firing based on intrafusal mechanical tension:
  $$\text{Ia} = s_{\max} + S \cdot s_{\min}, \quad S = 0.156$$
  $$s_{\max} = \tfrac{1}{2}\!\left(\text{AP}_{\text{bag1}} + \text{pri}_{\text{stat}} + \sqrt{(\text{AP}_{\text{bag1}} - \text{pri}_{\text{stat}})^2 + 10^{-4}}\right)$$
  $$s_{\min} = \tfrac{1}{2}\!\left(\text{AP}_{\text{bag1}} + \text{pri}_{\text{stat}} - \sqrt{(\text{AP}_{\text{bag1}} - \text{pri}_{\text{stat}})^2 + 10^{-4}}\right)$$
  $$\text{pri}_{\text{stat}} = G_{\text{pri,bag2}}\,\text{AP}_{\text{bag2}} + G_{\text{pri,chain}}\,\text{AP}_{\text{chain}}$$
  Dynamic Bag 1 accounts for velocity sensitivity via a non-linear power law ($v^{0.6}$); Static Bag 2 and Chain fibers encode length offset.

  > [!IMPORTANT]
  > **Afferent usage in the CCR loop:** The Mileusnic (2006) spindle model internally computes all three afferent signals. Only **Group Ia (primary)** is logged in the neural history buffer, propagated through the CCR delay, and injected into $H_{\text{ccr}}(s)$. **Group II (secondary)** is computed as the intermediate `pri_stat` term for the Ia blending formula but is **never written to the CCR buffer or used as a separate feedback channel**. **Group Ib (Golgi Tendon Organ)** is computed by the proprioception plugin but is **completely unused** in the reflex controller. Both omissions are deliberate: Group Ia already carries the composite velocity + length signal sufficient for Bode identification, and Ib negative force feedback would destabilize the closed loop at resonance.

* **Alpha–Gamma Coactivation (Fusimotor Drive):**
  Alpha motor commands $u_\alpha$ drive proportional fusimotor coactivation:
  $$\gamma_{\text{dyn},i} = K_{\gamma,\text{dyn}} \cdot u_{\alpha,i}, \quad \gamma_{\text{stat},i} = K_{\gamma,\text{stat}} \cdot u_{\alpha,i}$$
  $\gamma_{\text{dyn}}$ controls Dynamic Bag 1 contractile stiffness (velocity damping gain); $\gamma_{\text{stat}}$ maintains Bag 2 and Chain baseline tension so spindles never go slack during active shortening. The dead parameter `k_v` (hardcoded derivative gain on raw stretch velocity) has been **removed**; velocity damping is now exclusively controlled by the physiological $K_{\gamma,\text{dyn}}$ pathway.

* **Active Zero Placement via Lead Ratio Coupling:**
  Zero frequencies for both VCR and CCR filters are coupled to their poles via biological ratios:
  $$z_{\text{sc}} = r_{\text{lead,sc}} \cdot p_{\text{sc}}, \quad r_{\text{lead,sc}} \in [0.10,\, 0.45]$$
  $$z_{\text{ccr}} = r_{\text{lead,ccr}} \cdot p_{\text{ccr}}, \quad r_{\text{lead,ccr}} \in [0.10,\, 0.45]$$
  Maximum phase lead occurs at $\omega_m = \sqrt{z \cdot p}$. With $r \approx 0.20$–$0.25$, peak lead falls at $\omega_m \approx 8$–$14\text{ rad/s}$ ($1.3$–$2.2\text{ Hz}$) — directly inside the $1.11$–$2.48\text{ Hz}$ resonance transition corridor where the $35\text{ ms}$ delay causes the greatest phase lag.

* **Physiological Parameter Bounds (Phase 2.1):**
  CMA-ES search space is bounded to prevent physiologically implausible solutions:

  | Parameter | Symbol | Lower | Upper | Rationale |
  | :--- | :--- | :---: | :---: | :--- |
  | Canal gain | $K_{\text{sc}}$ | 35 | 95 Nm/(rad/s) | Prevents optimizer from slamming against the 120 hardware ceiling |
  | Spindle stiffness | $k_p$ | 0.80 | 1.70 | Prevents high-stiffness blowouts (Gen 5 failure: $k_p = 1.89$) |
  | Tonic ratio | $r_{\text{ton}}$ | 0.50 | 0.95 | Maintains $K_{\text{ton}} \leq K_{\text{sc}}$ (tonic ≤ canal phasic drive) |
  | Lead ratios | $r_{\text{lead}}$ | 0.10 | 0.45 | Guarantees $30^\circ$–$50^\circ$ phase lead in the resonance corridor |

* **Equilibrium Point ($\lambda$-Model) Fusimotor Biasing:**
  Descending voluntary actions deform the spindle baseline length ($L_{\text{eff}} = L_{\text{norm}} - \Delta L_{\text{task}}$), cancelling self-generated stretch reflex firing during voluntary movement.
* **Spinal Lead-Lag Reflex ($H_{\text{ccr}}(s)$):**
  Compensates for the $35\text{ ms}$ neuromuscular conduction delay via active phase lead:
  $$H_{\text{ccr}}(s) = k_p \frac{s + z_{\text{ccr}}}{s + p_{\text{ccr}}}$$
* **Synergy-Based Reciprocal Inhibition:**
  Presynaptic inhibition suppresses antagonist stretch reflexes during voluntary excitation via a $C^1$-smoothstep gate between $u_\alpha = 0.03$ (fully inhibited) and $u_\alpha = 0.08$ (fully active).
* **Quadratic Programming (QP) Torque Allocation:**
  Solves the multi-muscle redundancy problem for 72 muscles in real time:
  $$\min_{\mathbf{u}} \frac{1}{2} \sum_{i=1}^{72} u_i^2 \quad \text{subject to} \quad \sum_{i=1}^{72} \vec{r}_i \times \vec{F}_{\text{max}, i} u_i = \vec{\tau}_{\text{total}}, \quad 0.03 \le u_i \le 1.0$$"""

    text = text[:start_idx] + new_section + text[end_idx:]
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    print(f"Done. Section 3.3 replaced.")

