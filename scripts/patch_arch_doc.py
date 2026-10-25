path = r'C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\neuromotor_control_architecture_c4.md'
with open(path, encoding='utf-8') as f:
    text = f.read()

count = 0

def replace(text, old, new):
    global count
    if old in text:
        count += 1
        print(f'[OK] Replaced: ...{old[:70]}...')
        return text.replace(old, new)
    else:
        print(f'[MISS] Not found: ...{old[:70]}...')
        return text

# 1. Fix Delay buffer labels in mermaid (exact text check)
text = replace(text, '13 ms Axonal Conduction Latency', '35 ms Axonal Conduction Latency (VCR)')
text = replace(text, '18 ms Axonal Latency', '35 ms Axonal Conduction Latency (CCR)')

# 2. Fix QP lower bound in mermaid
text = replace(text, '0.01 <= u_i <= 1.0 (Bounded Co-contraction)', '0.03 <= u_i <= 1.0 (3% Co-contraction Floor)')

# 3. Section 3.3 LaTeX fixes: malformed s_max,min formula
old_smax = r'$$\text{Ia} = s_{\text{max}} + S \cdot s_{\text{min}}, \quad s_{\text{max,min}} = 0.5 \left( \text{AP}_{\text{bag1}} + \text{pri}_{\text{stat}} \pm \sqrt{(\text{AP}_{\text{bag1}} - \text{pri}_{\text{stat}})^2 + 10^{-4}} \right)$$'
new_smax = r'''$$\text{Ia} = s_{\max} + S \cdot s_{\min}, \quad S = 0.156$$
  $$s_{\max} = \tfrac{1}{2}\!\left(\text{AP}_{\text{bag1}} + \text{pri}_{\text{stat}} + \sqrt{(\text{AP}_{\text{bag1}} - \text{pri}_{\text{stat}})^2 + 10^{-4}}\right)$$
  $$s_{\min} = \tfrac{1}{2}\!\left(\text{AP}_{\text{bag1}} + \text{pri}_{\text{stat}} - \sqrt{(\text{AP}_{\text{bag1}} - \text{pri}_{\text{stat}})^2 + 10^{-4}}\right)$$
  $$\text{pri}_{\text{stat}} = G_{\text{pri,bag2}}\,\text{AP}_{\text{bag2}} + G_{\text{pri,chain}}\,\text{AP}_{\text{chain}}$$'''
text = replace(text, old_smax, new_smax)

# 4. Update CCR delay in Section 3.3
text = replace(text, 'Compensates for the $18\\text{ ms}$ neuromuscular transport delay via active phase lead:',
               'Compensates for the $35\\text{ ms}$ neuromuscular conduction delay via active phase lead:')

# 5. Fix QP lower bound in section 3.3
text = replace(text, r'0.01 \le u_i \le 1.0$$', r'0.03 \le u_i \le 1.0$$')

# 6. Fix reciprocal inhibition threshold (was u > 0.05, now smoothstep 0.03-0.08)
text = replace(text,
    'Presynaptic inhibition suppresses antagonist spindle reflexes when agonist alpha motor neurons are voluntarily excited ($u_\\alpha > 0.05$).',
    'Presynaptic inhibition suppresses antagonist stretch reflexes during voluntary excitation via a $C^1$-smoothstep gate between $u_\\alpha = 0.03$ (fully inhibited) and $u_\\alpha = 0.08$ (fully active).')

# 7. Fix reflex table delays
text = replace(text, '| **VCR Canal** | $13\\text{ ms}$', '| **VCR Canal** | $35\\text{ ms}$')
text = replace(text, '| **VCR Tonic** | $13\\text{ ms}$', '| **VCR Tonic** | $35\\text{ ms}$')
text = replace(text, '| **VCR Phasic** | $13\\text{ ms}$', '| **VCR Phasic** | $35\\text{ ms}$')
text = replace(text, '| **CCR Spindle** | $18\\text{ ms}$', '| **CCR Spindle** | $35\\text{ ms}$')

# 8. ESO: add note it is disabled
text = replace(text,
    '* **Cerebellar Extended State Observer (ESO):**\n  Reconstructs unmodeled low-frequency physical torques ($\\tau_{\\text{dist}}$) through a 2nd-order critically damped filter:\n  $$\\dot{x}_{\\text{eso,1}} = \\omega_o (y - x_{\\text{eso,1}}), \\quad \\dot{x}_{\\text{eso,2}} = \\omega_o (x_{\\text{eso,1}} - x_{\\text{eso,2}}), \\quad \\vec{\\tau}_{\\text{eso}} = -K_{\\text{eso}} x_{\\text{eso,2}}$$',
    '* **Cerebellar Extended State Observer (ESO):**\n  Reconstructs unmodeled low-frequency physical torques ($\\tau_{\\text{dist}}$) through a 2nd-order critically damped filter:\n  $$\\dot{x}_{\\text{eso,1}} = \\omega_o (y - x_{\\text{eso,1}}), \\quad \\dot{x}_{\\text{eso,2}} = \\omega_o (x_{\\text{eso,1}} - x_{\\text{eso,2}}), \\quad \\vec{\\tau}_{\\text{eso}} = -K_{\\text{eso}} x_{\\text{eso,2}}$$\n  > [!NOTE]\n  > **Disabled during Bode optimization:** $K_{\\text{eso}} = 0.0$ during Phase 2.1 CMA-ES campaigns. The ESO state equations are integrated but produce zero torque output. This prevents the disturbance observer from masking true reflex dynamics during frequency-sweep identification.')

print(f'\nTotal replacements applied: {count}')

with open(path, 'w', encoding='utf-8') as f:
    f.write(text)
print('File saved.')
