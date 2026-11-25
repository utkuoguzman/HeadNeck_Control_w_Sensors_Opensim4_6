# Future NEST Spiking Neural Network Phenomena (Deferred Tier C Adaptive Mechanisms)

> [!NOTE]
> **Architectural Boundary (OpenSim Continuous ODEs vs. NEST Spiking Co-Simulation)**  
> Mechanisms classified under **Tier C** are deliberately **excluded** from the current continuous-time OpenSim ODE controller ([HeadNeckNeuralController.cpp](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp)) to keep the continuous baseline minimal, non-overparameterized, and grounded directly in core vestibular, spinal, and biomechanical anatomy (**Tier A** & **Tier B**; see [tier_a_b_neurobiomechanical_plan.md](tier_a_b_neurobiomechanical_plan.md)).  
> When this musculoskeletal plant is coupled to a **NEST spiking neural network (SNN)**, the adaptive, plastic, probabilistic, and contextual phenomena below emerge naturally from real spiking synapses and microcircuits and should be implemented directly in NEST.

---

## 1. Tier C Phenomena Catalog for Future NEST Implementation

### C1. Use-Dependent Short-Term Synaptic Depression (Tsodyks–Markram STD) at Irregular Vestibular Afferent → VN Synapses

- **Neurophysiological Literature:**
  - *Bagnall, McElvain, Faulstich & du Lac (2008), J. Neurosci.* — Firing precision and synaptic efficacy across vestibular nucleus neuron types.
  - *McElvain, Bagnall, Sakatos & du Lac (2010), Neuron*; *McElvain et al. (2015)* — Strong activity-dependent short-term depression at vestibular afferent → vestibular nucleus (VN) excitatory synapses driven by high-rate/irregular bursts.
  - *Tsodyks & Markram (1997), PNAS* — Canonical vesicle-depletion / utilization formalism.
- **Continuous ODE Reference (for validation / mean-field comparison):**
  $$\dot{x}_{\text{res}} = \frac{1 - x_{\text{res}}}{\tau_{\text{rec}}} - U\,|f_{\text{irr}}|\,x_{\text{res}}, \qquad f_{\text{syn}} = x_{\text{res}}\,f_{\text{irr}}$$
  Eliminates any arbitrary `m/s²` acceleration threshold cliff (replacing phenomenological knees such as `highg_oto_knee_mps2 = 7.80`).
- **NEST Realization:**
  - Connect irregular CN VIII afferent Poisson/inhomogeneous-Poisson or LIF encoder populations (`irr_scc_*`, `irr_oto_*`) to second-order VN excitatory/inhibitory neurons via native NEST **`tsodyks_synapse`** or **`tsodyks2_synapse`** (tuning release probability `U` and vesicle recovery time constant $\tau_{\text{rec}}$; zero or weak facilitation $\tau_{\text{fac}} \approx 0$).
  - Regular afferent populations target VN/cerebellar populations with nearly linear / weak-STD synapses, preserving steady tonic gravity and low-frequency velocity fidelity.
- **Critical Dynamical Caveat Recorded from Stage A Torque X-Ray:**
  - During the `Sled 2g/3g` Stage A investigation (`t = 56–112 ms`), we established that **fast STD alone passes the initial `20–30 ms` onset transient before vesicles deplete**, which can *accentuate* an early onset spike (`88 ms` *splenius capitis* hump) and deepen the subsequent `128 ms` cleft if upstream signals are unphysically sharp.
  - Therefore, in NEST, `tsodyks_synapse` **must sit downstream of realistic peripheral irregular firing-rate saturation** (Linear–Nonlinear cascade saturation, e.g., Schneider et al., 2015) and **without** an unphysiological central double-differentiator (`z_phas → 0`) stacked on top of `H_irr(s)`.

---

### C2. Continuous Bayesian / Inverse-Variance Multisensory Reweighting & Cerebellar `SE2(3)` Innovation Weighting

- **Neurophysiological Literature:**
  - *Peterka (2002), J. Neurophysiol.* 88:1097–1118 — Amplitude-dependent multisensory reweighting between vestibular, proprioceptive, and visual cues in human postural stabilization.
  - *Ernst & Banks (2002), Nature* — Statistically optimal inverse-variance multisensory cue integration.
  - *Ma, Beck, Latham & Pouget (2006), Nat. Neurosci.*; *Carandini & Heeger (2012), Nat. Rev. Neurosci.* — Probabilistic population codes and canonical divisive normalization.
- **Continuous ODE Reference:**
  $$w_{\text{phas}}(t) = \frac{1}{1 + \left(A(t) / A_0\right)^2}$$
  where $A(t)$ tracks either gravito-inertial acceleration magnitude or cerebellar `SE2(3)` internal-model innovation $\|\tilde{\mathbf{a}}\|$, avoiding step-like threshold cliffs (`g_highg: 1.000 → 0.005` over `7.80 ± 0.65 m/s²`).
- **NEST Realization:**
  - Model cerebellar nodulus/uvula granule-cell layer integration across competing mossy-fiber modalities:
    1. CN VIII vestibular afferents (otoliths + canals),
    2. External Cuneate Nucleus proprioceptive afferents (`Ia`/`II`/`Ib`),
    3. Spinal/cuneate articular mechanoreceptor streams (Ruffini/Pacinian/Golgi-like cervical capsule afferents).
  - Realize inverse-variance reweighting through **divisive shunting inhibition** (`Golgi cells` / GABAergic interneurons using conductance-based LIF/AdEx neurons, e.g., `aeif_cond_exp` / `iaf_cond_exp`) rather than hardcoded polynomial `smoothstep` curves: higher sensory variance / burst rate raises inhibitory conductance $g_{\text{inh}}$, automatically reducing sensory gain smoothly as membrane division $1/(g_L + g_{\text{inh}})$.

---

### C3. Cerebellar Efference-Copy Attenuation of Vestibular-Only (VO) Neurons During Active Effort & Bracing

- **Neurophysiological Literature:**
  - *Roy & Cullen (2004), J. Neurosci.* — Dissociating self-generated vs. passively applied head motion in primate vestibular-only (VO) neurons.
  - *Brooks & Cullen (2013, 2014), Curr. Biol.* — Cerebellar fastigial nucleus prediction errors gate brainstem VO premotor neurons.
  - *Cullen (2019), Annu. Rev. Neurosci.* — ~`70%` suppression of VO neuron sensitivity during active self-motion / braced motor effort.
- **Continuous ODE Scope Note:**
  - In [HeadNeckNeuralController.h](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.h#L128), `highg_brace_phas_suppress` models this mechanism strictly for active/braced efforts (`urgency > 0.50`, `C^1`-zero for `u <= 0.50`).
  - **Honest scope boundary:** Cerebellar VO efference-copy cancellation **cannot** explain passive/unbraced `u <= 0` conditions because no voluntary bracing command is being generated.
- **NEST Realization:**
  - Implement an explicit **Cerebellar Cortical → Rostral Fastigial Nucleus (rFN) → Brainstem VO** cancellation microcircuit:
    - Corticopontine & spinal copy of active voluntary / bracing effort excites cerebellar **granule cells → parallel fibers → Purkinje cells**, alongside an internal forward-model sensory prediction.
    - Expected vestibular consequences inhibit rFN / VO neurons via **Purkinje GABAergic inhibition** paired with **climbing-fiber (inferior olive) PF–PC LTD/LTP plasticity**, preserving full VO sensitivity to unexpected external impacts (`u = 0` or `-0.75`) while selectively gating out expected self-bracing motion (`u > 0`).

---

### C4. Bandwidth- & Context-Dependent VCR–CCR Sensory Reweighting (Sustained Vibration vs. Transient Impulse)

- **Neurophysiological Literature:**
  - *Happee, de Bruijn, Forbes & van der Helm (2017), J. Biomech.* 58:203–211 — Humans modulate vestibulo-collic (VCR, head-in-space) vs. cervico-collic (CCR, head-on-trunk) weighting across perturbation bandwidth and visual/task context.
  - *Forbes et al. (2013, 2018)*; *Assländer & Peterka (2014)* — Dynamic context adaptation during multi-second perturbations.
- **NEST Realization:**
  - Slow (~`0.5–2.0 s`) descending prefrontal/reticular context modulation and slow cerebellar internal-model gain adaptation (e.g., neuromodulatory / tonic background drive onto VN vs. C1–C8 cervical interneurons) distinguishing sustained rhythmic vehicle vibration from single discrete transient impacts.
- **Explicit Methodological Caution Recorded (Do NOT Hack in Continuous ODEs):**
  - During Stage A analysis we observed that a slow RMS/bandwidth filter starting at `0` from rest is mathematically one of the few *causal* separators of multi-second `Bode_APEO` from `t = 50–80 ms` of a single sled pulse.
  - **Why we rejected this in continuous ODE form:** A tough thesis examiner would immediately flag a quiet-to-active RMS multiplier that suppresses early impact VCR gain as an artificial "Bode-vs-sled detector", because unexpected first-trial impacts in humans typically **augment** (rather than suppress) early EMG (see **C5**). In NEST, contextual reweighting must emerge explicitly from persistent cortical task state (`expected vibration` vs. `alert startle ready`), never from an implicit transient gate.

---

### C5. Reticulospinal Startle-Augmented Early Neck Recruitment & Multi-Trial Habituation

- **Neurophysiological Literature:**
  - *Blouin, Inglis & Siegmund (2006), J. Physiol.* 573:857–867 & *Spine* 31:146–154 — Unexpected (first-trial) seated sled perturbations evoke a short-latency reticulospinal startle + vestibular neck EMG burst (`~60–90 ms`, `10–20 Hz` synchronous drive) ahead of the main kinematically driven postural burst (`~130–180 ms`), whereas relaxed or habituated repeated trials lose this early burst cleanly (producing a unimodal response matching *Zheng et al., 2021*).
  - *Siegmund, Sanderson, Myers & Inglis (2003), J. Physiol.* — Habituation of neck muscle responses across repeated whiplash-like perturbations.
- **Empirical Connection to Our Clean $C^1$ Baseline:**
  - On our verified clean $C^1$ baseline ([run_zheng_2g_3g_urgency_sweep.py](../scripts/run_zheng_2g_3g_urgency_sweep.py)), relaxed/unaware NBDL urgencies (`u <= -0.50`, including NBDL `u = -0.75`) already exhibit a **strictly unimodal** *splenius capitis* activation (`nPk = 0..1`) dead-center inside human NBDL flexion corridors (`+37.48°` in `33–40°` at `2g`; `+45.93°` in `40–47°` at `3g`), while alert `u >= -0.25` displays the two-phase (`~88 ms` + `~160 ms`) envelope.
- **NEST Realization:**
  - Model a dedicated **Caudal Pontine Reticular Nucleus (PnC)** startle pathway alongside the vestibulospinal tract (MVST/LVST):
    - Multimodal abrupt vestibular + cutaneous/proprioceptive jerk triggers PnC giant neurons onto cervical $\alpha$-motoneurons (`~60–90 ms` impact latency).
    - Equip afferent → PnC synapses with **multi-trial habituating depression** (long-recovery depressing plasticity / inhibitory gating from prefrontal expectancy), reproducing *first-trial bimodal alert response → habituated unimodal NBDL response* end-to-end across sequential OpenSim–NEST episodes.

---

### C6. Supraspinal / Cerebellar Ascending Group `Ib` Reafference Cancellation (DSCT / ECN → Cerebellum)

- **Neurophysiological Literature:**
  - *Fournier, Katz & Pierrot-Deseilligny (1983), Exp. Brain Res.* & *Lafleur et al. (1992), J. Physiol.* — Unlike muscle spindles ($\gamma$-fusimotor control), **Golgi Tendon Organs (GTOs) have zero efferent innervation** and no spinal forward-model subtraction; voluntary suppression of homonymous spinal `Ib` inhibition occurs via descending cortical/brainstem interneuronal depression and presynaptic primary afferent depolarization (PAD), which is already modeled as **Layer 2 voluntary gating** (`1 - K_gto_vol_suppress * voluntary_activity_gate`) in [HeadNeckNeuralController.cpp](../HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp).
  - *Pierrot-Deseilligny & Burke (2012), The Circuitry of the Human Spinal Cord*; *Brooks & Cullen (2013), Curr. Biol.* — Distinguishing self-generated vs. externally imposed muscle-tendon forces occurs **supraspinally** along ascending dorsal spinocerebellar / external cuneate (DSCT / ECN) pathways projecting to cerebellar cortex and deep nuclei.
- **Continuous ODE Scope Note:**
  - In our continuous Tier B implementation, the `216`-state spinal Reference-GTO ODE layer was deliberately removed in favor of textbook spinal Layer-2 descending interneuronal depression/PAD plus physiological homonymous `Ib` force-limiting gain (`K_gto_ib ≈ 0.0004–0.0005`, ~`5–10%` MVC at `3g` peak tension).
- **NEST Realization:**
  - Route C1–C8 DRG `Ib` spikes into **External Cuneate / Cervical Spinocerebellar (ECN/CSCT)** projection neurons targeting cerebellar granule cells in parallel with motor efference-copy mossy fibers.
  - Train PF–PC synapses via climbing-fiber error signals to cancel self-generated voluntary tendon tension at deep cerebellar / rFN output neurons, isolating unexpected external load transients for supraspinal postural corrections without inventing an unphysiological spinal forward model.

---

## 2. Planned OpenSim ↔ NEST Co-Simulation Interface Bridge

When moving from standalone C++ control to closed-loop **OpenSim (Musculoskeletal Plant + Receptors) ↔ NEST (Spiking CNS)** co-simulation, our strict **zero-ground-truth rule** means every sensory stream listed below is already isolated as clean physical receptor outputs:

| Sensory Stream | OpenSim Component & Output Channel | Axonal Transport Delay to CNS | Target NEST Encoding Population | Target NEST Plasticity / Microcircuit Roles |
| :--- | :--- | :---: | :--- | :--- |
| **Regular / Irregular Canals** | `HeadNeckNeuralController` 6-canal `H_scc` (`vestibular_omega` / `irr`) | `2.0 ms` (`VN`), `3.0 ms` (`CB`) | CN VIII Poisson/LIF rate encoders | `tsodyks_synapse` (C1), divisive normalization (C2), Purkinje→rFN→VO (C3) |
| **Regular / Irregular Otoliths** | `HeadNeckNeuralController` 3D `H_oto` (`reg` & single-`tanh` LN `irr`, `z_phas=0`) | `2.0 ms` (`VN`), `3.0 ms` (`CB`) | Macular regular + LN-saturated irregular spike banks | `tsodyks_synapse` (C1), PnC startle trigger (C5), reweighting (C2) |
| **Muscle Spindles (`Ia`, `II`) + Equilibrated SRS** | `Millard12EqMuscleWithAfferents` (`primary_Ia`, `secondary_II`, 72 neck muscles with tendon-equilibrated Cui 2008 / Kirsch 1994 / De Groote 2018 SRS) | `1.0 ms` (`Ia`), `2.5 ms` (`II`→C1–C8), `3.0 ms` (`CB`) | C1–C8 DRG Ia/II afferent pools | Spinal stretch/reciprocal loops + ECN mossy fibers (C2) |
| **Golgi Tendon Organs (`Ib`)** | `Lin02GolgiTendonOrgan` (`tendon_Ib`, equilibrated tendon force, Layer-2 voluntary PAD gating) | `1.2 ms` (`Ib` axon) | C1–C8 DRG Ib afferent pools | Spinal disynaptic Ib interneurons + supraspinal ECN→cerebellar reafference cancellation (C6) |
| **Cervical Ligaments & Facet Capsules** | `DynamicLigamentForce` (67 Kuo et al. 2019 non-disc ligaments + `B_k50_c30` disc bushings; Lu 2005 / Chen 2006 / McLain 1994 Type II/III reflex) | `15.0 ms` total loop (`3.0 ms` articular Group II/III + polysynaptic) | Segmental articular DRG encoders | Protective ligamento-muscular synergy + PnC shock trigger (C5) |
| **Descending Motor Commands** | External NEST spike decoders → OpenSim 72×3 controls (`α, γ_d, γ_s`) | `2.5 ms` ventral root (`τ_7A`, `τ_7B`) | Lamina IX α/γ motoneuron pools + Renshaw cells | Population rate → Hill+SRS activation + fusimotor sensitivity |
