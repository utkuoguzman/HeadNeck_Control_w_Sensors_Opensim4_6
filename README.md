# Biomimetic Head-Neck-Eye Neuromotor System in OpenSim 4.6
### 100% Sensor-Driven Closed-Loop Control with Per-Sensor Luenberger Observers, Multisensory Mahony Fusion, and Spinal/Brainstem Reflexes

[![OpenSim](https://img.shields.io/badge/OpenSim-4.6-blue.svg)](https://opensim.stanford.edu/)
[![C++](https://img.shields.io/badge/C%2B%2B-17%20(AVX2%20%2B%20LTCG)-00599C.svg?logo=c%2B%2B)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg?logo=python)](https://python.org/)
[![Optimization](https://img.shields.io/badge/CMA--ES-Phase%202.4%20(16D%2C%20popsize%3D28%2C%20TPA%20%2B%20Orthogonal%20%2B%20GP)-brightgreen.svg)](https://github.com/CMA-ES/pycma)
[![Sensor-Only](https://img.shields.io/badge/CNS%20State%20Estimation-100%25%20Sensor--Driven%20(r%3D0.9997)-brightgreen.svg)](docs/neuromotor_control_architecture_c4.md)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

---

## 🧭 Documentation Map (Start Here)

To keep documentation clean and non-redundant, the mathematical formulations, interactive block diagrams, and engineering rationale are organized into two dedicated resources:

1. 🖥️ **[Interactive Simulink & Anatomical Explorer (`docs/neuromotor_simulink_explorer.html`)](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html)**  
   *Click to launch the live interactive web app in your browser ([view source](docs/neuromotor_simulink_explorer.html)).* Contains the full Simulink-style block diagrams for the Top-Level Loop and all 7 Subsystems, interactive 3D brain region highlighting, annotated LaTeX transfer functions, `[Derivation]` analytical constant derivations, and universal pseudocode for every block.
2. 📘 **[Engineering, Neurophysiological & Optimization Design Rationale (`docs/neuromotor_control_architecture_c4.md`)](docs/neuromotor_control_architecture_c4.md)**  
   Explains **why** each design choice was made:
   - The **Zero-Ground-Truth Rule** and dual-stream sensory split (Reflex vs. Luenberger Observer pathways).
   - **Mahony `SE_2(3)` Observer Verification** & progressive augmentations against Simbody ground truth ($r = +0.9997$, $\text{RMSE} = 0.719^\circ$).
   - **Sensory Transfer Function Modifications**, including the biproper *Schneider et al. (2015) / Happee et al. (2017)* otolith formulation (1st-order Taylor/Padé lead-lag, passband normalization, and 2-stage minimum-phase Luenberger inverse) and continuous ring-buffer delays.
   - **5-Delay Causal Decomposition** ($\tau_{\text{vest}} = 2\text{ ms}$ CN VIII, $\tau_{\text{prop}} = 7\text{ ms}$ shared proprioceptive afferent to Sub 4 & Sub 6, $\tau_{\text{asc}} = 20\text{ ms}$ thalamocortical, $\tau_{\text{desc}} = 12\text{ ms}$ unified descending pyramidal to Sub 5 & Sub 7, and $\tau_{\text{spinal}} = 8\text{ ms}$ shared C1–C8 spinal QP/motoneuron/ventral-root delay—yielding $\text{VCR} = 10\text{ ms}$, $\text{CCR} = 15\text{ ms}$, and $\text{Cortical} = 40\text{ ms}$).
   - **VCR & CCR Transfer Function Design** (center-frequency $\sqrt{r_{\text{lead}}}$ normalization, strictly-proper $D=0$ phasic otolith filter, physiological sensory & reflex recruitment thresholds, and voluntary-gated Sherrington reciprocal inhibition).
   - **Per-Muscle Literature-Centered 6-DOF Equilibrium Co-Contraction & Incremental Active-Set QP** (all 72 individual cervical muscles centered inside human EMG `%MVC` literature bands `1.0–2.5%`, `2.0–4.0%`, `2.0–5.0%`, `3.0–7.0%` to achieve exact 6-DOF upright static equilibrium with zero $t=0$ integrator preload, resting spindle fusimotor initialization, and incremental $\Delta u_i = u_{\alpha,i} - u_{\min,i}$ QP allocation) & **Paradigm A 4-Zone ROM Containment** ($75\%$ deadband early-abort guard).
   - **Intel AVX2 + LTCG Vectorization** & the **7 CMA-ES Algorithmic & Multibody Accelerators** (including oculomotor coordinate locking and extraocular muscle isolation during cervical campaigns).

---

## 📌 System Overview

This repository implements a biophysically grounded **3D musculoskeletal head-neck-eye neuromotor model** in **OpenSim 4.6** (`72` Hill-type cervical muscles, `6-DOF` upper/lower cervical spine). Operating under a **strict zero-ground-truth rule**, the Central Nervous System (CNS) never queries Simbody joint coordinates (`q`), joint velocities (`u`), or muscle lengths (`m.getLength`). Instead, all state estimation and reflex stabilization emerge from seven modular subsystems:

| Subsystem | Functional Role & Biological Substrates | Details & Equations |
| :--- | :--- | :---: |
| **Subsystem 1: Voluntary Motor Cortex** | 2nd-order critically damped trajectory generator, $20\text{ ms}$ ascending thalamocortical delay, decoupled task-space PID ($K_{i,\text{pitch}}$ with zero $t=0$ preload $I_\theta(0)=0$ and $K_{i,\text{roll}}=K_{i,\text{yaw}}=0$), $2\text{ Hz}$ cortical LPF, and unified $\tau_{\text{desc}} = 12\text{ ms}$ descending pyramidal delay to Sub 5 & Sub 7. | [Open in HTML Explorer](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) |
| **Subsystem 2: Vestibular Labyrinth** | 6 non-orthogonal semicircular canals (*Schneider et al., 2015*) + 3D utricle/saccule otolith maculae (*Schneider 2015 / Happee 2017 Biproper*). Splits **Irregular** phase-leading streams to VCR and **Regular** streams through biproper **Canal & 2-Stage Otolith Luenberger Observers** to Mahony fusion. | [Open in HTML Explorer](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) · [Rationale §3](docs/neuromotor_control_architecture_c4.md#3-sensory-transfer-function-modifications--realizations) |
| **Subsystem 3: 72-Spindles, Prop. Luenberger & Decoder** | 72 intrafusal muscle spindles (*Mileusnic et al., 2006*) initialized at resting coactivation equilibrium ($f_{\text{dyn},0}, f_{\text{stat},0}, f_{\text{chain},0}$) outputting Primary `Ia` and Secondary `II` firing rates ($\tau_{\text{prop}} = 7\text{ ms}$ to both Sub 4 and Sub 6), 72 **4-state Per-Spindle Luenberger Observers** (288 ODE states) removing incremental $\gamma_{\text{dyn}}/\gamma_{\text{stat}}$ efference copies, and a **$6\times 72$ Cervical Jacobian Decoder** eliminating S-shape intervertebral buckling cross-talk. | [Open in HTML Explorer](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) · [Rationale §2](docs/neuromotor_control_architecture_c4.md#2-qualitative--quantitative-verification-of-the-mahony-observer--its-augmentations-against-ground-truth) |
| **Subsystem 4: Mahony `SE_2(3)` Observer** | Multi-delay cerebellar Smith-compensated quaternion observer fusing $2\text{ ms}$-delayed Canal Luenberger velocity, $2\text{ ms}$-delayed Otolith Luenberger gravitoinertial force, and $7\text{ ms}$-delayed 6-DOF proprioceptive neck posture ($r = +0.9997$ vs. Simbody ground truth). | [Open in HTML Explorer](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) · [Rationale §2](docs/neuromotor_control_architecture_c4.md#2-qualitative--quantitative-verification-of-the-mahony-observer--its-augmentations-against-ground-truth) |
| **Subsystem 5: Vestibulo-Collic Reflex (VCR)** | Brainstem VCR ($\tau_{\text{vest}} = 2\text{ ms}$ input + $\tau_{\text{spinal}} = 8\text{ ms}$ in Sub 7 $= 10\text{ ms}$ total) combining Tonic Spatial Attitude LPF (`H_ton`, $D=0$), center-frequency normalized Semicircular Canal lead-lag (`H_sc`), and 1-state strictly-proper Irregular Otolith Phasic LPF (`H_phas`, $D=0$) with physiological sensory deadzone thresholds ($0.002\text{ rad}$, $0.005\text{ rad/s}$, $0.020\text{ m/s}^2$), matched-adaptation gravity cancellation, and a $+10\text{ ms}$ **Cerebellar Efference Forward Predictor** (*Roy & Cullen, 2004; Cullen, 2012; Brooks & Cullen, 2013, 2014*) synchronizing corticobulbar efference copies from $-12\text{ ms}$ to $-2\text{ ms}$. | [Open in HTML Explorer](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) · [Rationale §5](docs/neuromotor_control_architecture_c4.md#5-why-the-vcr--ccr-reflex-transfer-functions-are-formulated-this-way) |
| **Subsystem 6: Spinal CCR Reflex Filters** | Segmental C1–C8 Cervico-Collic Reflex (`CCR`) receiving the $7\text{ ms}$-delayed `Ia` spindle stretch from Subsystem 3 with $0\text{ ms}$ extra internal delay ($7 + 0 + 8 = 15\text{ ms}$ total CCR loop), applying the $0.5\text{ imp/s}$ synaptic recruitment threshold, filtering through a 72-channel center-frequency normalized lead-lag transfer function (`H_ccr`) and excitatory half-wave rectification (`s6_rect`) to output `spinal_ccr_drive[72]`. | [Open in HTML Explorer](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) · [Rationale §5](docs/neuromotor_control_architecture_c4.md#5-why-the-vcr--ccr-reflex-transfer-functions-are-formulated-this-way) |
| **Subsystem 7: Active-Set QP & Motoneuron Pool** | Final C1–C8 spinal motor integration stage receiving `voluntary_torque[3]` (Subsystem 1), `vestibulo_collic_torque[3]` (Subsystem 5), and `spinal_ccr_drive[72]` (Subsystem 6). Executes a 15-iteration **Incremental Active-Set Cholesky/LU Quadratic Programming (QP)** synergy solver ($\Delta u_i = u_{\alpha,i} - u_{\min,i} \in [0, 1 - u_{\min,i}]$) around the **72-muscle literature-centered 6-DOF static equilibrium co-contraction baseline**, voluntary-gated Sherrington reciprocal inhibition, Lamina IX motoneuron pool summation, the shared **$\tau_{\text{spinal}} = 8\text{ ms}$ spinal/ventral-root execution delay**, and supraspinal-only $\alpha\text{–}\gamma$ coactivation (*Hulliger, 1984*). | [Open in HTML Explorer](https://raw.githack.com/utkuoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/Harpee2017/docs/neuromotor_simulink_explorer.html) · [Rationale §6](docs/neuromotor_control_architecture_c4.md#6-per-muscle-literature-centered-6-dof-equilibrium-co-contraction--incremental-active-set-qp) |

---

## 📂 Repository Directory Layout

```text
HeadNeck_Control_w_Sensors_Opensim4_6/
├── docs/                                    # Interactive Explorer & Engineering Rationale
│   ├── neuromotor_simulink_explorer.html    # Interactive Simulink diagrams, LaTeX & [Derivation] cards
│   └── neuromotor_control_architecture_c4.md# Design rationale, verification, ROM, zones & CMA-ES
├── osim_files/                              # OpenSim 4.6 musculoskeletal models (.osim)
│   ├── HYOID_HeadNeckNeural_Model_Detailed.osim   # Canonical model (Paradigm A 4-zone ROM limits)
│   └── HYOID_HeadNeckNeural_Model_WithLimits.osim
├── proprioception_plugin/                   # Mileusnic (2006) muscle spindle C++ plugin (AVX2)
├── vestibular_plugin/                       # Schneider (2015) vestibular afferent C++ plugin
├── HeadNeckNeuralController_plugin/         # CNS Controller, Luenbergers, Mahony, VCR, CCR & QP (AVX2)
├── FixationController_plugin/               # Binocular gaze & VOR/COR controller
├── data/                                    # Initial centroids & pre-screen champions
│   └── prescreen_rational_champion.json     # First-principles rational warm-start centroid (x0)
├── scripts/                                 # Optimization campaigns & reporting tools
│   ├── run_phase2_4_campaign.py             # 16D CMA-ES campaign (TPA + Orthogonal + Local GP Sieve)
│   └── build_phase2_progress_table.py
├── logs/                                    # Campaign logs, per-frequency JSON cache & progress tables
└── cmaes_worker_p22.py                      # Single-frequency OpenSim worker with 75% ROM abort guard
```

---

## 🛠️ Quick Start & Build Instructions

### Prerequisites
* **OS & Compiler:** Windows 10/11 x64, Visual Studio 2022 (MSVC C++17 with `/arch:AVX2 /GL /LTCG`), CMake 3.22+
* **Simulation & Python:** OpenSim 4.6 (`C:\OpenSim 4.6`), Conda environment `opensim_env` (`python>=3.10`, `opensim`, `numpy`, `scipy`, `cma`)

### 1. Build the C++ OpenSim Plugins (Release with Intel AVX2 + LTCG)

```powershell
cmake --build proprioception_plugin/build --config Release
cmake --build vestibular_plugin/build --config Release
cmake --build HeadNeckNeuralController_plugin/build --config Release
cmake --build FixationController_plugin/build --config Release
```

### 2. Run the Phase 2.4 CMA-ES Optimization Campaign

Evaluates 28 candidates per generation (`2 × 14` physical CPU core waves) across the 6 canonical *Happee et al. (2017)* perturbation frequencies using TPA + Block-Orthogonal sampling and local Gaussian Process pre-screening (see [Rationale §8](docs/neuromotor_control_architecture_c4.md#8-c-hardware-vectorization-intel-avx2--cma-es-acceleration-architecture)):

```powershell
C:\Users\User\miniforge3\envs\opensim_env\python.exe scripts/run_phase2_4_campaign.py
```

---

## 🗺️ Project Roadmap

- [x] **Phase 8.1: Biomechanical Infrastructure & Sensor Plugins**
  - [x] OpenSim 4.6 integration with implicit `SemiExplicitEuler2` integrators.
  - [x] Mileusnic intrafusal spindle model smoothed for $C^2$ continuous solver stability.
  - [x] Bilateral non-orthogonal semicircular canal & otolith digital filter bank.
- [x] **Phase 8.2: Human Kinematics Benchmark Validation**
  - [x] Happee 2017 multi-frequency seat perturbation protocol ($0.31 - 7.95\text{ Hz}$).
  - [x] 18-core CMA-ES parallel optimization of 12 neuromotor reflex parameters.
  - [x] Discovery of active phase-lead zeros ($z_{\text{ccr}} \approx 298\text{ rad/s}, z_{\text{sc}} \approx 491\text{ rad/s}$).
  - [ ] Optimization: is about to be ironed out.
- [x] **Phase 8.3: Human Voluntary Tracking Benchmark Validation**
  - [x] Replicate the human voluntary velocity profiles in the literature.
- [ ] **Phase 8.4: Ocular-Cervical Coordination (VOR / COR)**
  - [ ] Connect vestibular canal outputs directly to FixationController eye muscles.
  - [ ] Replicate retinal slip minimization under erratic head perturbation.
- [ ] **Phase 8.5: Implement the Motion Rig**
  - [ ] Implement the motion rig.
  - [ ] Replicate real time motion rig experiments in simulation.
- [ ] **Phase 9: GVS**
  - [ ] Implement the effect of GVS on the SCC afferents.
  - [ ] Real world experiments and musculoskeletal model personalization per subject. 
- [ ] **Phase 10: Spiking Neural Network (NEST-GPU)**
  - [ ] Turn every control theoretical process into a neural one.
  - [ ] Direct coupling with 1.1M LIF-based AdEx spiking neurons on RTX 4070 Ti.
- [ ] **Phase 11.1: Binaural Spatial Audio & Cochlea**
  - [ ] Convolve the offline calculated Head-Related Transfer Functions (HRTFs) and Room Impulse Responses (RIRs) in iGPU.
  - [ ] Simulate the Gammatone excitation pattern changes as the head rotates.
- [ ] **Phase 11.2: Spiking Neural Network (NEST-GPU)**
  - [ ] Direct coupling with 1.1M LIF-based AdEx spiking neurons on RTX 4070 Ti.
  - [ ] Direct coupling with 1.1M conductivity-based AdEx spiking neurons on RTX 4070 Ti.
  - [ ] Naive discovery of the structure of the 3D space studies. 

---

## 📄 License & Attribution

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**. See the full [LICENSE](LICENSE) file for complete terms.

---

## 📄 Key Literature References

* **[Seated Perturbation Benchmark & Biproper Otolith Realization]** — **Happee, R., de Bruijn, E., Forbes, P. A., & van der Helm, F. C. T. (2017).** *Dynamic head-neck stabilization and modulation with perturbation bandwidth investigated using a multisegment neuromuscular model.* Journal of Biomechanics, 58, 203–211.
* **[Semicircular Canal & Regular/Irregular Otolith Afferent Transfer Functions]** — **Schneider, A. D., Jamali, M., Carriot, J., Chacron, M. J., & Cullen, K. E. (2015).** *The increased sensitivity of irregular peripheral canal and otolith vestibular afferents optimizes their encoding of natural stimuli.* Journal of Neuroscience, 35(14), 5522–5536.
* **[Static Tilt Otolith Baseline Sensitivity ($k_0 = 37.4\text{ spk/s/g}$)]** — **Fernández, C., Goldberg, J. M., & Abend, W. K. (1972).** *Response to static tilts of peripheral neurons innervating otolith organs of the squirrel monkey.* Journal of Neurophysiology, 35(6), 978–997.
* **[3D Non-Orthogonal Vestibular Labyrinth Orientation Matrix ($\mathbf{B}$)]** — **Zheng, X., et al. (2020).** *A novel neuromuscular head-neck model and its application on impact analysis.*
* **[Intrafusal Muscle Spindle Mechanics ($\text{Bag}_1, \text{Bag}_2, \text{Chain}$) & $\text{Ia}/\text{II}$ Transduction]** — **Mileusnic, M. P., Brown, I. E., Lan, N., & Loeb, G. E. (2006).** *Mathematical models of proprioceptors. I. Control and dynamic response of muscle spindles.* Journal of Neurophysiology, 96(4), 1772–1792.
* **[Supraspinal-Only $\alpha\text{–}\gamma$ Coactivation & Absence of Monosynaptic $\text{Ia} \to \gamma$ Feedback]** — **Hulliger, M. (1984).** *The mammalian muscle spindle and its central control.* Reviews of Physiology, Biochemistry and Pharmacology, 101, 1–110; **Burke, R. E., Walmsley, B., & Hodgson, J. A. (1979).** *HRP anatomy of group Ia afferent projections to motoneuron pools in the cat spinal cord.* Brain Research, 160(2), 347–352.
* **[Hill-Type Equilibrium Muscle Mechanics (`osimMillard12EqWithAff`)]** — **Millard, M., Uchida, T., Seth, A., & Delp, S. L. (2013).** *Flexing computational muscle: modeling and simulation of musculotendon dynamics.* ASME Journal of Biomechanical Engineering, 135(2), 021005.
* **[3D Multisensory Quaternion Attitude Observer on $\text{SO}(3)$]** — **Mahony, R., Hamel, T., & Pflimlin, J.-M. (2008).** *Nonlinear complementary filters on the special orthogonal group.* IEEE Transactions on Automatic Control, 53(5), 1203–1218.
* **[VCR & CCR Lead-Lag Reflex Topology & Brainstem/Spinal Latencies]** — **Peng, G. C. Y., Hain, T. C., & Peterson, B. W. (1996).** *A dynamical model for reflex activated head movements in the horizontal plane.* Biological Cybernetics, 75(4), 309–319.
* **[Cerebellar Forward Internal Model & Vestibular Reafference Cancellation]** — **Cullen, K. E. (2012).** *The vestibular system: multimodal integration and encoding of self-motion for motor control.* Trends in Neurosciences, 35(3), 185–196; **Roy, J. E., & Cullen, K. E. (2004).** *Dissociating self-generated from passively applied head motion: neural mechanisms in the vestibular nuclei.* Journal of Neuroscience, 24(9), 2102–2111; **Brooks, J. X., & Cullen, K. E. (2013, 2014).** *The primate cerebellum selectively encodes unexpected self-motion / Learning to expect the unexpected.* Current Biology, 23(11), 947–955 & 24(4), 380–389.
* **[Cervical Spine Kinematics, Moment Arms & Paradigm A ROM Limits]** — **Vasavada, A. N., Li, S., & Delp, S. L. (1998).** *Influence of muscle morphometry and moment arms on moment-generating capacity of human neck muscles.* Spine, 23(4), 412–422.
* **[72-Muscle Detailed Hyoid & Cervical Musculoskeletal Architecture]** — **Mortensen, J. D., Vasavada, A. N., & Merryweather, A. S. (2018).** *The inclusion of hyoid muscles improves moment generating capacity and dynamic simulations in musculoskeletal models of the head and neck.* PLOS ONE, 13(6), e0199912.
* **[Vestibular Semicircular Canal & Otolith Perceptual/Reflex Thresholds ($\omega_{\text{min,scc}}, a_{\text{min,oto}}, \theta_{\text{min,tilt}}$)]** — **Benson, A. J., Hutt, E. C., & Brown, S. F. (1989).** *Thresholds for the perception of whole body angular movement about a vertical axis.* Aviation, Space, and Environmental Medicine, 60(3), 205–213; **Benson, A. J., Kass, J. R., & Vogel, H. (1986).** *European vestibular experiments on the Spacelab-1 mission: 4. Thresholds of perception of whole-body linear oscillation.* Experimental Brain Research, 64(2), 264–271; **Gianna, C., Heimbrand, S., & Gresty, M. (1996).** *Thresholds for detection of motion direction during linear accelerations.* Experimental Brain Research, 109(3), 499–505; **Grabherr, L., Nicoucar, K., Mast, F. W., & Merfeld, D. M. (2008).** *Vestibular thresholds for yaw rotation about an earth-vertical axis as a function of frequency.* Experimental Brain Research, 186(4), 677–681; **Valko, Y., Lewis, R. F., Priesol, A. J., & Merfeld, D. M. (2012).** *Vestibular labyrinth contributions to human whole-body motion discrimination.* Journal of Neuroscience, 32(39), 13537–13542.
* **[Muscle Spindle & Spinal Ia Synaptic Recruitment Thresholds ($\Delta \text{Ia}_{\text{min}}$) & Small-Signal Clonus Suppression]** — **Matthews, P. B. C. (1972).** *Mammalian Muscle Receptors and Their Central Actions.* London: Edward Arnold; **Burke, D., Gandevia, S. C., & McKeon, B. (1983).** *The afferent volleys responsible for spinal proprioceptive reflexes in man.* Journal of Physiology, 339(1), 535–552; **Hasan, Z. (1983).** *A model of spindle afferent response to muscle stretch.* Journal of Neurophysiology, 49(4), 989–1006.
