"""
Unified Volition-Uncertainty Neuromotor Controller (VUNC)
Implementation for OpenSim 4.6 24-Muscle Head-Neck Model

Unifies voluntary trajectory tracking with sensory-prediction-error-driven
reflex modulation (Dynamic Gamma drive & Vestibular Semicircular Canal Damping).
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

@dataclass
class VUNCConfig:
    # --- Structural Dimensions ---
    num_muscles: int = 24
    num_dof: int = 3  # pitch, roll, yaw
    
    # --- Delays (seconds) ---
    delay_vestibular: float = 0.015   # 15 ms VCR delay
    delay_spindle: float = 0.020      # 20 ms CCR delay
    delay_cortical: float = 0.040     # 40 ms voluntary loop delay
    
    # --- Volition Channel Parameters ---
    tau_voluntary: float = 0.150      # Trajectory smoothing filter time constant (s)
    k_stat: float = 0.1513            # Alpha-gamma coactivation gain (Gen 5 optimal)
    gamma_s_base: float = 0.05        # Baseline static gamma drive
    
    # --- Uncertainty Channel Parameters ---
    tau_uncertainty: float = 0.200    # Leaky integration filter time constant (s)
    beta_uncertainty: float = 5.0     # Normalization scale for Mahalanobis error
    sigma_omega: float = 0.05         # Canal error variance scale (rad/s)
    sigma_acc: float = 0.50           # Otolith error variance scale (m/s^2)
    sigma_spindle_pos: float = 0.002  # Spindle length error variance (m)
    sigma_spindle_vel: float = 0.020  # Spindle velocity error variance (m/s)
    
    # --- Fusimotor Set Modulation ---
    gamma_d_min: float = 0.10         # Minimum dynamic gamma (quiescent / predictable)
    gamma_d_max: float = 0.85         # Maximum dynamic gamma (high uncertainty / disturbance)
    
    # --- Vestibulo-Collic Reflex (VCR) Parameters (Gen 5 Optima) ---
    K_sc_base: float = 2.00           # Base semicircular canal rate damping (Nm/(rad/s))
    delta_K_sc: float = 4.05          # Additional damping under uncertainty (peaks at 6.05)
    z_sc: float = 7.73                # Canal lead-lag zero (rad/s)
    p_sc: float = 44.44               # Canal lead-lag pole (rad/s)
    
    K_ton: float = 2.42               # Tonic otolith gain (Nm/rad)
    p_ton: float = 44.03              # Tonic otolith pole (rad/s)
    r_ton: float = 0.400              # Tonic otolith high-pass ratio
    
    # --- Cervico-Collic Reflex (CCR) Parameters ---
    kp_ccr: float = 0.785             # Proportional spindle feedback gain
    z_ccr: float = 0.160              # CCR filter zero (rad/s)
    p_ccr: float = 12.28              # CCR filter pole (rad/s)
    k_dyn_base: float = 0.1442        # Baseline velocity sensitivity exponent gain
    
    # --- Task Space Equilibrium & Gravity Compensation ---
    Ki_gravity_pitch: float = 40.2786 # Calibrated gravity trim integrator gain
    nominal_trim_pitch_Nm: float = 4.24 # Calibrated upright balance torque
    
    # --- Task Priority & Voluntary Compliance Shedding ---
    w_task_priority: float = 0.0      # 0.0 = relaxed passenger (full shedding), 1.0 = rigid gaze lock
    compliance_shedding_fraction: float = 0.90 # Maximum compliance shedding fraction under disturbance
    
    # --- Muscle Activation Bounds ---
    min_activation: float = 0.025
    max_activation: float = 1.000


class UnifiedVolitionUncertaintyController:
    """
    Neuromotor controller coupling planned voluntary trajectories with
    stochastic reflex modulation under uncertainty.
    """
    
    def __init__(self, config: Optional[VUNCConfig] = None):
        self.cfg = config or VUNCConfig()
        
        # State variables
        self.uncertainty: float = 0.0
        self.gravity_integrator: float = self.cfg.nominal_trim_pitch_Nm
        self.prev_time: Optional[float] = None
        
        # Internal lead-lag filter states
        self.x_sc = np.zeros(self.cfg.num_dof)
        self.x_ton = np.zeros(self.cfg.num_dof)
        self.x_ccr = np.zeros(self.cfg.num_muscles)
        
        # Ring buffers for delayed sensor history
        self.history_time: List[float] = []
        self.history_predicted: List[Dict[str, np.ndarray]] = []
        
        # 24-Muscle Directional Moment Arm Matrix (3 DOF x 24 muscles)
        self._init_moment_arms()
        
    def _init_moment_arms(self):
        np.random.seed(42)
        J = np.zeros((3, self.cfg.num_muscles))
        # Dorsal extensors (muscles 0..11)
        J[0, 0:12] = np.random.uniform(0.020, 0.045, size=12)
        # Ventral flexors (muscles 12..23)
        J[0, 12:24] = np.random.uniform(-0.040, -0.015, size=12)
        # Lateral flexors & rotators
        J[1, :] = np.random.uniform(-0.030, 0.030, size=24)
        J[2, :] = np.random.uniform(-0.025, 0.025, size=24)
        self.J_moment = J
        self.F_max = np.full(self.cfg.num_muscles, 180.0)

    def forward_model_prediction(self, q_des: np.ndarray, dq_des: np.ndarray, ddq_des: np.ndarray) -> Dict[str, np.ndarray]:
        omega_pred = dq_des.copy()
        acc_pred = np.array([
            -9.81 * np.sin(q_des[0]),
             9.81 * np.sin(q_des[1]),
             9.81 * np.cos(q_des[0]) * np.cos(q_des[1])
        ])
        l_pred = - self.J_moment.T @ q_des
        v_pred = - self.J_moment.T @ dq_des
        return {
            'omega': omega_pred,
            'acc': acc_pred,
            'l_m': l_pred,
            'v_m': v_pred
        }

    def compute_sensory_innovation(self, 
                                  t: float, 
                                  actual_sensors: Dict[str, np.ndarray]) -> Tuple[float, Dict[str, np.ndarray]]:
        target_t = t - self.cfg.delay_vestibular
        if not self.history_time or target_t < self.history_time[0]:
            return 0.0, {
                'e_omega': np.zeros(self.cfg.num_dof),
                'e_acc': np.zeros(self.cfg.num_dof),
                'e_lm': np.zeros(self.cfg.num_muscles),
                'e_vm': np.zeros(self.cfg.num_muscles)
            }
        
        idx = np.searchsorted(self.history_time, target_t)
        idx = min(idx, len(self.history_time) - 1)
        pred = self.history_predicted[idx]
        
        e_omega = actual_sensors['omega'] - pred['omega']
        e_acc = actual_sensors['acc'] - pred['acc']
        e_lm = actual_sensors['l_m'] - pred['l_m']
        e_vm = actual_sensors['v_m'] - pred['v_m']
        
        norm_omega = np.sum((e_omega / self.cfg.sigma_omega) ** 2)
        norm_acc = np.sum((e_acc / self.cfg.sigma_acc) ** 2)
        norm_spindle = np.sum((e_lm / self.cfg.sigma_spindle_pos) ** 2) + np.sum((e_vm / self.cfg.sigma_spindle_vel) ** 2)
        
        total_discrepancy = norm_omega + norm_acc + 0.1 * norm_spindle
        return total_discrepancy, {'e_omega': e_omega, 'e_acc': e_acc, 'e_lm': e_lm, 'e_vm': e_vm}

    def update_uncertainty(self, dt: float, discrepancy: float):
        drive = np.tanh(discrepancy / self.cfg.beta_uncertainty)
        alpha = dt / (self.cfg.tau_uncertainty + dt)
        self.uncertainty = (1.0 - alpha) * self.uncertainty + alpha * drive
        self.uncertainty = float(np.clip(self.uncertainty, 0.0, 1.0))

    def schedule_gains(self) -> Tuple[float, float, float]:
        gamma_d = self.cfg.gamma_d_min + (self.cfg.gamma_d_max - self.cfg.gamma_d_min) * self.uncertainty
        K_sc = self.cfg.K_sc_base + self.cfg.delta_K_sc * self.uncertainty
        w_task = np.clip(self.cfg.w_task_priority, 0.0, 1.0)
        shed = (1.0 - w_task) * self.cfg.compliance_shedding_fraction * self.uncertainty
        vol_compliance = 1.0 - shed
        return gamma_d, K_sc, vol_compliance

    def solve_muscle_activations(self, tau_net: np.ndarray, alpha_vol: np.ndarray) -> np.ndarray:
        A = self.J_moment * self.F_max
        lambda_reg = 1e-4
        inv_term = np.linalg.pinv(A @ A.T + lambda_reg * np.eye(3))
        delta_tau = tau_net - A @ alpha_vol
        delta_a = A.T @ inv_term @ delta_tau
        a_net = alpha_vol + delta_a
        return np.clip(a_net, self.cfg.min_activation, self.cfg.max_activation)

    def step(self, 
             t: float, 
             q_actual: np.ndarray, 
             dq_actual: np.ndarray,
             actual_sensors: Dict[str, np.ndarray],
             vol_plan: Dict[str, np.ndarray]) -> Dict[str, any]:
        dt = 0.001 if self.prev_time is None else max(1e-4, t - self.prev_time)
        self.prev_time = t
        
        pred = self.forward_model_prediction(vol_plan['q_des'], vol_plan['dq_des'], vol_plan['ddq_des'])
        self.history_time.append(t)
        self.history_predicted.append(pred)
        
        while len(self.history_time) > 1 and self.history_time[0] < t - 0.250:
            self.history_time.pop(0)
            self.history_predicted.pop(0)
            
        discrepancy, errors = self.compute_sensory_innovation(t, actual_sensors)
        self.update_uncertainty(dt, discrepancy)
        gamma_d, K_sc, vol_compliance = self.schedule_gains()
        
        tau_vol_feedforward = vol_plan.get('tau_ff', np.zeros(3))
        alpha_vol = np.clip(np.linalg.pinv(self.J_moment * self.F_max) @ tau_vol_feedforward, 0.05, 0.5)
        gamma_s = self.cfg.gamma_s_base + self.cfg.k_stat * alpha_vol
        
        e_omega = errors['e_omega']
        dx_sc = -self.cfg.p_sc * self.x_sc + e_omega
        self.x_sc += dx_sc * dt
        tau_vcr = - K_sc * ((self.cfg.z_sc - self.cfg.p_sc) * self.x_sc + e_omega)
        
        spindle_Ia = errors['e_lm'] + self.cfg.k_dyn_base * gamma_d * np.sign(errors['e_vm']) * (np.abs(errors['e_vm']) ** 0.6)
        tau_ccr = self.J_moment @ (self.cfg.kp_ccr * self.F_max * spindle_Ia)
        
        self.gravity_integrator += self.cfg.Ki_gravity_pitch * (0.045 - q_actual[0]) * dt
        self.gravity_integrator = float(np.clip(self.gravity_integrator, 0.0, 15.0))
        tau_gravity = np.array([self.gravity_integrator, 0.0, 0.0])
        
        tau_vol = vol_compliance * tau_vol_feedforward
        tau_net = tau_vol + tau_vcr + tau_ccr + tau_gravity
        activations = self.solve_muscle_activations(tau_net, alpha_vol)
        
        return {
            'muscle_activations': activations,
            'tau_net': tau_net,
            'tau_vol': tau_vol,
            'tau_vcr': tau_vcr,
            'tau_ccr': tau_ccr,
            'tau_gravity': tau_gravity,
            'uncertainty': self.uncertainty,
            'gamma_d': gamma_d,
            'gamma_s_mean': float(np.mean(gamma_s)),
            'K_sc': K_sc,
            'vol_compliance': vol_compliance
        }

if __name__ == '__main__':
    print("Testing Unified Volition-Uncertainty Neuromotor Controller...")
    ctrl = UnifiedVolitionUncertaintyController()
    
    # Simulate 100 ms of voluntary head rotation without perturbation (U -> 0)
    print("\n--- Test 1: Voluntary Head Rotation without Disturbance ---")
    for step in range(50):
        t = step * 0.002
        q_des = np.array([0.05 * np.sin(2 * np.pi * 1.0 * t), 0.0, 0.0])
        dq_des = np.array([0.05 * 2 * np.pi * np.cos(2 * np.pi * 1.0 * t), 0.0, 0.0])
        ddq_des = np.array([-0.05 * (2 * np.pi)**2 * np.sin(2 * np.pi * 1.0 * t), 0.0, 0.0])
        
        vol_plan = {'q_des': q_des, 'dq_des': dq_des, 'ddq_des': ddq_des, 'tau_ff': np.array([1.5, 0.0, 0.0])}
        # Actual sensor matches predicted perfectly
        sensors = {
            'omega': dq_des,
            'acc': np.array([-9.81 * np.sin(q_des[0]), 0.0, 9.81 * np.cos(q_des[0])]),
            'l_m': - ctrl.J_moment.T @ q_des,
            'v_m': - ctrl.J_moment.T @ dq_des
        }
        res = ctrl.step(t, q_des, dq_des, sensors, vol_plan)
        
    print(f"Time {t:.3f}s: Uncertainty = {res['uncertainty']:.4f} | gamma_d = {res['gamma_d']:.3f} | K_sc = {res['K_sc']:.2f} Nm/(rad/s) | Vol Compliance = {res['vol_compliance']:.2f}")

    # Introduce sudden external sled perturbation (sensory prediction error spikes)
    print("\n--- Test 2: External Perturbation Applied (Sled Acceleration) ---")
    for step in range(50, 150):
        t = step * 0.002
        # Plan expects stationary head
        vol_plan = {'q_des': np.zeros(3), 'dq_des': np.zeros(3), 'ddq_des': np.zeros(3), 'tau_ff': np.zeros(3)}
        # Actual head is violently shaken by sled disturbance
        sensors = {
            'omega': np.array([0.8 * np.cos(2 * np.pi * 5.0 * t), 0.0, 0.0]),
            'acc': np.array([1.5 * np.sin(2 * np.pi * 5.0 * t), 0.0, 9.81]),
            'l_m': np.random.normal(0.0, 0.005, size=24),
            'v_m': np.random.normal(0.0, 0.05, size=24)
        }
        res = ctrl.step(t, np.zeros(3), np.zeros(3), sensors, vol_plan)
        
    print(f"Time {t:.3f}s: Uncertainty = {res['uncertainty']:.4f} | gamma_d = {res['gamma_d']:.3f} | K_sc = {res['K_sc']:.2f} Nm/(rad/s) | Vol Compliance = {res['vol_compliance']:.2f}")
    print("VUNC module test completed successfully!")
