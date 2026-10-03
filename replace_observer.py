import os
import re

path = r'HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp'
with open(path, 'r') as f:
    content = f.read()

# 1. State Variables
old_states = """    // Mahony Filter State Variables
    addStateVariable("mahony_q0", Stage::Dynamics);
    addStateVariable("mahony_q1", Stage::Dynamics);
    addStateVariable("mahony_q2", Stage::Dynamics);
    addStateVariable("mahony_q3", Stage::Dynamics);"""
new_states = """    // SE_2(3) Spatial Observer States
    addStateVariable("obs_q0", Stage::Dynamics);
    addStateVariable("obs_q1", Stage::Dynamics);
    addStateVariable("obs_q2", Stage::Dynamics);
    addStateVariable("obs_q3", Stage::Dynamics);
    addStateVariable("obs_vx", Stage::Dynamics);
    addStateVariable("obs_vy", Stage::Dynamics);
    addStateVariable("obs_vz", Stage::Dynamics);
    addStateVariable("obs_px", Stage::Dynamics);
    addStateVariable("obs_py", Stage::Dynamics);
    addStateVariable("obs_pz", Stage::Dynamics);"""
content = content.replace(old_states, new_states)

# 2. Initial Values
old_init = """    setStateVariableValue(s, "mahony_q0", 1.0);
    setStateVariableValue(s, "mahony_q1", 0.0);
    setStateVariableValue(s, "mahony_q2", 0.0);
    setStateVariableValue(s, "mahony_q3", 0.0);"""
new_init = """    setStateVariableValue(s, "obs_q0", 1.0);
    setStateVariableValue(s, "obs_q1", 0.0);
    setStateVariableValue(s, "obs_q2", 0.0);
    setStateVariableValue(s, "obs_q3", 0.0);
    setStateVariableValue(s, "obs_vx", 0.0);
    setStateVariableValue(s, "obs_vy", 0.0);
    setStateVariableValue(s, "obs_vz", 0.0);
    setStateVariableValue(s, "obs_px", 0.0);
    setStateVariableValue(s, "obs_py", 0.0);
    setStateVariableValue(s, "obs_pz", 0.0);"""
content = content.replace(old_init, new_init)

# 3. Derivatives
pattern = r"// --- MAHONY FILTER \(Cerebellar Sensory Fusion\) ---.*?^}"
replacement = """// --- SE_2(3) SPATIAL OBSERVER (Tilt-Translation Disambiguation) ---
    SimTK::Vector omega_sense = vcr_data.record.omega_recon;
    
    if (omega_sense.size() == 3) {
        // 1. Simulate Otolith Attachment (Raw specific force: f = a_lin - g)
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); // Head frame
        
        // 2. Current Observer Beliefs
        double q0 = getStateVariableValue(s, "obs_q0");
        double q1 = getStateVariableValue(s, "obs_q1");
        double q2 = getStateVariableValue(s, "obs_q2");
        double q3 = getStateVariableValue(s, "obs_q3");
        
        double vx = getStateVariableValue(s, "obs_vx");
        double vy = getStateVariableValue(s, "obs_vy");
        double vz = getStateVariableValue(s, "obs_vz");
        SimTK::Vec3 v_est(vx, vy, vz);
        
        double px = getStateVariableValue(s, "obs_px");
        double py = getStateVariableValue(s, "obs_py");
        double pz = getStateVariableValue(s, "obs_pz");
        SimTK::Vec3 p_est(px, py, pz);
        
        // Normalize quaternion
        double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
        
        // 3. TILT CORRECTION (Mahony)
        // Internal Forward Model (Efference Copy) - Subtract expected acceleration from f_oto
        double K_eff_pitch = 1.33; // Pitch torque to X-axis acceleration
        SimTK::Vec3 a_expected(K_eff_pitch * getStateVariableValue(s, "eff_tau_pitch"), 0.0, 0.0);
        SimTK::Vec3 f_corrected = f_oto - a_expected;
        if (f_corrected.norm() > 1e-4) {
            f_corrected = f_corrected.normalize();
        }
        
        // Estimated UP direction (rotate [0, 1, 0] by q^-1)
        double expected_up_x = 2.0 * (q1*q2 + q0*q3);
        double expected_up_y = (q0*q0 - q1*q1 + q2*q2 - q3*q3);
        double expected_up_z = 2.0 * (q2*q3 - q0*q1);
        SimTK::Vec3 expected_up(expected_up_x, expected_up_y, expected_up_z);
        
        SimTK::Vec3 e = f_corrected % expected_up; // Cross product error
        
        // 4. ORIENTATION DERIVATIVE (PI Gyroscope Correction)
        double Kp_obs = 2.0;
        double gx = omega_sense[0] + Kp_obs * e[0];
        double gy = omega_sense[1] + Kp_obs * e[1];
        double gz = omega_sense[2] + Kp_obs * e[2];
        
        double dq0 = 0.5 * (-q1*gx - q2*gy - q3*gz);
        double dq1 = 0.5 * ( q0*gx + q2*gz - q3*gy);
        double dq2 = 0.5 * ( q0*gy - q1*gz + q3*gx);
        double dq3 = 0.5 * ( q0*gz + q1*gy - q2*gx);
        
        // 5. TRANSLATION DERIVATIVES
        // Reconstruct pure linear acceleration in WORLD frame: a_lin = R * f_oto + g_world
        // R matrix from q (World to Head):
        // Wait, q here represents Head to World or World to Head?
        // Standard Mahony q represents World to Sensor (Head).
        // Let's use SimTK::Rotation for safety.
        SimTK::Rotation R_est(SimTK::Quaternion(q0, q1, q2, q3)); 
        // Assuming R_est transforms from World to Head.
        // Then f_oto (Head frame) to world frame is ~R_est * f_oto.
        SimTK::Vec3 a_lin_world = (~R_est * f_oto) + g_world;
        
        // Biological Leaky Integrator (Velocity Storage & Egocentric tether)
        double tau_v = 15.0; // Decay in dark
        double tau_p = 5.0;
        
        SimTK::Vec3 dv = a_lin_world - (1.0 / tau_v) * v_est;
        SimTK::Vec3 dp = v_est - (1.0 / tau_p) * p_est;
        
        // Write to state derivatives
        setStateVariableDerivativeValue(s, "obs_q0", dq0);
        setStateVariableDerivativeValue(s, "obs_q1", dq1);
        setStateVariableDerivativeValue(s, "obs_q2", dq2);
        setStateVariableDerivativeValue(s, "obs_q3", dq3);
        
        setStateVariableDerivativeValue(s, "obs_vx", dv[0]);
        setStateVariableDerivativeValue(s, "obs_vy", dv[1]);
        setStateVariableDerivativeValue(s, "obs_vz", dv[2]);
        
        setStateVariableDerivativeValue(s, "obs_px", dp[0]);
        setStateVariableDerivativeValue(s, "obs_py", dp[1]);
        setStateVariableDerivativeValue(s, "obs_pz", dp[2]);
        
    } else {
        setStateVariableDerivativeValue(s, "obs_q0", 0.0);
        setStateVariableDerivativeValue(s, "obs_q1", 0.0);
        setStateVariableDerivativeValue(s, "obs_q2", 0.0);
        setStateVariableDerivativeValue(s, "obs_q3", 0.0);
        
        setStateVariableDerivativeValue(s, "obs_vx", 0.0);
        setStateVariableDerivativeValue(s, "obs_vy", 0.0);
        setStateVariableDerivativeValue(s, "obs_vz", 0.0);
        
        setStateVariableDerivativeValue(s, "obs_px", 0.0);
        setStateVariableDerivativeValue(s, "obs_py", 0.0);
        setStateVariableDerivativeValue(s, "obs_pz", 0.0);
    }"""
content = re.sub(pattern, replacement, content, flags=re.MULTILINE|re.DOTALL)

with open(path, 'w') as f:
    f.write(content)
print("Updated derivatives!")
