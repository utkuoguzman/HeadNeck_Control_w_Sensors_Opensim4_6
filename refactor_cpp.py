import os
import re

header_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.h"
cpp_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp"

with open(header_path, "r") as f:
    h_code = f.read()

# Replace properties in header
h_code = h_code.replace('OpenSim_DECLARE_PROPERTY(G_sc, double, "Vestibular semicircular canal gain");\n    OpenSim_DECLARE_PROPERTY(G_ton, double, "Tonic otolith gain");\n    OpenSim_DECLARE_PROPERTY(G_phas, double, "Phasic otolith gain");',
'''OpenSim_DECLARE_PROPERTY(K_ton, double, "Tonic otolith TF Gain");
    OpenSim_DECLARE_PROPERTY(z_ton, double, "Tonic otolith TF Zero");
    OpenSim_DECLARE_PROPERTY(p_ton, double, "Tonic otolith TF Pole");
    OpenSim_DECLARE_PROPERTY(K_sc, double, "Semicircular canal TF Gain");
    OpenSim_DECLARE_PROPERTY(z_sc, double, "Semicircular canal TF Zero");
    OpenSim_DECLARE_PROPERTY(p_sc, double, "Semicircular canal TF Pole");
    OpenSim_DECLARE_PROPERTY(K_phas, double, "Phasic otolith TF Gain");
    OpenSim_DECLARE_PROPERTY(z_phas, double, "Phasic otolith TF Zero");
    OpenSim_DECLARE_PROPERTY(p_phas, double, "Phasic otolith TF Pole");''')

with open(header_path, "w") as f:
    f.write(h_code)

with open(cpp_path, "r") as f:
    cpp_code = f.read()

# Replace constructProperty
cpp_code = cpp_code.replace('constructProperty_G_sc(1.0);\n    constructProperty_G_ton(0.01);\n    constructProperty_G_phas(0.01);',
'''constructProperty_K_ton(52.35);
    constructProperty_z_ton(0.01);
    constructProperty_p_ton(8.03);
    constructProperty_K_sc(18.09);
    constructProperty_z_sc(0.01);
    constructProperty_p_sc(3.93);
    constructProperty_K_phas(8.68);
    constructProperty_z_phas(17.17);
    constructProperty_p_phas(28.72);''')

# Add State Variables in extendAddToSystem
# Look for: addStateVariable("yaw_error_integral");
state_vars_str = '''addStateVariable("yaw_error_integral");
    
    // Neural Filter States (Pitch, Roll, Yaw)
    addStateVariable("ton_state_pitch");
    addStateVariable("ton_state_roll");
    addStateVariable("ton_state_yaw");
    addStateVariable("sc_state_pitch");
    addStateVariable("sc_state_roll");
    addStateVariable("sc_state_yaw");
    addStateVariable("phas_state_pitch");
    addStateVariable("phas_state_roll");
    addStateVariable("phas_state_yaw");'''
cpp_code = cpp_code.replace('addStateVariable("yaw_error_integral");', state_vars_str)

# Now inject the State Variable Derivatives logic at the end of computeStateVariableDerivatives
injection_point = "setStateVariableDerivativeValue(s, \"yaw_error_integral\", yaw_err);"
deriv_injection = '''setStateVariableDerivativeValue(s, "yaw_error_integral", yaw_err);

    // --- Neural Filter State Derivatives ---
    DelayedData vcr_data = getDelayedData(s.getTime(), get_delay_time());
    HistRecord vcr_record = vcr_data.record;
    
    if (vcr_record.omega_recon.size() == 3) {
        // We need to re-extract estimated angles from delayed quaternion
        double q0 = vcr_record.theta[0]; // wait, theta stores roll, yaw, pitch? Let's check: 
        // In computeControls, hr.theta[0] = head_roll; hr.theta[1] = head_yaw; hr.theta[2] = head_pitch;
        double est_roll = vcr_record.theta[0];
        double est_yaw = vcr_record.theta[1];
        double est_pitch = vcr_record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - get_desired_pitch();
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - get_desired_roll();
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - get_desired_yaw();
        
        double u_sc_pitch = vcr_record.omega_recon[2] - get_desired_pitch_v();
        double u_sc_roll  = vcr_record.omega_recon[0] - get_desired_roll_v();
        double u_sc_yaw   = vcr_record.omega_recon[1] - get_desired_yaw_v();
        
        // Phasic specific force
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); 
        // Need delayed estimated rotation for biological acceleration. 
        // But since we didn't save the quaternion in history, we can approximate with just the head angles.
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = get_desired_pitch_a() * r_head; 
        double expected_ah_z = get_desired_roll_a() * r_head;
        
        double u_phas_pitch = a_lin_head[0] - expected_ah_x;
        double u_phas_roll  = a_lin_head[2] - expected_ah_z;
        double u_phas_yaw   = 0.0; // Not used for yaw
        
        double x_ton_pitch = getStateVariableValue(s, "ton_state_pitch");
        double x_ton_roll  = getStateVariableValue(s, "ton_state_roll");
        double x_ton_yaw   = getStateVariableValue(s, "ton_state_yaw");
        
        double x_sc_pitch = getStateVariableValue(s, "sc_state_pitch");
        double x_sc_roll  = getStateVariableValue(s, "sc_state_roll");
        double x_sc_yaw   = getStateVariableValue(s, "sc_state_yaw");
        
        double x_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
        double x_phas_roll  = getStateVariableValue(s, "phas_state_roll");
        double x_phas_yaw   = getStateVariableValue(s, "phas_state_yaw");
        
        setStateVariableDerivativeValue(s, "ton_state_pitch", -get_p_ton() * x_ton_pitch + u_ton_pitch);
        setStateVariableDerivativeValue(s, "ton_state_roll", -get_p_ton() * x_ton_roll + u_ton_roll);
        setStateVariableDerivativeValue(s, "ton_state_yaw", -get_p_ton() * x_ton_yaw + u_ton_yaw);
        
        setStateVariableDerivativeValue(s, "sc_state_pitch", -get_p_sc() * x_sc_pitch + u_sc_pitch);
        setStateVariableDerivativeValue(s, "sc_state_roll", -get_p_sc() * x_sc_roll + u_sc_roll);
        setStateVariableDerivativeValue(s, "sc_state_yaw", -get_p_sc() * x_sc_yaw + u_sc_yaw);
        
        setStateVariableDerivativeValue(s, "phas_state_pitch", -get_p_phas() * x_phas_pitch + u_phas_pitch);
        setStateVariableDerivativeValue(s, "phas_state_roll", -get_p_phas() * x_phas_roll + u_phas_roll);
        setStateVariableDerivativeValue(s, "phas_state_yaw", -get_p_phas() * x_phas_yaw + u_phas_yaw);
    } else {
        setStateVariableDerivativeValue(s, "ton_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "ton_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "ton_state_yaw", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_yaw", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_yaw", 0.0);
    }
'''
cpp_code = cpp_code.replace(injection_point, deriv_injection)


# Finally, replace the tau_vcr application in computeControls
old_tau_code = '''        // G_ton (Otolith Tonic) acts as a proportional gravity compensator
        // We subtract the Efference Copy of expected static tilt
        tau_vcr[0] -= get_G_ton() * ((estimated_pitch - baseline_spatial_orientation[2]) - get_desired_pitch()); // pitch
        tau_vcr[1] -= get_G_ton() * ((estimated_roll - baseline_spatial_orientation[0]) - get_desired_roll());  // roll
        tau_vcr[2] -= get_G_ton() * ((estimated_yaw - baseline_spatial_orientation[1]) - get_desired_yaw());   // yaw
        
        // G_sc (Semicircular Canal) dampens angular velocity
        // We subtract the Efference Copy of expected angular velocity
        tau_vcr[0] -= get_G_sc() * (vcr_record.omega_recon[2] - get_desired_pitch_v()); // pitch
        tau_vcr[1] -= get_G_sc() * (vcr_record.omega_recon[0] - get_desired_roll_v()); // roll
        tau_vcr[2] -= get_G_sc() * (vcr_record.omega_recon[1] - get_desired_yaw_v()); // yaw
        
        // G_phas (Otolith Phasic) dampens linear acceleration
        // BIOLOGICAL REALISM: The controller now derives pure acceleration from the otoliths
        // using the SE_2(3) spatial observer. It no longer relies on ground-truth physics!
        
        // Simulate raw otolith specific force
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); 
        
        // Derive pure biological acceleration using our internal orientation belief
        SimTK::Rotation R_est(SimTK::Quaternion(q0, q1, q2, q3)); 
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world); // R_est is World->Head, so R_est * g_world is g in head frame
        
        double ah_x = a_lin_head[0];
        double ah_y = a_lin_head[1];
        double ah_z = a_lin_head[2];
        
        // Estimate expected linear acceleration caused by desired angular acceleration
        // Approximating head radius (e.g. 0.15m from neck pivot to otoliths)
        double r_head = 0.15; 
        double expected_ah_x = get_desired_pitch_a() * r_head; 
        double expected_ah_z = get_desired_roll_a() * r_head;
        
        // Pitch responds to X (Forward), Roll responds to Z (Lateral)
        tau_vcr[0] -= get_G_phas() * (ah_x - expected_ah_x); 
        tau_vcr[1] -= get_G_phas() * (ah_z - expected_ah_z); '''

new_tau_code = '''        // Neural Transfer Functions Output Application
        // y = K*(z - p)*x + K*u
        
        double est_roll = vcr_record.theta[0];
        double est_yaw = vcr_record.theta[1];
        double est_pitch = vcr_record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - get_desired_pitch();
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - get_desired_roll();
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - get_desired_yaw();
        
        double u_sc_pitch = vcr_record.omega_recon[2] - get_desired_pitch_v();
        double u_sc_roll  = vcr_record.omega_recon[0] - get_desired_roll_v();
        double u_sc_yaw   = vcr_record.omega_recon[1] - get_desired_yaw_v();
        
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); 
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = get_desired_pitch_a() * r_head; 
        double expected_ah_z = get_desired_roll_a() * r_head;
        
        double u_phas_pitch = a_lin_head[0] - expected_ah_x;
        double u_phas_roll  = a_lin_head[2] - expected_ah_z;
        double u_phas_yaw   = 0.0;
        
        double x_ton_pitch = getStateVariableValue(s, "ton_state_pitch");
        double x_ton_roll  = getStateVariableValue(s, "ton_state_roll");
        double x_ton_yaw   = getStateVariableValue(s, "ton_state_yaw");
        
        double x_sc_pitch = getStateVariableValue(s, "sc_state_pitch");
        double x_sc_roll  = getStateVariableValue(s, "sc_state_roll");
        double x_sc_yaw   = getStateVariableValue(s, "sc_state_yaw");
        
        double x_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
        double x_phas_roll  = getStateVariableValue(s, "phas_state_roll");
        double x_phas_yaw   = getStateVariableValue(s, "phas_state_yaw");
        
        double y_ton_pitch = get_K_ton() * (get_z_ton() - get_p_ton()) * x_ton_pitch + get_K_ton() * u_ton_pitch;
        double y_ton_roll  = get_K_ton() * (get_z_ton() - get_p_ton()) * x_ton_roll  + get_K_ton() * u_ton_roll;
        double y_ton_yaw   = get_K_ton() * (get_z_ton() - get_p_ton()) * x_ton_yaw   + get_K_ton() * u_ton_yaw;
        
        double y_sc_pitch = get_K_sc() * (get_z_sc() - get_p_sc()) * x_sc_pitch + get_K_sc() * u_sc_pitch;
        double y_sc_roll  = get_K_sc() * (get_z_sc() - get_p_sc()) * x_sc_roll  + get_K_sc() * u_sc_roll;
        double y_sc_yaw   = get_K_sc() * (get_z_sc() - get_p_sc()) * x_sc_yaw   + get_K_sc() * u_sc_yaw;
        
        double y_phas_pitch = get_K_phas() * (get_z_phas() - get_p_phas()) * x_phas_pitch + get_K_phas() * u_phas_pitch;
        double y_phas_roll  = get_K_phas() * (get_z_phas() - get_p_phas()) * x_phas_roll  + get_K_phas() * u_phas_roll;
        double y_phas_yaw   = get_K_phas() * (get_z_phas() - get_p_phas()) * x_phas_yaw   + get_K_phas() * u_phas_yaw;
        
        tau_vcr[0] -= y_ton_pitch + y_sc_pitch + y_phas_pitch;
        tau_vcr[1] -= y_ton_roll  + y_sc_roll  + y_phas_roll;
        tau_vcr[2] -= y_ton_yaw   + y_sc_yaw   + y_phas_yaw;'''

cpp_code = cpp_code.replace(old_tau_code, new_tau_code)

with open(cpp_path, "w") as f:
    f.write(cpp_code)

print("C++ code refactored to use Transfer Functions!")
