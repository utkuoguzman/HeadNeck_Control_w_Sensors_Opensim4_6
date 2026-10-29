import os

src_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp"
dst_path = "HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.cpp"

with open(src_path, "r", encoding="utf-8") as f:
    code = f.read()

# 1. Replace includes
code = code.replace('#include "HeadNeckNeuralController.h"', 
                    '#include "HeadNeckNeuralControllerAdaptive.h"\n#include "RegisterTypes_osimHeadNeckNeuralAdaptive.h"')

# 2. Add static reference pitch helper before constructor
helper = """
static double computeReferencePitchDeg(double t, double multisine_scale) {
    const double t_delay = 1.5;
    const double t_ramp = 1.0;
    if (t < t_delay) return 0.0;
    double t_rel = t - t_delay;
    double w = 1.0;
    if (t_rel < t_ramp) {
        double tau = t_rel / t_ramp;
        w = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
    }
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double targets_gain[6] = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double targets_phase_deg[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};
    
    double y_ref_deg = 0.0;
    for (int k = 0; k < 6; ++k) {
        double f = freqs[k];
        double phi_k = SimTK::Pi * k * (k - 1) / 6.0;
        double base_amp = 0.0215 * std::pow(0.835 / f, 2);
        double A_k = base_amp * multisine_scale;
        double phase_rad = phi_k + targets_phase_deg[k] * SimTK::Pi / 180.0;
        y_ref_deg += A_k * targets_gain[k] * std::sin(2.0 * SimTK::Pi * f * t_rel + phase_rad);
    }
    return w * y_ref_deg;
}
"""

# Replace class name
code = code.replace("HeadNeckNeuralController::", "HeadNeckNeuralControllerAdaptive::")
code = code.replace("HeadNeckNeuralController()", "HeadNeckNeuralControllerAdaptive()")

# In constructor, add properties
target_prop = "constructProperty_fc_voluntary(2.0);"
adaptive_props = """constructProperty_fc_voluntary(2.0);
    constructProperty_enable_adaptation(true);
    constructProperty_adaptation_rate_sc(0.15);
    constructProperty_adaptation_rate_ton(0.15);
    constructProperty_adaptation_rate_phas(0.40);
    constructProperty_adaptation_rate_ccr(0.20);
    constructProperty_sigma_leakage(0.02);
    constructProperty_adaptation_start_time(2.5);
    constructProperty_multisine_scale(0.142887);"""
code = code.replace(target_prop, adaptive_props)

# Insert helper before constructor
code = code.replace("HeadNeckNeuralControllerAdaptive::HeadNeckNeuralControllerAdaptive()",
                    helper + "\nHeadNeckNeuralControllerAdaptive::HeadNeckNeuralControllerAdaptive()")

# In extendAddToSystem, add adaptive state variables
target_add = """        addStateVariable("sp_fstat_" + mname, SimTK::Stage::Velocity);
    }
}"""
adaptive_add = """        addStateVariable("sp_fstat_" + mname, SimTK::Stage::Velocity);
    }

    // Continuous Adaptive Parameters & Filter States (Path 1)
    addStateVariable("adapt_K_sc", SimTK::Stage::Velocity);
    addStateVariable("adapt_K_ton", SimTK::Stage::Velocity);
    addStateVariable("adapt_K_phas", SimTK::Stage::Velocity);
    addStateVariable("adapt_kp_prop", SimTK::Stage::Velocity);
    for (int i = 0; i < 4; ++i) {
        addStateVariable("psif_x1_" + std::to_string(i), SimTK::Stage::Velocity);
        addStateVariable("psif_x2_" + std::to_string(i), SimTK::Stage::Velocity);
    }
}"""
code = code.replace(target_add, adaptive_add)

# In extendInitStateFromProperties, initialize adaptive states
target_init = """        setStateVariableValue(s, "sp_fstat_" + mname, 0.0);
    }
}"""
adaptive_init = """        setStateVariableValue(s, "sp_fstat_" + mname, 0.0);
    }

    setStateVariableValue(s, "adapt_K_sc", get_K_sc());
    setStateVariableValue(s, "adapt_K_ton", get_K_ton());
    setStateVariableValue(s, "adapt_K_phas", get_K_phas());
    setStateVariableValue(s, "adapt_kp_prop", get_Kp_proprioception());
    for (int i = 0; i < 4; ++i) {
        setStateVariableValue(s, "psif_x1_" + std::to_string(i), 0.0);
        setStateVariableValue(s, "psif_x2_" + std::to_string(i), 0.0);
    }
}"""
code = code.replace(target_init, adaptive_init)

# In initializeGeometryAndBaseline, bind sv_adapt_* pointers
target_bind = """    sv_phas[0] = traverseToStateVariable("phas_state_pitch");
    sv_phas[1] = traverseToStateVariable("phas_state_roll");
    sv_phas[2] = traverseToStateVariable("phas_state_yaw");"""
adaptive_bind = """    sv_phas[0] = traverseToStateVariable("phas_state_pitch");
    sv_phas[1] = traverseToStateVariable("phas_state_roll");
    sv_phas[2] = traverseToStateVariable("phas_state_yaw");

    sv_adapt_K_sc = traverseToStateVariable("adapt_K_sc");
    sv_adapt_K_ton = traverseToStateVariable("adapt_K_ton");
    sv_adapt_K_phas = traverseToStateVariable("adapt_K_phas");
    sv_adapt_kp_prop = traverseToStateVariable("adapt_kp_prop");
    for (int i = 0; i < 4; ++i) {
        sv_psif_x1[i] = traverseToStateVariable("psif_x1_" + std::to_string(i));
        sv_psif_x2[i] = traverseToStateVariable("psif_x2_" + std::to_string(i));
    }"""
code = code.replace(target_bind, adaptive_bind)

# Add Output Getters
target_getters = """double HeadNeckNeuralControllerAdaptive::getMahonyYawSpeed(const SimTK::State& s) const {
    return mahony_speed[2];
}"""
adaptive_getters = """double HeadNeckNeuralControllerAdaptive::getMahonyYawSpeed(const SimTK::State& s) const {
    return mahony_speed[2];
}

double HeadNeckNeuralControllerAdaptive::getAdaptKsc(const SimTK::State& s) const {
    return sv_adapt_K_sc ? sv_adapt_K_sc->getValue(s) : getStateVariableValue(s, "adapt_K_sc");
}

double HeadNeckNeuralControllerAdaptive::getAdaptKton(const SimTK::State& s) const {
    return sv_adapt_K_ton ? sv_adapt_K_ton->getValue(s) : getStateVariableValue(s, "adapt_K_ton");
}

double HeadNeckNeuralControllerAdaptive::getAdaptKphas(const SimTK::State& s) const {
    return sv_adapt_K_phas ? sv_adapt_K_phas->getValue(s) : getStateVariableValue(s, "adapt_K_phas");
}

double HeadNeckNeuralControllerAdaptive::getAdaptKpprop(const SimTK::State& s) const {
    return sv_adapt_kp_prop ? sv_adapt_kp_prop->getValue(s) : getStateVariableValue(s, "adapt_kp_prop");
}

double HeadNeckNeuralControllerAdaptive::getActualPitchDeg(const SimTK::State& s) const {
    double p1 = cached_coord_p1 ? cached_coord_p1->getValue(s) : 0.0;
    double p2 = cached_coord_p2 ? cached_coord_p2->getValue(s) : 0.0;
    return (p1 + p2) * 180.0 / SimTK::Pi;
}

double HeadNeckNeuralControllerAdaptive::getRefPitchDeg(const SimTK::State& s) const {
    return computeReferencePitchDeg(s.getTime(), get_multisine_scale());
}

double HeadNeckNeuralControllerAdaptive::getAdaptErrorDeg(const SimTK::State& s) const {
    return getActualPitchDeg(s) - getRefPitchDeg(s);
}"""
code = code.replace(target_getters, adaptive_getters)

# In computeControls, use effective gains
ctrl_target = """        double y_ton_pitch = get_K_ton() * get_p_ton() * x_ton_pitch;
        double y_ton_roll  = get_K_ton() * get_p_ton() * x_ton_roll;
        double y_ton_yaw   = get_K_ton() * get_p_ton() * x_ton_yaw;
        
        double p_sc = get_p_sc();
        double z_sc = get_z_sc();
        if (std::abs(z_sc) < 1e-4) z_sc = (z_sc >= 0.0 ? 1e-4 : -1e-4);
        double b_dir_sc = 1.0;
        double b_state_sc = z_sc - p_sc;

        double y_sc_pitch = get_K_sc() * (b_dir_sc * u_sc_pitch + b_state_sc * x_sc_pitch);
        double y_sc_roll  = get_K_sc() * (b_dir_sc * u_sc_roll  + b_state_sc * x_sc_roll);
        double y_sc_yaw   = get_K_sc() * (b_dir_sc * u_sc_yaw   + b_state_sc * x_sc_yaw);
        
        double y_phas_pitch = get_K_phas() * get_p_phas() * x_phas_pitch;
        double y_phas_roll  = get_K_phas() * get_p_phas() * x_phas_roll;
        double y_phas_yaw   = get_K_phas() * get_p_phas() * x_phas_yaw;"""

ctrl_adaptive = """        double eff_K_ton = (sv_adapt_K_ton && get_enable_adaptation()) ? sv_adapt_K_ton->getValue(s) : get_K_ton();
        double eff_K_sc  = (sv_adapt_K_sc && get_enable_adaptation())  ? sv_adapt_K_sc->getValue(s)  : get_K_sc();
        double eff_K_phas = (sv_adapt_K_phas && get_enable_adaptation()) ? sv_adapt_K_phas->getValue(s) : get_K_phas();

        double y_ton_pitch = eff_K_ton * get_p_ton() * x_ton_pitch;
        double y_ton_roll  = eff_K_ton * get_p_ton() * x_ton_roll;
        double y_ton_yaw   = eff_K_ton * get_p_ton() * x_ton_yaw;
        
        double p_sc = get_p_sc();
        double z_sc = get_z_sc();
        if (std::abs(z_sc) < 1e-4) z_sc = (z_sc >= 0.0 ? 1e-4 : -1e-4);
        double b_dir_sc = 1.0;
        double b_state_sc = z_sc - p_sc;

        double y_sc_pitch = eff_K_sc * (b_dir_sc * u_sc_pitch + b_state_sc * x_sc_pitch);
        double y_sc_roll  = eff_K_sc * (b_dir_sc * u_sc_roll  + b_state_sc * x_sc_roll);
        double y_sc_yaw   = eff_K_sc * (b_dir_sc * u_sc_yaw   + b_state_sc * x_sc_yaw);
        
        double y_phas_pitch = eff_K_phas * get_p_phas() * x_phas_pitch;
        double y_phas_roll  = eff_K_phas * get_p_phas() * x_phas_roll;
        double y_phas_yaw   = eff_K_phas * get_p_phas() * x_phas_yaw;"""
code = code.replace(ctrl_target, ctrl_adaptive)

# In computeControls spindle reflex:
spindle_target = "excitation += get_Kp_proprioception() * (Ia_firing / 100.0) * inhibition_factor;"
spindle_adaptive = """double eff_kp_prop = (sv_adapt_kp_prop && get_enable_adaptation()) ? sv_adapt_kp_prop->getValue(s) : get_Kp_proprioception();
            excitation += eff_kp_prop * (Ia_firing / 100.0) * inhibition_factor;"""
code = code.replace(spindle_target, spindle_adaptive)

# Capture u_ton_pitch and u_phas_pitch in member variables
code = code.replace("double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;",
                    "double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;\n        last_u_ton_pitch = u_ton_pitch;")

code = code.replace("double u_phas_pitch = a_lin_head[0] - expected_ah_x;",
                    "double u_phas_pitch = a_lin_head[0] - expected_ah_x;\n        last_u_phas_pitch = u_phas_pitch;")

# In computeStateVariableDerivatives, append continuous adaptation dynamics
end_deriv_target = """    if (ccr_record.Ia.size() == active_muscle_indices.size() && active_muscle_indices.size() > 0) {
        for (size_t i = 0; i < active_muscle_indices.size(); ++i) {
            double x_ccr = sv_ccr_state[i]->getValue(s);
            double Ia_delayed = ccr_record.Ia[i];
            sv_ccr_state[i]->setDerivative(s, -p_ccr * x_ccr + Ia_delayed);
        }
    } else {
        for (size_t i = 0; i < active_muscle_indices.size(); ++i) {
            sv_ccr_state[i]->setDerivative(s, 0.0);
        }
    }

}"""

end_deriv_adaptive = """    if (ccr_record.Ia.size() == active_muscle_indices.size() && active_muscle_indices.size() > 0) {
        for (size_t i = 0; i < active_muscle_indices.size(); ++i) {
            double x_ccr = sv_ccr_state[i]->getValue(s);
            double Ia_delayed = ccr_record.Ia[i];
            sv_ccr_state[i]->setDerivative(s, -p_ccr * x_ccr + Ia_delayed);
        }
    } else {
        for (size_t i = 0; i < active_muscle_indices.size(); ++i) {
            sv_ccr_state[i]->setDerivative(s, 0.0);
        }
    }

    // --- CONTINUOUS ADAPTIVE SENSITIVITY FILTER & PARAMETER DYNAMICS (PATH 1) ---
    double t_now = s.getTime();
    double p1_val = cached_coord_p1 ? cached_coord_p1->getValue(s) : 0.0;
    double p2_val = cached_coord_p2 ? cached_coord_p2->getValue(s) : 0.0;
    double head_pitch_rad = p1_val + p2_val;
    double ref_pitch_deg = computeReferencePitchDeg(t_now, get_multisine_scale());
    double ref_pitch_rad = ref_pitch_deg * SimTK::Pi / 180.0;
    double err_rad = head_pitch_rad - ref_pitch_rad;

    last_error_deg = (head_pitch_rad - ref_pitch_rad) * 180.0 / SimTK::Pi;
    last_ref_pitch_deg = ref_pitch_deg;
    last_actual_pitch_deg = head_pitch_rad * 180.0 / SimTK::Pi;

    // Regressors:
    // psi[0]: Semicircular canal angular velocity in pitch (rad/s)
    // psi[1]: Tonic otolith tilt innovation in pitch (rad)
    // psi[2]: Phasic otolith acceleration innovation in pitch (m/s^2)
    // psi[3]: Muscle spindle velocity proxy (rad/s)
    double psi[4];
    psi[0] = luen_w_pitch;
    psi[1] = last_u_ton_pitch;
    psi[2] = last_u_phas_pitch;
    psi[3] = 0.5 * std::abs(luen_w_pitch);

    // Secondary path 2nd-order head-neck mechanics:
    // omega_n = 20.0 rad/s (~3.18 Hz), zeta = 0.707
    // s^2 + 2*zeta*omega_n*s + omega_n^2 = s^2 + 28.28*s + 400.0
    const double wn2 = 400.0;
    const double two_zeta_wn = 28.28;

    for (int i = 0; i < 4; ++i) {
        if (sv_psif_x1[i] && sv_psif_x2[i]) {
            double x1 = sv_psif_x1[i]->getValue(s);
            double x2 = sv_psif_x2[i]->getValue(s);
            double dx1 = x2;
            double dx2 = -two_zeta_wn * x2 - wn2 * x1 + wn2 * psi[i];
            sv_psif_x1[i]->setDerivative(s, dx1);
            sv_psif_x2[i]->setDerivative(s, dx2);
        }
    }

    // Continuous parameter gradient adaptation
    if (get_enable_adaptation() && t_now >= get_adaptation_start_time()) {
        double rates[4] = {
            get_adaptation_rate_sc(),
            get_adaptation_rate_ton(),
            get_adaptation_rate_phas(),
            get_adaptation_rate_ccr()
        };
        double theta_0[4] = {
            get_K_sc(),
            get_K_ton(),
            get_K_phas(),
            get_Kp_proprioception()
        };
        double theta_min[4] = {0.001, 0.001, 0.200, 0.150};
        double theta_max[4] = {2.500, 2.500, 8.000, 4.000};
        
        const StateVariable* sv_thetas[4] = {
            sv_adapt_K_sc, sv_adapt_K_ton, sv_adapt_K_phas, sv_adapt_kp_prop
        };

        for (int i = 0; i < 4; ++i) {
            if (sv_thetas[i] && sv_psif_x1[i]) {
                double th = sv_thetas[i]->getValue(s);
                double psi_f_i = sv_psif_x1[i]->getValue(s);
                
                // Continuous gradient descent: -gamma * psi_f * e - sigma * (th - th0)
                double dth = -rates[i] * psi_f_i * err_rad - get_sigma_leakage() * (th - theta_0[i]);
                
                // Smooth cubic barrier (C^2 continuous) to prevent boundary violations
                if (th < theta_min[i]) {
                    double viol = theta_min[i] - th;
                    dth += 500.0 * viol * viol * viol;
                } else if (th > theta_max[i]) {
                    double viol = th - theta_max[i];
                    dth -= 500.0 * viol * viol * viol;
                }
                sv_thetas[i]->setDerivative(s, dth);
            }
        }
    } else {
        if (sv_adapt_K_sc) sv_adapt_K_sc->setDerivative(s, 0.0);
        if (sv_adapt_K_ton) sv_adapt_K_ton->setDerivative(s, 0.0);
        if (sv_adapt_K_phas) sv_adapt_K_phas->setDerivative(s, 0.0);
        if (sv_adapt_kp_prop) sv_adapt_kp_prop->setDerivative(s, 0.0);
    }
}"""

assert end_deriv_target in code, "ERROR: end_deriv_target not found in code!"
code = code.replace(end_deriv_target, end_deriv_adaptive)

with open(dst_path, "w", encoding="utf-8") as f:
    f.write(code)

print("SUCCESS: Generated", dst_path, "Length:", len(code))

