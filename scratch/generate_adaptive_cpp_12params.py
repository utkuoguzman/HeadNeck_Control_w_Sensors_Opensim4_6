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

static double computeReferencePitchVelRad(double t, double multisine_scale) {
    const double t_delay = 1.5;
    const double t_ramp = 1.0;
    if (t < t_delay) return 0.0;
    double t_rel = t - t_delay;
    double w = (t_rel < t_ramp) ? (10.0 * std::pow(t_rel/t_ramp, 3) - 15.0 * std::pow(t_rel/t_ramp, 4) + 6.0 * std::pow(t_rel/t_ramp, 5)) : 1.0;
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double targets_gain[6] = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double targets_phase_deg[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};
    double y_dot_deg = 0.0;
    for (int k = 0; k < 6; ++k) {
        double f = freqs[k];
        double omega_k = 2.0 * SimTK::Pi * f;
        double phi_k = SimTK::Pi * k * (k - 1) / 6.0;
        double base_amp = 0.0215 * std::pow(0.835 / f, 2);
        double A_k = base_amp * multisine_scale;
        double phase_rad = phi_k + targets_phase_deg[k] * SimTK::Pi / 180.0;
        y_dot_deg += A_k * targets_gain[k] * omega_k * std::cos(omega_k * t_rel + phase_rad);
    }
    return w * y_dot_deg * SimTK::Pi / 180.0;
}

static const char* const ADAPT_12_NAMES[12] = {
    "adapt_K_ton",
    "adapt_p_ton",
    "adapt_K_sc",
    "adapt_z_sc",
    "adapt_p_sc",
    "adapt_K_phas",
    "adapt_p_phas",
    "adapt_kp_prop",
    "adapt_z_ccr",
    "adapt_p_ccr",
    "adapt_K_gamma_dyn",
    "adapt_K_gamma_stat"
};
"""

# Replace class name
code = code.replace("HeadNeckNeuralController::", "HeadNeckNeuralControllerAdaptive::")
code = code.replace("HeadNeckNeuralController()", "HeadNeckNeuralControllerAdaptive()")

# In constructor, add properties
target_prop = "constructProperty_fc_voluntary(2.0);"
adaptive_props = """constructProperty_fc_voluntary(2.0);
    constructProperty_enable_adaptation(true);
    constructProperty_adaptation_rate_multiplier(1.0);
    constructProperty_sigma_leakage(0.02);
    constructProperty_adaptation_start_time(2.5);
    constructProperty_multisine_scale(0.142887);"""
code = code.replace(target_prop, adaptive_props)

# Insert helper before constructor
code = code.replace("HeadNeckNeuralControllerAdaptive::HeadNeckNeuralControllerAdaptive()",
                    helper + "\nHeadNeckNeuralControllerAdaptive::HeadNeckNeuralControllerAdaptive()")

# In extendAddToSystem, add 12 adaptive state variables
target_add = """        addStateVariable("sp_fstat_" + mname, SimTK::Stage::Velocity);
    }
}"""
adaptive_add = """        addStateVariable("sp_fstat_" + mname, SimTK::Stage::Velocity);
    }

    // Continuous 12-Parameter Adaptive States & 24 Sensitivity Filter States (Path 1)
    for (int i = 0; i < 12; ++i) {
        addStateVariable(ADAPT_12_NAMES[i], SimTK::Stage::Velocity);
        addStateVariable("psif_x1_" + std::to_string(i), SimTK::Stage::Velocity);
        addStateVariable("psif_x2_" + std::to_string(i), SimTK::Stage::Velocity);
    }
}"""
code = code.replace(target_add, adaptive_add)

# In extendInitStateFromProperties, initialize 12 adaptive states
target_init = """        setStateVariableValue(s, "sp_fstat_" + mname, 0.0);
    }
}"""
adaptive_init = """        setStateVariableValue(s, "sp_fstat_" + mname, 0.0);
    }

    setStateVariableValue(s, "adapt_K_ton", get_K_ton());
    setStateVariableValue(s, "adapt_p_ton", get_p_ton());
    setStateVariableValue(s, "adapt_K_sc", get_K_sc());
    setStateVariableValue(s, "adapt_z_sc", get_z_sc());
    setStateVariableValue(s, "adapt_p_sc", get_p_sc());
    setStateVariableValue(s, "adapt_K_phas", get_K_phas());
    setStateVariableValue(s, "adapt_p_phas", get_p_phas());
    setStateVariableValue(s, "adapt_kp_prop", get_Kp_proprioception());
    setStateVariableValue(s, "adapt_z_ccr", get_z_ccr());
    setStateVariableValue(s, "adapt_p_ccr", get_p_ccr());
    setStateVariableValue(s, "adapt_K_gamma_dyn", get_K_gamma_dyn());
    setStateVariableValue(s, "adapt_K_gamma_stat", get_K_gamma_stat());

    for (int i = 0; i < 12; ++i) {
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

    for (int i = 0; i < 12; ++i) {
        sv_adapt_theta[i] = traverseToStateVariable(ADAPT_12_NAMES[i]);
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

double HeadNeckNeuralControllerAdaptive::getAdaptKton(const SimTK::State& s) const {
    return sv_adapt_theta[0] ? sv_adapt_theta[0]->getValue(s) : getStateVariableValue(s, "adapt_K_ton");
}
double HeadNeckNeuralControllerAdaptive::getAdaptPton(const SimTK::State& s) const {
    return sv_adapt_theta[1] ? sv_adapt_theta[1]->getValue(s) : getStateVariableValue(s, "adapt_p_ton");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKsc(const SimTK::State& s) const {
    return sv_adapt_theta[2] ? sv_adapt_theta[2]->getValue(s) : getStateVariableValue(s, "adapt_K_sc");
}
double HeadNeckNeuralControllerAdaptive::getAdaptZsc(const SimTK::State& s) const {
    return sv_adapt_theta[3] ? sv_adapt_theta[3]->getValue(s) : getStateVariableValue(s, "adapt_z_sc");
}
double HeadNeckNeuralControllerAdaptive::getAdaptPsc(const SimTK::State& s) const {
    return sv_adapt_theta[4] ? sv_adapt_theta[4]->getValue(s) : getStateVariableValue(s, "adapt_p_sc");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKphas(const SimTK::State& s) const {
    return sv_adapt_theta[5] ? sv_adapt_theta[5]->getValue(s) : getStateVariableValue(s, "adapt_K_phas");
}
double HeadNeckNeuralControllerAdaptive::getAdaptPphas(const SimTK::State& s) const {
    return sv_adapt_theta[6] ? sv_adapt_theta[6]->getValue(s) : getStateVariableValue(s, "adapt_p_phas");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKpprop(const SimTK::State& s) const {
    return sv_adapt_theta[7] ? sv_adapt_theta[7]->getValue(s) : getStateVariableValue(s, "adapt_kp_prop");
}
double HeadNeckNeuralControllerAdaptive::getAdaptZccr(const SimTK::State& s) const {
    return sv_adapt_theta[8] ? sv_adapt_theta[8]->getValue(s) : getStateVariableValue(s, "adapt_z_ccr");
}
double HeadNeckNeuralControllerAdaptive::getAdaptPccr(const SimTK::State& s) const {
    return sv_adapt_theta[9] ? sv_adapt_theta[9]->getValue(s) : getStateVariableValue(s, "adapt_p_ccr");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKgammaDyn(const SimTK::State& s) const {
    return sv_adapt_theta[10] ? sv_adapt_theta[10]->getValue(s) : getStateVariableValue(s, "adapt_K_gamma_dyn");
}
double HeadNeckNeuralControllerAdaptive::getAdaptKgammaStat(const SimTK::State& s) const {
    return sv_adapt_theta[11] ? sv_adapt_theta[11]->getValue(s) : getStateVariableValue(s, "adapt_K_gamma_stat");
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

# In computeControls, substitute effective 12 parameters
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

ctrl_adaptive = """        double eff_K_ton   = (sv_adapt_theta[0] && get_enable_adaptation()) ? sv_adapt_theta[0]->getValue(s) : get_K_ton();
        double eff_p_ton   = (sv_adapt_theta[1] && get_enable_adaptation()) ? sv_adapt_theta[1]->getValue(s) : get_p_ton();
        double eff_K_sc    = (sv_adapt_theta[2] && get_enable_adaptation()) ? sv_adapt_theta[2]->getValue(s) : get_K_sc();
        double eff_z_sc    = (sv_adapt_theta[3] && get_enable_adaptation()) ? sv_adapt_theta[3]->getValue(s) : get_z_sc();
        double eff_p_sc    = (sv_adapt_theta[4] && get_enable_adaptation()) ? sv_adapt_theta[4]->getValue(s) : get_p_sc();
        double eff_K_phas  = (sv_adapt_theta[5] && get_enable_adaptation()) ? sv_adapt_theta[5]->getValue(s) : get_K_phas();
        double eff_p_phas  = (sv_adapt_theta[6] && get_enable_adaptation()) ? sv_adapt_theta[6]->getValue(s) : get_p_phas();

        double y_ton_pitch = eff_K_ton * eff_p_ton * x_ton_pitch;
        double y_ton_roll  = eff_K_ton * eff_p_ton * x_ton_roll;
        double y_ton_yaw   = eff_K_ton * eff_p_ton * x_ton_yaw;
        
        if (std::abs(eff_z_sc) < 1e-4) eff_z_sc = (eff_z_sc >= 0.0 ? 1e-4 : -1e-4);
        double b_dir_sc = 1.0;
        double b_state_sc = eff_z_sc - eff_p_sc;

        double y_sc_pitch = eff_K_sc * (b_dir_sc * u_sc_pitch + b_state_sc * x_sc_pitch);
        double y_sc_roll  = eff_K_sc * (b_dir_sc * u_sc_roll  + b_state_sc * x_sc_roll);
        double y_sc_yaw   = eff_K_sc * (b_dir_sc * u_sc_yaw   + b_state_sc * x_sc_yaw);
        
        double y_phas_pitch = eff_K_phas * eff_p_phas * x_phas_pitch;
        double y_phas_roll  = eff_K_phas * eff_p_phas * x_phas_roll;
        double y_phas_yaw   = eff_K_phas * eff_p_phas * x_phas_yaw;"""
code = code.replace(ctrl_target, ctrl_adaptive)

# Spindle reflex in computeControls:
spindle_target = "excitation += get_Kp_proprioception() * (Ia_firing / 100.0) * inhibition_factor;"
spindle_adaptive = """double eff_kp_prop = (sv_adapt_theta[7] && get_enable_adaptation()) ? sv_adapt_theta[7]->getValue(s) : get_Kp_proprioception();
            excitation += eff_kp_prop * (Ia_firing / 100.0) * inhibition_factor;"""
code = code.replace(spindle_target, spindle_adaptive)

# Gamma controls in computeControls:
gamma_target = """        if (n_controls >= 3) {
            my_ctrl[1] = 100.0 * get_K_gamma_dyn() * u_alpha[i];  // Gamma dynamic (pulses/s)
            my_ctrl[2] = 100.0 * get_K_gamma_stat() * u_alpha[i]; // Gamma static (pulses/s)
        }"""
gamma_adaptive = """        if (n_controls >= 3) {
            double eff_k_gdyn = (sv_adapt_theta[10] && get_enable_adaptation()) ? sv_adapt_theta[10]->getValue(s) : get_K_gamma_dyn();
            double eff_k_gstat = (sv_adapt_theta[11] && get_enable_adaptation()) ? sv_adapt_theta[11]->getValue(s) : get_K_gamma_stat();
            my_ctrl[1] = 100.0 * eff_k_gdyn * u_alpha[i];  // Gamma dynamic (pulses/s)
            my_ctrl[2] = 100.0 * eff_k_gstat * u_alpha[i]; // Gamma static (pulses/s)
        }"""
code = code.replace(gamma_target, gamma_adaptive)

# Gamma values in observer (computeStateVariableDerivatives):
obs_gamma_target = """        const double k_gdyn = 100.0 * get_K_gamma_dyn();
        const double k_gstat = 100.0 * get_K_gamma_stat();"""
obs_gamma_adaptive = """        double eff_k_gdyn = (sv_adapt_theta[10] && get_enable_adaptation()) ? sv_adapt_theta[10]->getValue(s) : get_K_gamma_dyn();
        double eff_k_gstat = (sv_adapt_theta[11] && get_enable_adaptation()) ? sv_adapt_theta[11]->getValue(s) : get_K_gamma_stat();
        const double k_gdyn = 100.0 * eff_k_gdyn;
        const double k_gstat = 100.0 * eff_k_gstat;"""
code = code.replace(obs_gamma_target, obs_gamma_adaptive)

# Capture u_ton_pitch and u_phas_pitch
code = code.replace("double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;",
                    "double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - traj_pos_pitch;\n        last_u_ton_pitch = u_ton_pitch;")

code = code.replace("double u_phas_pitch = a_lin_head[0] - expected_ah_x;",
                    "double u_phas_pitch = a_lin_head[0] - expected_ah_x;\n        last_u_phas_pitch = u_phas_pitch;")

# In computeStateVariableDerivatives, update filter derivatives with effective poles
tf_deriv_target = """        sv_ton[0]->setDerivative(s, -get_p_ton() * x_ton_pitch + u_ton_pitch);
        sv_ton[1]->setDerivative(s, -get_p_ton() * x_ton_roll + u_ton_roll);
        sv_ton[2]->setDerivative(s, -get_p_ton() * x_ton_yaw + u_ton_yaw);

        sv_sc[0]->setDerivative(s, -get_p_sc() * x_sc_pitch + u_sc_pitch);
        sv_sc[1]->setDerivative(s, -get_p_sc() * x_sc_roll + u_sc_roll);
        sv_sc[2]->setDerivative(s, -get_p_sc() * x_sc_yaw + u_sc_yaw);

        sv_phas[0]->setDerivative(s, -get_p_phas() * x_phas_pitch + u_phas_pitch);
        sv_phas[1]->setDerivative(s, -get_p_phas() * x_phas_roll + u_phas_roll);
        sv_phas[2]->setDerivative(s, -get_p_phas() * x_phas_yaw + u_phas_yaw);"""

tf_deriv_adaptive = """        double eff_p_ton  = (sv_adapt_theta[1] && get_enable_adaptation()) ? sv_adapt_theta[1]->getValue(s) : get_p_ton();
        double eff_p_sc   = (sv_adapt_theta[4] && get_enable_adaptation()) ? sv_adapt_theta[4]->getValue(s) : get_p_sc();
        double eff_p_phas = (sv_adapt_theta[6] && get_enable_adaptation()) ? sv_adapt_theta[6]->getValue(s) : get_p_phas();

        sv_ton[0]->setDerivative(s, -eff_p_ton * x_ton_pitch + u_ton_pitch);
        sv_ton[1]->setDerivative(s, -eff_p_ton * x_ton_roll + u_ton_roll);
        sv_ton[2]->setDerivative(s, -eff_p_ton * x_ton_yaw + u_ton_yaw);

        sv_sc[0]->setDerivative(s, -eff_p_sc * x_sc_pitch + u_sc_pitch);
        sv_sc[1]->setDerivative(s, -eff_p_sc * x_sc_roll + u_sc_roll);
        sv_sc[2]->setDerivative(s, -eff_p_sc * x_sc_yaw + u_sc_yaw);

        sv_phas[0]->setDerivative(s, -eff_p_phas * x_phas_pitch + u_phas_pitch);
        sv_phas[1]->setDerivative(s, -eff_p_phas * x_phas_roll + u_phas_roll);
        sv_phas[2]->setDerivative(s, -eff_p_phas * x_phas_yaw + u_phas_yaw);"""
code = code.replace(tf_deriv_target, tf_deriv_adaptive)

# CCR pole update in computeStateVariableDerivatives:
ccr_pole_target = "double p_ccr = get_p_ccr();"
ccr_pole_adaptive = "double p_ccr = (sv_adapt_theta[9] && get_enable_adaptation()) ? sv_adapt_theta[9]->getValue(s) : get_p_ccr();"
code = code.replace(ccr_pole_target, ccr_pole_adaptive)

# In computeStateVariableDerivatives, append 12-parameter continuous adaptation dynamics
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

    // --- CONTINUOUS 12-PARAMETER ADAPTIVE SENSITIVITY FILTER & PARAMETER DYNAMICS (PATH 1) ---
    double t_now = s.getTime();
    double p1_val = cached_coord_p1 ? cached_coord_p1->getValue(s) : 0.0;
    double p2_val = cached_coord_p2 ? cached_coord_p2->getValue(s) : 0.0;
    double head_pitch_rad = p1_val + p2_val;
    double ref_pitch_deg = computeReferencePitchDeg(t_now, get_multisine_scale());
    double ref_pitch_rad = ref_pitch_deg * SimTK::Pi / 180.0;
    double ref_pitch_vel_rad = computeReferencePitchVelRad(t_now, get_multisine_scale());
    
    double err_pos_rad = head_pitch_rad - ref_pitch_rad;
    double err_vel_rad = luen_w_pitch - ref_pitch_vel_rad;

    last_error_deg = err_pos_rad * 180.0 / SimTK::Pi;
    last_ref_pitch_deg = ref_pitch_deg;
    last_actual_pitch_deg = head_pitch_rad * 180.0 / SimTK::Pi;

    double x_ton_p  = sv_ton[0]->getValue(s);
    double x_sc_p   = sv_sc[0]->getValue(s);
    double x_phas_p = sv_phas[0]->getValue(s);

    // 12 Physical Regressors matching CMA-ES parameters:
    //  0: K_ton          -> last_u_ton_pitch
    //  1: p_ton          -> x_ton_pitch
    //  2: K_sc           -> luen_w_pitch
    //  3: z_sc           -> x_sc_pitch
    //  4: p_sc           -> -x_sc_pitch
    //  5: K_phas         -> last_u_phas_pitch
    //  6: p_phas         -> x_phas_pitch
    //  7: kp_prop        -> 0.5 * abs(luen_w_pitch)
    //  8: z_ccr          -> 0.5 * luen_w_pitch
    //  9: p_ccr          -> -0.5 * luen_w_pitch
    // 10: K_gamma_dyn    -> luen_w_pitch (velocity error gradient)
    // 11: K_gamma_stat   -> head_pitch_rad (position error gradient)
    double psi[12];
    psi[0]  = last_u_ton_pitch;
    psi[1]  = x_ton_p;
    psi[2]  = luen_w_pitch;
    psi[3]  = x_sc_p;
    psi[4]  = -x_sc_p;
    psi[5]  = last_u_phas_pitch;
    psi[6]  = x_phas_p;
    psi[7]  = 0.5 * std::abs(luen_w_pitch);
    psi[8]  = 0.5 * luen_w_pitch;
    psi[9]  = -0.5 * luen_w_pitch;
    psi[10] = luen_w_pitch;
    psi[11] = head_pitch_rad;

    // Secondary path 2nd-order head-neck mechanics:
    // omega_n = 20.0 rad/s (~3.18 Hz), zeta = 0.707
    // s^2 + 2*zeta*omega_n*s + omega_n^2 = s^2 + 28.28*s + 400.0
    const double wn2 = 400.0;
    const double two_zeta_wn = 28.28;

    for (int i = 0; i < 12; ++i) {
        if (sv_psif_x1[i] && sv_psif_x2[i]) {
            double x1 = sv_psif_x1[i]->getValue(s);
            double x2 = sv_psif_x2[i]->getValue(s);
            double dx1 = x2;
            double dx2 = -two_zeta_wn * x2 - wn2 * x1 + wn2 * psi[i];
            sv_psif_x1[i]->setDerivative(s, dx1);
            sv_psif_x2[i]->setDerivative(s, dx2);
        }
    }

    // Continuous 12-parameter Normalized LMS gradient adaptation
    if (get_enable_adaptation() && t_now >= get_adaptation_start_time()) {
        double mult = get_adaptation_rate_multiplier();
        if (mult < 0.01) mult = 0.01;

        const double theta_0[12] = {
            get_K_ton(),
            get_p_ton(),
            get_K_sc(),
            get_z_sc(),
            get_p_sc(),
            get_K_phas(),
            get_p_phas(),
            get_Kp_proprioception(),
            get_z_ccr(),
            get_p_ccr(),
            get_K_gamma_dyn(),
            get_K_gamma_stat()
        };

        const double theta_min[12] = {
            0.001, 5.0, 0.001, 0.001, 15.0, 0.20, 1.0, 0.15, 0.0001, 20.0, 0.01, 0.01
        };

        const double theta_max[12] = {
            2.50, 150.0, 2.50, 60.0, 160.0, 8.00, 45.0, 4.00, 25.0, 240.0, 2.00, 2.00
        };

        // Measured empirical regressor scales (RMS during multisine excitation)
        const double reg_scale[12] = {
            0.030,   // K_ton: last_u_ton_pitch ~ 0.03 rad
            0.0005,  // p_ton: x_ton ~ 0.0005 rad
            0.250,   // K_sc: luen_w_pitch ~ 0.25 rad/s
            0.003,   // z_sc: x_sc ~ 0.003 rad
            0.003,   // p_sc: -x_sc ~ 0.003 rad
            0.150,   // K_phas: last_u_phas_pitch ~ 0.15 m/s^2
            0.010,   // p_phas: x_phas ~ 0.01 m/s
            0.125,   // kp_prop: 0.5*|w| ~ 0.125 rad/s
            0.125,   // z_ccr: 0.5*w ~ 0.125 rad/s
            0.125,   // p_ccr: -0.5*w ~ 0.125 rad/s
            0.250,   // K_gamma_dyn: luen_w_pitch ~ 0.25 rad/s
            0.030    // K_gamma_stat: head_pitch_rad ~ 0.03 rad
        };

        // Error normalization scales (RMS tracking error target)
        const double err_scale[12] = {
            0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.20, 0.02
        };

        // Dimensionless normalized learning rate:
        // gamma_norm = 0.010 causes ~10-25% proportional adaptation across all 12 parameters
        const double gamma_norm = 0.010 * mult;

        for (int i = 0; i < 12; ++i) {
            if (sv_adapt_theta[i] && sv_psif_x1[i]) {
                double th = sv_adapt_theta[i]->getValue(s);
                double psi_f_i = sv_psif_x1[i]->getValue(s);
                double err_i = (i == 10) ? err_vel_rad : err_pos_rad;
                
                double norm_psi = psi_f_i / reg_scale[i];
                double norm_err = err_i / err_scale[i];

                // Normalized continuous gradient descent:
                // dth/dt = -gamma_norm * theta_0[i] * norm_psi * norm_err - sigma_leak * (th - theta_0[i])
                double dth = -gamma_norm * theta_0[i] * norm_psi * norm_err - get_sigma_leakage() * (th - theta_0[i]);
                
                // Scale-invariant smooth cubic barrier (C^2 continuous) to prevent boundary violations
                if (th < theta_min[i]) {
                    double viol = (theta_min[i] - th) / theta_0[i];
                    dth += 1000.0 * theta_0[i] * viol * viol * viol;
                } else if (th > theta_max[i]) {
                    double viol = (th - theta_max[i]) / theta_0[i];
                    dth -= 1000.0 * theta_0[i] * viol * viol * viol;
                }
                sv_adapt_theta[i]->setDerivative(s, dth);
            }
        }
    } else {
        for (int i = 0; i < 12; ++i) {
            if (sv_adapt_theta[i]) sv_adapt_theta[i]->setDerivative(s, 0.0);
        }
    }
}"""

assert end_deriv_target in code, "ERROR: end_deriv_target not found in code!"
code = code.replace(end_deriv_target, end_deriv_adaptive)

with open(dst_path, "w", encoding="utf-8") as f:
    f.write(code)

print("SUCCESS: Generated 12-parameter HeadNeckNeuralControllerAdaptive.cpp, Length:", len(code))
