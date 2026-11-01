with open("HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.cpp", "r", encoding="utf-8") as f:
    cpp_text = f.read()

# 1. Replace evalClosedLoopBode with I_pivot = 0.0953 kg*m^2 + Multiplicative Space-Mapping Anchoring at theta_0
start_bode = cpp_text.find("// =========================================================================================")
end_bode   = cpp_text.find("static double computeReferencePitchDeg")
assert start_bode != -1 and end_bode != -1, "evalClosedLoopBode markers not found"

new_bode_section = """// =========================================================================================
// EXACT CLOSED-LOOP COMPLEX BODE TRANSFER FUNCTION MODEL G_CL(j*omega; theta)
// Anchored via Multiplicative Space-Mapping at the OpenSim baseline theta_0 and using the
// true parallel-axis cervical pivot inertia I_pivot = I_zz + m_head * r_neck^2 = 0.0953 kg*m^2.
// Parameter vector theta[0..11]:
//   0: K_ton, 1: p_ton, 2: K_sc, 3: z_sc, 4: p_sc, 5: K_phas, 6: p_phas,
//   7: kp_prop, 8: z_ccr, 9: p_ccr, 10: K_gamma_dyn, 11: K_gamma_stat
// =========================================================================================
static std::complex<double> evalRawClosedLoopBode(double freq_hz, const double th[12]) {
    const double omega = 2.0 * SimTK::Pi * freq_hz;
    const std::complex<double> s(0.0, omega);
    const std::complex<double> s2(-omega * omega, 0.0);

    // Delays & Muscle Excitation-Contraction Dynamics
    const std::complex<double> D_vcr = std::exp(-s * 0.010);
    const std::complex<double> D_ccr = std::exp(-s * 0.015);
    const std::complex<double> D_vol = std::exp(-s * 0.040);
    const std::complex<double> H_act = 1.0 / ((1.0 + 0.012 * s) * (1.0 + 0.035 * s));

    // True parallel-axis cervical pivot inertia: I_pivot = I_zz (0.0226) + m_head (4.30) * r_neck^2 (0.13^2) = 0.0953 kg*m^2
    // Places the passive + reflex resonance at ~2.2 Hz matching OpenSim multibody dynamics
    const double I_pivot = 0.0953;
    const double B_pass  = 0.35;
    const double K_pass  = 3.80;
    const double Ki_pitch = 40.2786;
    const double omega_vol = 2.0 * SimTK::Pi * 2.0;
    const std::complex<double> Z_int = (Ki_pitch / s) * (omega_vol / (s + omega_vol)) * D_vol;
    const std::complex<double> D_mech = I_pivot * s2 + B_pass * s + K_pass + Z_int;

    // 1. Tonic Otolith Reflex: H_ton(s) = K_ton * p_ton / (s + p_ton)
    const double K_ton = std::max(1e-4, th[0]);
    const double p_ton = std::max(1.0,  th[1]);
    const std::complex<double> Z_ton = K_ton * (p_ton / (s + p_ton)) * D_vcr * H_act;

    // 2. Center-Frequency Normalized Semicircular Canal Reflex:
    //    H_sc(s) = (K_sc / sqrt(z_sc / p_sc)) * (s + z_sc) / (s + p_sc) * (1 + 0.02442*s) * s
    const double K_sc = std::max(1e-4, th[2]);
    const double z_sc = std::max(1e-3, th[3]);
    const double p_sc = std::max(1.0,  th[4]);
    const double norm_sc = 1.0 / std::sqrt( std::max(1e-4, z_sc / p_sc) );
    const std::complex<double> H_canal = 1.0 + 0.02442 * s;
    const std::complex<double> Z_sc = 18.0 * K_sc * norm_sc * ((s + z_sc) / (s + p_sc)) * H_canal * s * D_vcr * H_act;

    // 3. Phasic Otolith Linear Acceleration Reflex:
    //    a_x_skull = s^2 * X_sled - r_neck * s^2 * Theta
    const double K_phas = std::max(1e-4, th[5]);
    const double p_phas = std::max(0.5,  th[6]);
    const double r_neck = 0.13;
    const std::complex<double> H_phas = K_phas * (p_phas / (s + p_phas)) * D_vcr * H_act;
    const std::complex<double> Z_phas_rot = -r_neck * s2 * H_phas;

    // 4. Center-Frequency Normalized Cervico-Collic Reflex (CCR) + Fusimotor Coactivation:
    const double kp_prop = std::max(1e-4, th[7]);
    const double z_ccr   = std::max(1e-3, th[8]);
    const double p_ccr   = std::max(1.0,  th[9]);
    const double k_gdyn  = std::max(0.01, th[10]);
    const double k_gstat = std::max(0.01, th[11]);
    const double norm_ccr = 1.0 / std::sqrt( std::max(1e-4, z_ccr / p_ccr) );
    const std::complex<double> H_spindle = 9.5 * ((1.0 + 0.35 * k_gstat) + (0.055 + 0.045 * k_gdyn) * s);
    const std::complex<double> Z_ccr = kp_prop * norm_ccr * ((s + z_ccr) / (s + p_ccr)) * H_spindle * D_ccr * H_act;

    const std::complex<double> D_CL = D_mech + Z_ton + Z_sc + Z_phas_rot + Z_ccr;
    const double m_head_r = 4.30 * 0.13; // 0.559 kg*m inertial coupling
    const std::complex<double> N_CL = (-s2) * (m_head_r + 0.025 * H_phas) * (180.0 / SimTK::Pi);

    return N_CL / D_CL;
}

// Multiplicative Space-Mapped Closed-Loop Bode Evaluator:
// Anchors G_CL(j*omega_k; th) to the exact measured OpenSim response at theta_0 so that
// at th == theta_0, the model error is 0.00 dB and 0.00 deg across all 6 frequencies!
static std::complex<double> evalSpaceMappedBode(int k_idx, const double th[12], const double th0[12]) {
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    // Exact OpenSim baseline response measured at theta_0 (KP0_Rescaled verification):
    const double osim_base_gain[6]      = {33.90, 175.74, 260.57, 645.64, 959.64, 714.44};
    const double osim_base_phase_deg[6] = {-9.90, -97.80, -99.90, -189.40, -271.10, -349.30};

    double ph_rad = osim_base_phase_deg[k_idx] * SimTK::Pi / 180.0;
    std::complex<double> G_osim_0 = std::polar(osim_base_gain[k_idx], ph_rad);
    std::complex<double> G_raw_0  = evalRawClosedLoopBode(freqs[k_idx], th0);
    std::complex<double> G_raw_th = evalRawClosedLoopBode(freqs[k_idx], th);

    return G_osim_0 * (G_raw_th / G_raw_0);
}

"""

cpp_text = cpp_text[:start_bode] + new_bode_section + cpp_text[end_bode:]

# 2. Initialize demodulator states in extendInitStateFromProperties to the exact OpenSim baseline phasor G_osim_0
old_init_demod = """    const double init_freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double tgt_gains[6]  = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double tgt_phases[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};
    for (int k = 0; k < 6; ++k) {
        // Initialize demodulators to target phasor so initial error is 0 before excitation settles
        double ph_rad = tgt_phases[k] * SimTK::Pi / 180.0;
        double S0 = tgt_gains[k] * std::cos(ph_rad);
        double C0 = tgt_gains[k] * std::sin(ph_rad);
        setStateVariableValue(s, "psif_x1_" + std::to_string(k),     S0);
        setStateVariableValue(s, "psif_x1_" + std::to_string(k + 6), S0);
        setStateVariableValue(s, "psif_x2_" + std::to_string(k),     C0);
        setStateVariableValue(s, "psif_x2_" + std::to_string(k + 6), C0);
    }"""

new_init_demod = """    // Initialize Lyapunov Harmonic Observer states to the exact OpenSim baseline phasors at theta_0
    const double osim_base_gain[6]      = {33.90, 175.74, 260.57, 645.64, 959.64, 714.44};
    const double osim_base_phase_deg[6] = {-9.90, -97.80, -99.90, -189.40, -271.10, -349.30};
    for (int k = 0; k < 6; ++k) {
        double ph_rad = osim_base_phase_deg[k] * SimTK::Pi / 180.0;
        double S0 = osim_base_gain[k] * std::cos(ph_rad);
        double C0 = osim_base_gain[k] * std::sin(ph_rad);
        setStateVariableValue(s, "psif_x1_" + std::to_string(k),     S0);
        setStateVariableValue(s, "psif_x1_" + std::to_string(k + 6), S0);
        setStateVariableValue(s, "psif_x2_" + std::to_string(k),     C0);
        setStateVariableValue(s, "psif_x2_" + std::to_string(k + 6), C0);
    }"""

assert old_init_demod in cpp_text, "old_init_demod not found"
cpp_text = cpp_text.replace(old_init_demod, new_init_demod, 1)

# 3. Replace the HBAC block in computeStateVariableDerivatives with Coupled Lyapunov Harmonic Observer + Space-Mapped Sensitivity + Annealing Taper
start_hbac = cpp_text.find("    // --- CONTINUOUS 12-PARAMETER HARMONIC BODE-SENSITIVITY ADAPTIVE CONTROL (OPTION 1: HBAC) ---")
end_hbac   = cpp_text.find("void HeadNeckNeuralControllerAdaptive::computeControls(const SimTK::State& s, SimTK::Vector& controls) const {")
assert start_hbac != -1 and end_hbac != -1, "HBAC markers not found"

new_lyapunov_hbac = """    // --- CONTINUOUS 12-PARAMETER LYAPUNOV HARMONIC OBSERVER & SPACE-MAPPED BODE ADAPTIVE CONTROL ---
    double t_now = s.getTime();
    double p1_val = cached_coord_p1 ? cached_coord_p1->getValue(s) : 0.0;
    double p2_val = cached_coord_p2 ? cached_coord_p2->getValue(s) : 0.0;
    double head_pitch_rad = p1_val + p2_val;
    double head_pitch_deg = head_pitch_rad * 180.0 / SimTK::Pi;
    double ref_pitch_deg = computeReferencePitchDeg(t_now, get_multisine_scale());
    double ref_pitch_rad = ref_pitch_deg * SimTK::Pi / 180.0;
    
    double err_pos_rad = head_pitch_rad - ref_pitch_rad;
    last_error_deg = err_pos_rad * 180.0 / SimTK::Pi;
    last_ref_pitch_deg = ref_pitch_deg;
    last_actual_pitch_deg = head_pitch_deg;

    // 1. Coupled Lyapunov Harmonic Observer (Hostetter 1980 / Bitmead 1984):
    //    Reconstructs the sum of all 6 harmonics theta_recon(t) and updates each harmonic phasor
    //    (S_k, C_k) using ONLY the unexplained residual e_resid(t) = head_pitch_deg(t) - theta_recon(t).
    //    This subtracts out the +-5.8 deg low-frequency wave before demodulating the 0.020 deg high-frequency
    //    channels (5.583 Hz and 7.927 Hz), completely eliminating the 290x cross-frequency spectral leakage!
    const double freqs[6] = {0.324, 0.783, 1.099, 2.446, 5.583, 7.927};
    const double targets_gain[6] = {86.55, 236.93, 319.74, 423.34, 503.30, 572.69};
    const double targets_phase_deg[6] = {-7.03, -96.96, -109.23, -161.34, -201.38, -247.62};
    const double m_scale = get_multisine_scale();
    const double t_delay = 1.5;
    const double t_ramp  = 1.0;

    if (t_now < t_delay + t_ramp) {
        for (int k = 0; k < 6; ++k) {
            sv_psif_x1[k]->setDerivative(s, 0.0);
            sv_psif_x1[k + 6]->setDerivative(s, 0.0);
            sv_psif_x2[k]->setDerivative(s, 0.0);
            sv_psif_x2[k + 6]->setDerivative(s, 0.0);
        }
    } else {
        double t_rel = t_now - t_delay;
        double A_arr[6], psi_arr[6];
        double theta_recon_deg = 0.0;

        for (int k = 0; k < 6; ++k) {
            double f = freqs[k];
            double omega_k = 2.0 * SimTK::Pi * f;
            double phi_k = SimTK::Pi * k * (k - 1) / 6.0;
            double base_amp = 0.0215 * std::pow(0.835 / f, 2);
            A_arr[k] = std::max(1e-6, base_amp * m_scale);
            psi_arr[k] = omega_k * t_rel + phi_k;

            double S_hat_k = sv_psif_x1[k + 6]->getValue(s);
            double C_hat_k = sv_psif_x2[k + 6]->getValue(s);
            theta_recon_deg += A_arr[k] * (S_hat_k * std::sin(psi_arr[k]) + C_hat_k * std::cos(psi_arr[k]));
        }

        // Unexplained harmonic residual (all 6 estimated harmonics cancelled out!)
        double e_resid_deg = head_pitch_deg - theta_recon_deg;

        for (int k = 0; k < 6; ++k) {
            double s1 = sv_psif_x1[k]->getValue(s);
            double s2 = sv_psif_x1[k + 6]->getValue(s);
            double c1 = sv_psif_x2[k]->getValue(s);
            double c2 = sv_psif_x2[k + 6]->getValue(s);

            // Normalize residual by total multisine envelope so high-frequency channels are not amplified by 1/A_k noise
            double A_eff = std::max(A_arr[k], 0.0005);
            double dS_innov = 2.0 * e_resid_deg * std::sin(psi_arr[k]) / A_eff;
            double dC_innov = 2.0 * e_resid_deg * std::cos(psi_arr[k]) / A_eff;

            double omega_lp = 0.25 * (2.0 * SimTK::Pi * freqs[k]);
            sv_psif_x1[k]->setDerivative(s,     omega_lp * (s2 + dS_innov - s1));
            sv_psif_x1[k + 6]->setDerivative(s, omega_lp * (s1 - s2));
            sv_psif_x2[k]->setDerivative(s,     omega_lp * (c2 + dC_innov - c1));
            sv_psif_x2[k + 6]->setDerivative(s, omega_lp * (c1 - c2));
        }
    }

    // 2. Continuous 12-parameter Space-Mapped Bode-Sensitivity Gradient Descent with Annealing Taper
    if (get_enable_adaptation() && t_now >= get_adaptation_start_time()) {
        double mult = get_adaptation_rate_multiplier();
        if (mult < 0.01) mult = 0.01;

        // Smooth cosine annealing taper: full learning rate for t in [2.5s, 14.0s],
        // smoothly tapering to 0 by t = 18.0s so that t in [18.0s, 30.0s] is 100% stationary!
        double taper = 1.0;
        if (t_now >= 18.0) {
            taper = 0.0;
        } else if (t_now > 13.0) {
            double u_tap = (t_now - 13.0) / 5.0;
            taper = 0.5 * (1.0 + std::cos(SimTK::Pi * u_tap));
        }

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
            0.01, 5.0, 0.01, 1.0, 15.0, 0.20, 1.0, 0.03, 1.0, 20.0, 0.05, 0.05
        };

        const double theta_max[12] = {
            3.00, 150.0, 1.50, 80.0, 160.0, 10.00, 45.0, 2.00, 80.0, 240.0, 1.50, 1.50
        };

        double th_curr[12];
        for (int i = 0; i < 12; ++i) {
            th_curr[i] = sv_adapt_theta[i] ? sv_adapt_theta[i]->getValue(s) : theta_0[i];
        }

        // Compute online Bode Gain Error (dB) and Phase Error (deg) across all 6 frequencies
        // using Space-Mapped Closed-Loop Bode fused with the Lyapunov Harmonic Observer
        double err_gain_norm[6]  = {0.0};
        double err_phase_norm[6] = {0.0};

        for (int k = 0; k < 6; ++k) {
            double S_hat = sv_psif_x1[k + 6]->getValue(s);
            double C_hat = sv_psif_x2[k + 6]->getValue(s);
            double sim_gain_demod  = std::sqrt(S_hat * S_hat + C_hat * C_hat);
            double sim_phase_demod = std::atan2(C_hat, S_hat) * 180.0 / SimTK::Pi;

            // Space-mapped analytical prediction at current th_curr (exact at th_curr == theta_0)
            std::complex<double> G_sm = evalSpaceMappedBode(k, th_curr, theta_0);
            double sm_gain  = std::abs(G_sm);
            double sm_phase = std::arg(G_sm) * 180.0 / SimTK::Pi;

            // Weight Lyapunov demodulator more strongly at low/mid frequencies where SNR is high,
            // and Space-Mapped model more strongly at 5.58/7.93 Hz where sled amplitude is microns
            const double w_demod[6] = {0.50, 0.50, 0.50, 0.35, 0.15, 0.10};
            double alpha_k = w_demod[k];

            double eff_gain = (sim_gain_demod > 5.0)
                              ? std::pow(sim_gain_demod, alpha_k) * std::pow(std::max(5.0, sm_gain), 1.0 - alpha_k)
                              : std::max(5.0, sm_gain);

            double d_demod_tgt = std::remainder(sim_phase_demod - targets_phase_deg[k], 360.0);
            double d_sm_tgt    = std::remainder(sm_phase        - targets_phase_deg[k], 360.0);
            double phase_diff_deg = alpha_k * d_demod_tgt + (1.0 - alpha_k) * d_sm_tgt;

            double gain_db_diff = 20.0 * std::log10(std::max(1e-3, eff_gain) / targets_gain[k]);
            err_gain_norm[k]  = gain_db_diff / 1.0;    // 1.0 dB corridor
            err_phase_norm[k] = phase_diff_deg / 10.0; // 10.0 deg corridor
        }

        // Compute exact 12x12 Space-Mapped Complex Bode Sensitivity Jacobian J(th_curr)
        // and apply diagonally-preconditioned Gauss-Newton update
        const double gamma_hbac = 0.035 * mult * taper;

        for (int i = 0; i < 12; ++i) {
            if (!sv_adapt_theta[i]) continue;
            double th_i = th_curr[i];
            double d_th = std::max(1e-5, 1e-4 * std::abs(theta_0[i]));

            double th_plus[12], th_minus[12];
            for (int j = 0; j < 12; ++j) {
                th_plus[j]  = th_curr[j];
                th_minus[j] = th_curr[j];
            }
            th_plus[i]  += d_th;
            th_minus[i] -= d_th;

            double grad_i  = 0.0;
            double hess_ii = 0.0;

            for (int k = 0; k < 6; ++k) {
                std::complex<double> G_p = evalSpaceMappedBode(k, th_plus,  theta_0);
                std::complex<double> G_m = evalSpaceMappedBode(k, th_minus, theta_0);

                // Dimensionless logarithmic sensitivities: theta_0[i] * d(dB/1.0)/d(theta_i) and theta_0[i] * d(deg/10.0)/d(theta_i)
                std::complex<double> d_lnG = std::log(G_p / G_m) / (2.0 * d_th);
                double s_gain_k  = theta_0[i] * (20.0 / std::log(10.0)) * d_lnG.real() / 1.0;
                double s_phase_k = theta_0[i] * (180.0 / SimTK::Pi)     * d_lnG.imag() / 10.0;

                grad_i  += s_gain_k * err_gain_norm[k] + s_phase_k * err_phase_norm[k];
                hess_ii += s_gain_k * s_gain_k + s_phase_k * s_phase_k;
            }

            // Diagonally-preconditioned natural gradient descent step
            double norm_step = grad_i / (std::sqrt(hess_ii) + 0.35);
            norm_step = std::clamp(norm_step, -1.5, 1.5);

            double dth = -gamma_hbac * theta_0[i] * norm_step - taper * get_sigma_leakage() * (th_i - theta_0[i]);

            // Smooth C^2 cubic barrier at physiological bounds
            if (th_i < theta_min[i]) {
                double viol = (theta_min[i] - th_i) / std::max(1e-3, theta_0[i]);
                dth += 1000.0 * theta_0[i] * viol * viol * viol;
            } else if (th_i > theta_max[i]) {
                double viol = (th_i - theta_max[i]) / std::max(1e-3, theta_0[i]);
                dth -= 1000.0 * theta_0[i] * viol * viol * viol;
            }
            sv_adapt_theta[i]->setDerivative(s, dth);
        }
    } else {
        for (int i = 0; i < 12; ++i) {
            if (sv_adapt_theta[i]) sv_adapt_theta[i]->setDerivative(s, 0.0);
        }
    }
}

"""

cpp_text = cpp_text[:start_hbac] + new_lyapunov_hbac + cpp_text[end_hbac:]

with open("HeadNeckNeuralControllerAdaptive_plugin/HeadNeckNeuralControllerAdaptive.cpp", "w", encoding="utf-8") as f:
    f.write(cpp_text)

print("Successfully upgraded HeadNeckNeuralControllerAdaptive.cpp with Lyapunov Harmonic Observer + Space-Mapped I_pivot=0.0953!")

