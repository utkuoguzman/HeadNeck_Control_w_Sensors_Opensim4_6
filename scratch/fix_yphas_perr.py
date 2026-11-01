with open("HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp", "r", encoding="utf-8") as f:
    src = f.read()

# 1. Keep p_err active (with consistent 40ms-delayed vol_rel_*) so the inverted-pendulum head is stabilized during settling
old_perr = """    // PID Integrator derivatives (using 40ms-delayed vol_rel_* consistently; frozen at static gravity trim during stabilization)
    double p_err = (std::abs(get_desired_pitch()) > 1e-6) ? (traj_pos_pitch - vol_rel_pitch) : 0.0;
    double p_int = sv_err_int[0]->getValue(s);
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    sv_err_int[0]->setDerivative(s, p_err);

    double r_err = (std::abs(get_desired_roll()) > 1e-6) ? (traj_pos_roll - vol_rel_roll) : 0.0;
    double r_int = sv_err_int[1]->getValue(s);
    if (r_int > 0.4 && r_err > 0) r_err = 0.0;
    if (r_int < -0.4 && r_err < 0) r_err = 0.0;
    sv_err_int[1]->setDerivative(s, r_err);

    double y_err = (std::abs(get_desired_yaw()) > 1e-6) ? (traj_pos_yaw - vol_rel_yaw) : 0.0;"""

new_perr = """    // PID Integrator derivatives (using 40ms-delayed vol_rel_* consistently)
    double p_err = traj_pos_pitch - vol_rel_pitch;
    double p_int = sv_err_int[0]->getValue(s);
    if (p_int > 0.6 && p_err > 0) p_err = 0.0;
    if (p_int < -0.6 && p_err < 0) p_err = 0.0;
    sv_err_int[0]->setDerivative(s, p_err);

    double r_err = traj_pos_roll - vol_rel_roll;
    double r_int = sv_err_int[1]->getValue(s);
    if (r_int > 0.4 && r_err > 0) r_err = 0.0;
    if (r_int < -0.4 && r_err < 0) r_err = 0.0;
    sv_err_int[1]->setDerivative(s, r_err);

    double y_err = traj_pos_yaw - vol_rel_yaw;"""

assert old_perr in src, "old_perr not found"
src = src.replace(old_perr, new_perr, 1)

# 2. Restore strictly-proper y_phas = K_phas * p_phas * x_phas (prevents positive skull tangential acceleration feedback loop!)
old_yphas = """        // True Phasic (High-Pass) Otolith Filter: H_phas(s) = K_phas * s / (s + p_phas)
        double y_phas_pitch = get_K_phas() * (u_phas_pitch - get_p_phas() * x_phas_pitch);
        double y_phas_roll  = get_K_phas() * (u_phas_roll  - get_p_phas() * x_phas_roll);
        double y_phas_yaw   = get_K_phas() * (u_phas_yaw   - get_p_phas() * x_phas_yaw);"""

new_yphas = """        // Strictly-Proper Phasic Otolith Filter: H_phas(s) = K_phas * p_phas / (s + p_phas)
        // (Integrates skull tangential acceleration -r_neck*ddtheta into viscous damping above p_phas)
        double y_phas_pitch = get_K_phas() * get_p_phas() * x_phas_pitch;
        double y_phas_roll  = get_K_phas() * get_p_phas() * x_phas_roll;
        double y_phas_yaw   = get_K_phas() * get_p_phas() * x_phas_yaw;"""

assert old_yphas in src, "old_yphas not found"
src = src.replace(old_yphas, new_yphas, 1)

with open("HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp", "w", encoding="utf-8", newline="\r\n") as f:
    f.write(src)

print("Updated HeadNeckNeuralController.cpp!")

