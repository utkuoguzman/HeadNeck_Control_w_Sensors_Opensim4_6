with open("HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp", "r", encoding="utf-8") as f:
    src = f.read()

# 1. Restore vol_act gate on K_recip so all 72 muscles retain full CCR anti-buckling stiffness (p1 vs p2) during stabilization
old_recip = """            // Synergy-Based Sherrington Reciprocal Inhibition (Descending Presynaptic Inhibition)
            // Uses a C1-continuous smoothstep based on descending motor command u_alpha[i]:
            // When u_alpha[i] <= u_min_i (antagonist floor), stretch reflex is gated by K_recip_inhib.
            // By u_alpha[i] >= u_min_i + 0.05 (active agonist), stretch reflex is 100% active (1.0).
            double K_recip = get_K_recip_inhib();"""

new_recip = """            // Synergy-Based Reciprocal Inhibition (Descending Presynaptic Inhibition)
            // Gated by active voluntary trajectory vigor so all 72 muscles retain full anti-buckling CCR stiffness (p1 vs p2) during postural stabilization:
            double vol_act = std::clamp(
                (std::abs(traj_pos_pitch) + std::abs(traj_pos_roll) + std::abs(traj_pos_yaw)
                 + 0.2 * (std::abs(traj_vel_pitch) + std::abs(traj_vel_roll) + std::abs(traj_vel_yaw))) / 0.02,
                0.0, 1.0);
            double K_recip = 1.0 - vol_act * (1.0 - get_K_recip_inhib());"""

assert old_recip in src, "old_recip not found"
src = src.replace(old_recip, new_recip, 1)

# 2. Use 1st-order vol_filt for tau_vol (preserves phase margin of Ki=40.28 gravity trim during 0..1.5s settling)
old_tau_vol = """    // 2. Filtered Voluntary Drive from 2nd-Order Cortical Low-Pass Filter (Transcortical Delay + 2 Hz Roll-Off)
    tau_vol[0] = sv_vol_filt2[0]->getValue(s);
    tau_vol[1] = sv_vol_filt2[1]->getValue(s);
    tau_vol[2] = sv_vol_filt2[2]->getValue(s);"""

new_tau_vol = """    // 2. Filtered Voluntary Drive from Cortical Low-Pass Filter (Transcortical Delay + 2 Hz Cutoff)
    tau_vol[0] = sv_vol_filt[0]->getValue(s);
    tau_vol[1] = sv_vol_filt[1]->getValue(s);
    tau_vol[2] = sv_vol_filt[2]->getValue(s);"""

assert old_tau_vol in src, "old_tau_vol not found"
src = src.replace(old_tau_vol, new_tau_vol, 1)

with open("HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp", "w", encoding="utf-8", newline="\r\n") as f:
    f.write(src)

print("Updated HeadNeckNeuralController.cpp (restored anti-buckling CCR and 1st-order vol_filt phase margin)!")

