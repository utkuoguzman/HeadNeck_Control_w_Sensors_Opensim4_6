# Patch Header
header_path = "scratch/HeadNeckNeuralController_plugin_split/HeadNeckNeuralController.h"
with open(header_path, "r", encoding="utf-8") as f:
    h_content = f.read()

target_h = 'OpenSim_DECLARE_PROPERTY(delay_smith_proprio, double, "Proprioceptive transport delay in seconds for Smith compensation (default: 0.015)");'
replacement_h = target_h + '\n    OpenSim_DECLARE_PROPERTY(delay_afferent_proprio, double, "Proprioceptive afferent transport delay in seconds arriving at the central observer (default: 0.0)");'

if target_h in h_content:
    h_content = h_content.replace(target_h, replacement_h)
    with open(header_path, "w", encoding="utf-8") as f:
        f.write(h_content)
    print("Header patched successfully!")
else:
    print("Error: target_h not found in header!")

# Patch Cpp
cpp_path = "scratch/HeadNeckNeuralController_plugin_split/HeadNeckNeuralController.cpp"
with open(cpp_path, "r", encoding="utf-8") as f:
    c_content = f.read()

# 1. Constructor property initialization
target_c1 = 'constructProperty_delay_smith_proprio(0.015); // Disabled by default during neck study; set true for eye-movement study'
replacement_c1 = target_c1 + '\n    constructProperty_delay_afferent_proprio(0.0);'

if target_c1 in c_content:
    c_content = c_content.replace(target_c1, replacement_c1)
    print("Constructor patched successfully!")
else:
    print("Error: target_c1 not found in cpp!")

# 2. Proprioceptive innovation in extendComputeStateVariableDerivatives
target_c2 = """    // Proprioceptive global belief (trunk angle + relative neck angle from 72 muscles)
    double prop_gnd_pitch = q_gnd_p + est_neck_rel_angle[0];
    double prop_gnd_roll  = q_gnd_r + est_neck_rel_angle[1];
    double prop_gnd_yaw   = q_gnd_y + est_neck_rel_angle[2];
    
    // Error vector in World frame [Roll=0, Yaw=1, Pitch=2]
    SimTK::Vec3 e_prop_world;
    if (get_use_smith_predictor() && !history.empty()) {
        DelayedData prop_delayed = getDelayedData(s.getTime(), get_delay_smith_proprio());
        if (prop_delayed.record.valid) {
            double pred_roll  = prop_delayed.record.theta[0];
            double pred_yaw   = prop_delayed.record.theta[1];
            double pred_pitch = prop_delayed.record.theta[2];
            e_prop_world = SimTK::Vec3(prop_gnd_roll - pred_roll, prop_gnd_yaw - pred_yaw, prop_gnd_pitch - pred_pitch);
        } else {
            e_prop_world = SimTK::Vec3(prop_gnd_roll - est_roll, prop_gnd_yaw - est_yaw, prop_gnd_pitch - est_pitch);
        }
    } else {
        e_prop_world = SimTK::Vec3(prop_gnd_roll - est_roll, prop_gnd_yaw - est_yaw, prop_gnd_pitch - est_pitch);
    }"""

replacement_c2 = """    // --- ANATOMICALLY SPLIT PROPRIOCEPTIVE AFFERENT CONDUCTION ---
    // In human anatomy, spindle action potentials take ~5-8 ms (delay_afferent_proprio)
    // to travel along dorsal roots and cuneate fasciculus to the brainstem/cerebellum.
    double arriving_rel_pitch = est_neck_rel_angle[0];
    double arriving_rel_roll  = est_neck_rel_angle[1];
    double arriving_rel_yaw   = est_neck_rel_angle[2];
    double arriving_trunk_pitch = q_gnd_p;
    double arriving_trunk_roll  = q_gnd_r;
    double arriving_trunk_yaw   = q_gnd_y;

    double tau_aff = get_delay_afferent_proprio();
    DelayedData aff_data;
    if (tau_aff > 1e-5 && !history.empty()) {
        aff_data = getDelayedData(s.getTime(), tau_aff);
        if (aff_data.record.valid) {
            arriving_rel_pitch = aff_data.record.est_neck_rel_angle[0];
            arriving_rel_roll  = aff_data.record.est_neck_rel_angle[1];
            arriving_rel_yaw   = aff_data.record.est_neck_rel_angle[2];
            arriving_trunk_pitch = aff_data.record.theta_trunk[2];
            arriving_trunk_roll  = aff_data.record.theta_trunk[0];
            arriving_trunk_yaw   = aff_data.record.theta_trunk[1];
        }
    }

    double prop_gnd_pitch = arriving_trunk_pitch + arriving_rel_pitch;
    double prop_gnd_roll  = arriving_trunk_roll  + arriving_rel_roll;
    double prop_gnd_yaw   = arriving_trunk_yaw   + arriving_rel_yaw;
    
    // Error vector in World frame [Roll=0, Yaw=1, Pitch=2]
    SimTK::Vec3 e_prop_world;
    if (get_use_smith_predictor() && !history.empty()) {
        // Smith Predictor / Cerebellar Forward Model:
        // Compares the arriving delayed measurement prop_gnd(t - tau_aff)
        // against the stored estimate from the EXACT SAME DELAYED HORIZON pred(t - tau_aff)!
        double tau_pred = get_delay_smith_proprio();
        DelayedData prop_delayed = (std::abs(tau_pred - tau_aff) < 1e-5 && aff_data.record.valid)
                                   ? aff_data
                                   : getDelayedData(s.getTime(), tau_pred);
        if (prop_delayed.record.valid) {
            double pred_roll  = prop_delayed.record.theta[0];
            double pred_yaw   = prop_delayed.record.theta[1];
            double pred_pitch = prop_delayed.record.theta[2];
            e_prop_world = SimTK::Vec3(prop_gnd_roll - pred_roll, prop_gnd_yaw - pred_yaw, prop_gnd_pitch - pred_pitch);
        } else {
            e_prop_world = SimTK::Vec3(prop_gnd_roll - est_roll, prop_gnd_yaw - est_yaw, prop_gnd_pitch - est_pitch);
        }
    } else {
        // Original Mahony without delay compensation:
        // Compares arriving delayed measurement against current un-delayed estimate est(t)
        e_prop_world = SimTK::Vec3(prop_gnd_roll - est_roll, prop_gnd_yaw - est_yaw, prop_gnd_pitch - est_pitch);
    }"""

# Normalize newlines
target_norm2 = target_c2.replace("\r\n", "\n")
c_content_norm = c_content.replace("\r\n", "\n")

if target_norm2 in c_content_norm:
    c_content_norm = c_content_norm.replace(target_norm2, replacement_c2.replace("\r\n", "\n"))
    with open(cpp_path, "w", encoding="utf-8") as f:
        f.write(c_content_norm)
    print("Cpp patched successfully!")
else:
    print("Error: target_c2 not found in cpp!")

