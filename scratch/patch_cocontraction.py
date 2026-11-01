import re

# 1. Patch HeadNeckNeuralController.h
h_path = r"HeadNeckNeuralController_plugin/HeadNeckNeuralController.h"
with open(h_path, "r", encoding="utf-8") as f:
    h_content = f.read()

if "cached_u_min" not in h_content:
    target = "mutable std::vector<double> cached_F_max;"
    replacement = "mutable std::vector<double> cached_F_max;\n    mutable std::vector<double> cached_u_min;"
    assert target in h_content, "target not found in .h"
    h_content = h_content.replace(target, replacement, 1)
    with open(h_path, "w", encoding="utf-8") as f:
        f.write(h_content)
    print("Patched HeadNeckNeuralController.h successfully.")
else:
    print("HeadNeckNeuralController.h already has cached_u_min.")

# 2. Patch HeadNeckNeuralController.cpp
cpp_path = r"HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp"
with open(cpp_path, "r", encoding="utf-8") as f:
    cpp_content = f.read()

# Replace initialization of cached_u_min in initializeGeometryAndBaseline
init_target = """    cached_scale_Ia.resize(num_muscles, 0.0);
    cached_F_max.resize(num_muscles, 0.0);
    
    for(size_t i=0; i<active_muscle_indices.size(); ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        std::string mname_i = m.getName();"""

init_replacement = """    cached_scale_Ia.resize(num_muscles, 0.0);
    cached_F_max.resize(num_muscles, 0.0);
    cached_u_min.resize(num_muscles, 0.03);
    
    for(size_t i=0; i<active_muscle_indices.size(); ++i) {
        const Muscle& m = muscles.get(active_muscle_indices[i]);
        std::string mname_i = m.getName();
        
        // Anatomical Co-Contraction Tiers
        double u_val = 0.030; // Default for eye muscles / unclassified
        if (mname_i.find("stern_mast") != std::string::npos ||
            mname_i.find("cleid_mast") != std::string::npos ||
            mname_i.find("cleid_occ")  != std::string::npos ||
            mname_i.find("digastric")  != std::string::npos ||
            mname_i.find("Geniohyoid") != std::string::npos ||
            mname_i.find("Mylohyoid")  != std::string::npos ||
            mname_i.find("Stylohyoid") != std::string::npos ||
            mname_i.find("Sterno_hyoid") != std::string::npos ||
            mname_i.find("SternoThyroid") != std::string::npos ||
            mname_i.find("Omo_hyoid")  != std::string::npos) {
            u_val = 0.025; // Superficial Flexors (2.5%)
        } else if (mname_i.find("long_cap") != std::string::npos ||
                   mname_i.find("long_col") != std::string::npos ||
                   mname_i.find("scalenus") != std::string::npos) {
            u_val = 0.040; // Deep Cervical Flexors (4.0%)
        } else if (mname_i.find("trap_") != std::string::npos ||
                   mname_i.find("splen_cap") != std::string::npos ||
                   mname_i.find("splen_cerv") != std::string::npos ||
                   mname_i.find("levator_scap") != std::string::npos) {
            u_val = 0.050; // Superficial Extensors (5.0%)
        } else if (mname_i.find("semi_cap") != std::string::npos ||
                   mname_i.find("semi_cerv") != std::string::npos ||
                   mname_i.find("longissi_") != std::string::npos ||
                   mname_i.find("iliocost_") != std::string::npos ||
                   mname_i.find("rectcap_") != std::string::npos ||
                   mname_i.find("obl_cap")  != std::string::npos) {
            u_val = 0.070; // Deep Cervical Extensors (7.0%)
        }
        cached_u_min[i] = u_val;"""

assert init_target in cpp_content, "init_target not found in .cpp"
cpp_content = cpp_content.replace(init_target, init_replacement, 1)

# In computeControls: QP solver initialization
qp_init_target = "    std::vector<double> u_alpha(N, 0.03);"
qp_init_replacement = """    std::vector<double> u_alpha(N);
    for(int i=0; i<N; ++i) u_alpha[i] = cached_u_min[i];"""
assert qp_init_target in cpp_content, "qp_init_target not found in .cpp"
cpp_content = cpp_content.replace(qp_init_target, qp_init_replacement, 1)

# In computeControls: QP lower bound check
qp_bound_target = """                if(a_unconstrained < 0.03) {
                    u_alpha[i] = 0.03;
                    is_free[i] = false;
                    bounds_violated = true;"""
qp_bound_replacement = """                if(a_unconstrained < cached_u_min[i]) {
                    u_alpha[i] = cached_u_min[i];
                    is_free[i] = false;
                    bounds_violated = true;"""
assert qp_bound_target in cpp_content, "qp_bound_target not found in .cpp"
cpp_content = cpp_content.replace(qp_bound_target, qp_bound_replacement, 1)

# In computeControls: muscle loop u_min check & clamping
clamp_target = """            double u_min = 0.03;
            double u_max = 0.08;
            
            if (u_alpha[i] <= u_min) {
                inhibition_factor = K_recip;
            } else if (u_alpha[i] >= u_max) {
                inhibition_factor = 1.0;
            } else {
                double t = (u_alpha[i] - u_min) / (u_max - u_min);
                double smooth_t = t * t * (3.0 - 2.0 * t);
                inhibition_factor = K_recip + smooth_t * (1.0 - K_recip);
            }
            
            // We scale the firing rate (0-500) into excitation (0-1) using Kp_proprioception
            excitation += get_Kp_proprioception() * (Ia_firing / 100.0) * inhibition_factor; 
        }
        
        excitation = std::clamp(excitation, 0.03, 1.0);"""

clamp_replacement = """            double u_min_i = cached_u_min[i];
            double u_max_i = u_min_i + 0.05;
            
            if (u_alpha[i] <= u_min_i) {
                inhibition_factor = K_recip;
            } else if (u_alpha[i] >= u_max_i) {
                inhibition_factor = 1.0;
            } else {
                double t = (u_alpha[i] - u_min_i) / (u_max_i - u_min_i);
                double smooth_t = t * t * (3.0 - 2.0 * t);
                inhibition_factor = K_recip + smooth_t * (1.0 - K_recip);
            }
            
            // We scale the firing rate (0-500) into excitation (0-1) using Kp_proprioception
            excitation += get_Kp_proprioception() * (Ia_firing / 100.0) * inhibition_factor; 
        }
        
        excitation = std::clamp(excitation, cached_u_min[i], 1.0);"""

assert clamp_target in cpp_content, "clamp_target not found in .cpp"
cpp_content = cpp_content.replace(clamp_target, clamp_replacement, 1)

with open(cpp_path, "w", encoding="utf-8") as f:
    f.write(cpp_content)
print("Patched HeadNeckNeuralController.cpp successfully.")

