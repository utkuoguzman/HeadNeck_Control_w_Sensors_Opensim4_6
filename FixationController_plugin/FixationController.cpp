#include "FixationController.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <Schneider15VestibularAfferent.h>
#include <Millard12EqMuscleWithSimplifiedAfferent.h>

using namespace OpenSim;
using namespace SimTK;

FixationController::FixationController() : Controller() {
    constructProperties();
}

void FixationController::constructProperties() {
    constructProperty_target_location(Vec3(2.0, 0.5, 0.0)); // 2m straight ahead, 0.5m up
    constructProperty_kp_horizontal(5.0);
    constructProperty_kd_horizontal(0.01);
    constructProperty_ki_horizontal(0.0);
    constructProperty_kp_vertical(5.0);
    constructProperty_kd_vertical(0.01);
    constructProperty_ki_vertical(0.0);
    constructProperty_kp_torsion(5.0);
    constructProperty_kd_torsion(0.01);
    constructProperty_ki_torsion(0.0);
    constructProperty_K_vor(1.0);
    constructProperty_K_cor(0.1);
}

void FixationController::computeControls(const State& s, Vector& controls) const {
    if (!_model) return;

    Vec3 target_pt = get_target_location();
    const auto& ground = _model->getGround();
    const auto& skull = _model->getBodySet().get("skull");

    double dt = s.getTime() - last_time;
    if (dt < 0 || dt > 0.1) dt = 0.01; // prevent large jumps on reset
    last_time = s.getTime();

    // We process both Right and Left eyes
    std::string prefixes[2] = {"r_", "l_"};

    for (int i = 0; i < 2; ++i) {
        std::string pre = prefixes[i];
        
        // 1. Get current eye coordinates
        const auto& add_abd = _model->getCoordinateSet().get(pre + "eye_add_abd");
        const auto& sup_inf = _model->getCoordinateSet().get(pre + "eye_sup_inf");
        const auto& inc_exc = _model->getCoordinateSet().get(pre + "eye_inc_exc");
        
        // 2. Compute Target Angles (Inverse Kinematics relative to SKULL)
        // target_pt is now treated as a point fixed in the skull frame
        const auto& eye_body = _model->getBodySet().get(pre + "eye");
        Vec3 eye_origin_G = eye_body.getPositionInGround(s);
        Vec3 eye_origin_skull = ground.findStationLocationInAnotherFrame(s, eye_origin_G, skull);
        Vec3 to_target_skull = target_pt - eye_origin_skull;
        
        // X is forward, Y is up, Z is right.
        // Horizontal angle (Yaw) -> rotation around Y (axis 0 1 0).
        // Right hand rule: thumb UP (+Y), fingers curl from +X (forward) to -Z (left).
        // Therefore, a POSITIVE rotation around Y means the eye looks LEFT.
        double desired_yaw = std::atan2(-to_target_skull[2], to_target_skull[0]);
        
        // Vertical angle (Pitch) -> rotation around Z (axis 0 0 1).
        // Right hand rule: thumb RIGHT (+Z), fingers curl from +X (forward) to +Y (up).
        // Therefore, a POSITIVE rotation around Z means the eye looks UP.
        double desired_pitch = std::atan2(to_target_skull[1], to_target_skull[0]);
        
        double desired_torsion = 0.0;

        // 3. Current state
        double current_yaw = add_abd.getValue(s);
        double current_yaw_v = add_abd.getSpeedValue(s);
        
        double current_pitch = sup_inf.getValue(s);
        double current_pitch_v = sup_inf.getSpeedValue(s);
        
        double current_torsion = inc_exc.getValue(s);
        double current_torsion_v = inc_exc.getSpeedValue(s);

        // 4. PID Feedback Control
        double e_yaw = desired_yaw - current_yaw;
        double e_pitch = desired_pitch - current_pitch;
        double e_torsion = desired_torsion - current_torsion;
        
        int_err_yaw[i] += e_yaw * dt;
        int_err_pitch[i] += e_pitch * dt;
        int_err_torsion[i] += e_torsion * dt;
        
        // Anti-windup limits for integral terms
        int_err_yaw[i] = std::max(-1.0, std::min(1.0, int_err_yaw[i]));
        int_err_pitch[i] = std::max(-1.0, std::min(1.0, int_err_pitch[i]));
        int_err_torsion[i] = std::max(-1.0, std::min(1.0, int_err_torsion[i]));

        double err_yaw = get_kp_horizontal() * e_yaw + get_ki_horizontal() * int_err_yaw[i] - get_kd_horizontal() * current_yaw_v;
        double err_pitch = get_kp_vertical() * e_pitch + get_ki_vertical() * int_err_pitch[i] - get_kd_vertical() * current_pitch_v;
        double err_torsion = get_kp_torsion() * e_torsion + get_ki_torsion() * int_err_torsion[i] - get_kd_torsion() * current_torsion_v;

        // --- VOR (Vestibulo-Ocular Reflex) ---
        // Read canal firing rates from the vestibular sensor (Absolute Head Velocity)
        Vec3 canal_rate(0.0);
        for (const auto& v : _model->getComponentList<Schneider15VestibularAfferent>()) {
            canal_rate = v.getCanalFiringRate(s);
        }
        
        // canal_rate: X=Roll, Y=Yaw, Z=Pitch
        // Feed-forward VOR perfectly opposes absolute head motion
        err_torsion -= get_K_vor() * canal_rate[0]; // Roll
        err_yaw     -= get_K_vor() * canal_rate[1]; // Yaw
        err_pitch   -= get_K_vor() * canal_rate[2]; // Pitch

        // --- COR (Cervico-Ocular Reflex) ---
        // Simplified Tensor Network: 70 spindles -> 3D Neck Velocity -> 6 Eye Muscles
        // Compute angular velocity of skull relative to the base of the neck (spine)
        const auto& spine = _model->getBodySet().get("spine");
        Vec3 head_vel_G = skull.getAngularVelocityInGround(s);
        Vec3 spine_vel_G = spine.getAngularVelocityInGround(s);
        Vec3 neck_vel_G = head_vel_G - spine_vel_G;
        
        // Project neck velocity into the Skull frame (X=Roll, Y=Yaw, Z=Pitch)
        Vec3 neck_vel_skull = ground.expressVectorInAnotherFrame(s, neck_vel_G, skull);
        
        // Feed-forward COR perfectly opposes neck motion
        err_torsion -= get_K_cor() * neck_vel_skull[0];
        err_yaw     -= get_K_cor() * neck_vel_skull[1];
        err_pitch   -= get_K_cor() * neck_vel_skull[2];

        // 5. Map Control Effort to Muscles
        // Horizontal: Medial / Lateral Rectus
        // POSITIVE err_yaw means we need to move LEFT.
        // Right eye (i=0): Medial Rectus pulls LEFT (inward), Lateral Rectus pulls RIGHT (outward).
        // Left eye (i=1): Lateral Rectus pulls LEFT (outward), Medial Rectus pulls RIGHT (inward).
        
        double lat_rect_ctrl = 0.0, med_rect_ctrl = 0.0;
        if (i == 0) { // Right Eye
            if (err_yaw > 0) med_rect_ctrl = err_yaw;   // move LEFT
            else lat_rect_ctrl = -err_yaw;              // move RIGHT
        } else { // Left Eye
            if (err_yaw > 0) lat_rect_ctrl = err_yaw;   // move LEFT
            else med_rect_ctrl = -err_yaw;              // move RIGHT
        }

        // Vertical: Superior / Inferior Rectus
        // positive pitch (up) is Superior Rectus
        double sup_rect_ctrl = 0.0, inf_rect_ctrl = 0.0;
        if (err_pitch > 0) sup_rect_ctrl = err_pitch;
        else inf_rect_ctrl = -err_pitch;

        // Torsion: Superior / Inferior Oblique
        double sup_obl_ctrl = 0.0, inf_obl_ctrl = 0.0;
        if (err_torsion > 0) sup_obl_ctrl = err_torsion;
        else inf_obl_ctrl = -err_torsion;

        // 6. Apply to Muscles
        if (_model->getMuscles().contains(pre + "Lateral_Rectus")) {
            _model->getMuscles().get(pre + "Lateral_Rectus").addInControls(Vector(1, lat_rect_ctrl), controls);
            _model->getMuscles().get(pre + "Medial_Rectus").addInControls(Vector(1, med_rect_ctrl), controls);
            _model->getMuscles().get(pre + "Superior_Rectus").addInControls(Vector(1, sup_rect_ctrl), controls);
            _model->getMuscles().get(pre + "Inferior_Rectus").addInControls(Vector(1, inf_rect_ctrl), controls);
            _model->getMuscles().get(pre + "Superior_Oblique").addInControls(Vector(1, sup_obl_ctrl), controls);
            _model->getMuscles().get(pre + "Inferior_Oblique").addInControls(Vector(1, inf_obl_ctrl), controls);
        }
    }
}

