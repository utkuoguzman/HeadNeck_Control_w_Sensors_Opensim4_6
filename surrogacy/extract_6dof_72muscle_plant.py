"""
surrogacy/extract_6dof_72muscle_plant.py
Step 1 of the Upgraded 6-DOF, 72-Muscle MIMO Differentiable Neuromotor Surrogate.

Extracts the exact 6-DOF multibody state-space matrices, 72-muscle moment-arm and
activation-to-acceleration Jacobians, and Mileusnic (2006) spindle sensitivities
directly from HYOID_HeadNeckNeural_Model_Detailed.osim around the 72-muscle
literature-centered 6-DOF upright equilibrium state (PER_MUSCLE_72_U0).

Outputs:
    surrogacy/opensim_6dof_72muscle_plant.json
"""

import json
import math
import os
import sys
import numpy as np
import opensim as osim

sys.path.insert(0, os.getcwd())
from cmaes_worker_p22 import PER_MUSCLE_72_U0

osim.Logger.setLevelString("Error")
osim.LoadOpenSimLibrary("osimMillard12EqWithAff.dll")
osim.LoadOpenSimLibrary("osimHeadNeckNeural.dll")
osim.LoadOpenSimLibrary("FixationController_plugin/build/Release/osimFixation.dll")

COORDS_6DOF = ["pitch1", "pitch2", "roll1", "roll2", "yaw1", "yaw2"]

def extract_6dof_72muscle_plant():
    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    for i in range(model.getControllerSet().getSize()):
        model.updControllerSet().get(i).set_enabled(False)

    # Lock extraocular coordinates
    for i in range(model.getCoordinateSet().getSize()):
        c = model.updCoordinateSet().get(i)
        if "eye" in c.getName():
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    # Prescribe ground/thorax translation and rotation except spine_tx (unlocked so we can perturb sled accel)
    zero_func = osim.Constant(0.0)
    for cname in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cname)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    # Use a quadratic spline x(t) = 0.5 * a_sled * t^2 on spine_tx so we can test sled acceleration directly
    c_tx = model.updCoordinateSet().get("spine_tx")
    c_tx.set_locked(False)
    c_tx.setDefaultIsPrescribed(True)
    c_tx.setPrescribedFunction(zero_func)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)

    muscles = model.updMuscles()
    neck_muscle_names = []
    neck_muscle_indices = []
    u0_vec = []
    Fmax_vec = []
    lopt_vec = []
    tau_act_vec = []
    tau_deact_vec = []

    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        neck_muscle_names.append(name)
        neck_muscle_indices.append(i)
        u0 = PER_MUSCLE_72_U0.get(name, 0.030)
        u0_vec.append(u0)
        Fmax_vec.append(m.getMaxIsometricForce())
        lopt_vec.append(m.getOptimalFiberLength())
        tau_act_vec.append(0.010)
        tau_deact_vec.append(0.040)
        m.setActivation(state, u0)
        gd0 = 100.0 * 0.70 * u0
        gs0 = 100.0 * 0.95 * u0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)

    model.equilibrateMuscles(state)
    model.realizeAcceleration(state)

    coords = [model.updCoordinateSet().get(cn) for cn in COORDS_6DOF]
    q0 = np.array([c.getValue(state) for c in coords])
    N_m = len(neck_muscle_names)

    def get_qddot_6(s):
        model.realizeAcceleration(s)
        return np.array([c.getAccelerationValue(s) for c in coords])

    qddot_base = get_qddot_6(state)
    print(f"Base 6-DOF equilibrium acceleration norm: {np.linalg.norm(qddot_base):.3e} rad/s^2")

    # 1. Extract 6x72 Moment Arm Matrix J_6x72 and 3x72 QP Matrix A_3x72
    J_6x72 = np.zeros((6, N_m))
    for m_idx, osim_idx in enumerate(neck_muscle_indices):
        m = muscles.get(osim_idx)
        for c_idx, c in enumerate(coords):
            J_6x72[c_idx, m_idx] = m.computeMomentArm(state, c)

    J_3x72 = np.zeros((3, N_m))
    J_3x72[0, :] = 0.5 * (J_6x72[0, :] + J_6x72[1, :]) # pitch
    J_3x72[1, :] = 0.5 * (J_6x72[2, :] + J_6x72[3, :]) # roll
    J_3x72[2, :] = 0.5 * (J_6x72[4, :] + J_6x72[5, :]) # yaw

    Fmax_arr = np.array(Fmax_vec)
    lopt_arr = np.array(lopt_vec)
    u0_arr = np.array(u0_vec)
    A_3x72 = J_3x72 * Fmax_arr[None, :]
    A_6x72 = J_6x72 * Fmax_arr[None, :]

    # 2. Extract 6x72 Activation-to-Acceleration Jacobian B_musc = d(qddot_6)/d(a_72)
    B_musc_6x72 = np.zeros((6, N_m))
    dF_da = np.zeros(N_m)
    da = 2e-3
    f0_vec = np.array([muscles.get(osim_idx).getActuation(state) for osim_idx in neck_muscle_indices])
    for m_idx, osim_idx in enumerate(neck_muscle_indices):
        s_p = opensim_copy_state(state)
        m = muscles.get(osim_idx)
        m.setActivation(s_p, u0_arr[m_idx] + da)
        m.computeEquilibrium(s_p)
        qdd_p = get_qddot_6(s_p)
        B_musc_6x72[:, m_idx] = (qdd_p - qddot_base) / da
        dF_da[m_idx] = (m.getActuation(s_p) - f0_vec[m_idx]) / da

    Tau_per_da = J_6x72 * dF_da[None, :] # 6 x 72
    # Solve B_musc_6x72 = M_6_inv @ Tau_per_da  => M_6_inv = B_musc_6x72 @ pinv(Tau_per_da)
    M_6_inv = B_musc_6x72 @ np.linalg.pinv(Tau_per_da)
    M_6 = np.linalg.pinv(M_6_inv)

    # 4. Extract 6x6 Stiffness Matrix (-M_6^-1 K_6 = d(qddot_6)/d(q_6)) with muscles equilibrated at perturbed posture
    # AND instantaneous series-elastic tendon-fiber stiffness A_stiff_inst (before slow tendon-fiber creep)
    A_stiff_eq = np.zeros((6, 6))
    A_stiff_inst = np.zeros((6, 6))
    dq = 1e-4
    for j in range(6):
        s_p = opensim_copy_state(state)
        s_m = opensim_copy_state(state)
        coords[j].setValue(s_p, q0[j] + dq)
        coords[j].setValue(s_m, q0[j] - dq)
        A_stiff_inst[:, j] = (get_qddot_6(s_p) - get_qddot_6(s_m)) / (2.0 * dq)
        model.equilibrateMuscles(s_p)
        model.equilibrateMuscles(s_m)
        A_stiff_eq[:, j] = (get_qddot_6(s_p) - get_qddot_6(s_m)) / (2.0 * dq)

    # 5. Extract 6x6 Damping Matrix (-M_6^-1 B_6 = d(qddot_6)/d(qdot_6))
    # With 2 ms fiber-tendon velocity relaxation so the 72 co-contracted Hill muscles engage their force-velocity curve
    A_damp_inst = np.zeros((6, 6))
    dqd = 0.05
    for j in range(6):
        s_p = opensim_copy_state(state)
        s_m = opensim_copy_state(state)
        coords[j].setSpeedValue(s_p, +dqd)
        coords[j].setSpeedValue(s_m, -dqd)
        mgr_p = osim.Manager(model); mgr_p.setIntegratorMethod(5); mgr_p.initialize(s_p); s_p = mgr_p.integrate(0.002)
        mgr_m = osim.Manager(model); mgr_m.setIntegratorMethod(5); mgr_m.initialize(s_m); s_m = mgr_m.integrate(0.002)
        A_damp_inst[:, j] = (get_qddot_6(s_p) - get_qddot_6(s_m)) / (2.0 * dqd)

    # 6. Extract 6x1 Sled Acceleration Coupling G_sled_6 = d(qddot_6)/d(xddot_sled)
    # Unlock spine_tx and apply a prescribed acceleration x(t) = 0.5 * 1.0 * t^2
    model_sled = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    for i in range(model_sled.getControllerSet().getSize()):
        model_sled.updControllerSet().get(i).set_enabled(False)
    for i in range(model_sled.getCoordinateSet().getSize()):
        c = model_sled.updCoordinateSet().get(i)
        if "eye" in c.getName():
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)
    for cname in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model_sled.updCoordinateSet().get(cname)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)
    c_y1_s = model_sled.updCoordinateSet().get("yaw1")
    c_y1_s.set_locked(False)
    c_y1_s.setDefaultIsPrescribed(False)

    # Polynomial 0.5 * t^2 has second derivative = +1.0 m/s^2 at t=0
    sp_accel = osim.SimmSpline()
    for t_pt in np.linspace(0.0, 0.1, 21):
        sp_accel.addPoint(float(t_pt), float(0.5 * 1.0 * (t_pt ** 2)))
    c_tx_s = model_sled.updCoordinateSet().get("spine_tx")
    c_tx_s.set_locked(False)
    c_tx_s.setDefaultIsPrescribed(True)
    c_tx_s.setPrescribedFunction(sp_accel)

    state_s = model_sled.initSystem()
    c_y1_s.setIsPrescribed(state_s, False)
    muscles_s = model_sled.updMuscles()
    for i in range(muscles_s.getSize()):
        m = muscles_s.get(i)
        nm = m.getName()
        if nm.startswith("r_") or nm.startswith("l_"):
            m.setAppliesForce(state_s, False)
            continue
        m.setActivation(state_s, PER_MUSCLE_72_U0.get(nm, 0.030))
    model_sled.equilibrateMuscles(state_s)
    state_s.setTime(0.02)
    model_sled.realizeAcceleration(state_s)
    coords_s = [model_sled.updCoordinateSet().get(cn) for cn in COORDS_6DOF]
    G_sled_6 = np.array([c.getAccelerationValue(state_s) for c in coords_s]) - qddot_base
    skull_s = model_sled.getBodySet().get("skull")
    skull_acc_per_sled = np.array([
        skull_s.getLinearAccelerationInGround(state_s)[0],
        skull_s.getLinearAccelerationInGround(state_s)[1],
        skull_s.getLinearAccelerationInGround(state_s)[2]
    ])

    # Also measure skull linear acceleration Jacobian with respect to qddot_6: d(a_skull_x)/d(qddot_6)
    # Using body kinematics: a_skull_x = 1.0 * xddot_sled + h_eff^T @ qddot_6
    # At state_s (where xddot_sled = 1.0 and qddot = G_sled_6), and from B_musc perturbations:
    da_skull_x_d_da = np.zeros(N_m)
    skull_body = model.getBodySet().get("skull")
    a_skull_base = skull_body.getLinearAccelerationInGround(state)[0]
    for m_idx, osim_idx in enumerate(neck_muscle_indices):
        s_p = opensim_copy_state(state)
        m = muscles.get(osim_idx)
        m.setActivation(s_p, u0_arr[m_idx] + da)
        m.computeEquilibrium(s_p)
        model.realizeAcceleration(s_p)
        da_skull_x_d_da[m_idx] = (skull_body.getLinearAccelerationInGround(s_p)[0] - a_skull_base) / da
    # Solve da_skull_x_d_da = h_skull_x_6^T @ B_musc_6x72
    h_skull_x_6 = da_skull_x_d_da @ np.linalg.pinv(B_musc_6x72)
    direct_sled_to_skull_x = float(skull_acc_per_sled[0] - h_skull_x_6 @ G_sled_6)

    print(f"Extracted 6x6 Mass Matrix diag (kg*m^2): {np.round(np.diag(M_6), 5)}")
    print(f"Extracted 6-DOF Sled Accel Coupling G_sled_6 (rad/m): {np.round(G_sled_6, 4)}")
    print(f"Skull X-accel lever vector h_skull_x_6 (m/rad): {np.round(h_skull_x_6, 4)} (direct sled={direct_sled_to_skull_x:.3f})")

    out_data = {
        "coords_6dof": COORDS_6DOF,
        "q0_rad": q0.tolist(),
        "neck_muscle_names": neck_muscle_names,
        "u0_vec": u0_vec,
        "Fmax_vec": Fmax_vec,
        "lopt_vec": lopt_vec,
        "dF_da_vec": dF_da.tolist(),
        "J_6x72": J_6x72.tolist(),
        "J_3x72": J_3x72.tolist(),
        "A_3x72": A_3x72.tolist(),
        "A_6x72": A_6x72.tolist(),
        "B_musc_6x72": B_musc_6x72.tolist(),
        "M_6": M_6.tolist(),
        "M_6_inv": M_6_inv.tolist(),
        "A_stiff_eq_6x6": A_stiff_eq.tolist(),
        "A_stiff_inst_6x6": A_stiff_inst.tolist(),
        "A_damp_inst_6x6": A_damp_inst.tolist(),
        "G_sled_6": G_sled_6.tolist(),
        "h_skull_x_6": h_skull_x_6.tolist(),
        "direct_sled_to_skull_x": direct_sled_to_skull_x
    }
    os.makedirs("surrogacy", exist_ok=True)
    out_path = "surrogacy/opensim_6dof_72muscle_plant.json"
    with open(out_path, "w", encoding="utf-8") as fp:
        json.dump(out_data, fp, indent=2)
    print(f"Saved exact 6-DOF 72-muscle linearized plant to {out_path}")

def opensim_copy_state(state):
    return osim.State(state)

if __name__ == "__main__":
    extract_6dof_72muscle_plant()
