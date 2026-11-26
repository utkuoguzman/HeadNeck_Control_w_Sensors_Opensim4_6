#!/usr/bin/env python3
"""
Build and verify `osim_files/HYOID_HeadNeckNeural_Model_KuoLigaments.osim`.

Imports Calvin Kuo et al. (2019) 67 true cervical ligaments (15 anatomical groups:
ALL, PLL, LF, IS, IT, LN, JC, TM, CLT, AL, API, AAA, PAA, AAO, PAO) from
`Ligaments_model/Head_Neck_Model_OpenSim46.osim` into a clean copy of
`osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim` (leaving both originals
untouched).

Architectural choices (approved by user):
  1. Exclude Kuo's 12 `AF_` disc annulus fibers so HYOID's 8 `BushingForce`s
     are not double-counted.
  2. Keep HYOID's 6 independent coupled neck DOFs (18 `CoordinateCouplerConstraint`s).
  3. Keep HYOID's 84 `Millard12EqMuscleWithAfferents` (72 neck + 12 extraocular)
     and 17 bodies (including separate `skull` and `jaw` welded via `jawjnt`).
"""

import copy
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HYOID_IN = os.path.join(REPO_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")
KUO_IN = os.path.join(REPO_ROOT, "Ligaments_model", "Head_Neck_Model_OpenSim46.osim")
HYOID_OUT = os.path.join(REPO_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_KuoLigaments.osim")

AFF_DLL = os.path.join(REPO_ROOT, "proprioception_plugin", "build", "Release", "osimMillard12EqWithAff.dll")
if not os.path.exists(AFF_DLL):
    AFF_DLL = os.path.join(REPO_ROOT, "osimMillard12EqWithAff.dll")
CTRL_DLL = os.path.join(REPO_ROOT, "HeadNeckNeuralController_plugin", "build", "Release", "osimHeadNeckNeural.dll")
if not os.path.exists(CTRL_DLL):
    CTRL_DLL = os.path.join(REPO_ROOT, "osimHeadNeckNeural.dll")
FIX_DLL = os.path.join(REPO_ROOT, "FixationController_plugin", "build", "Release", "osimFixation.dll")

COORDINATE_FREE_CTRL_DEFAULTS = [
    ("desired_rotation_vec", "0 0 0"),
    ("desired_translation_vec", "0 0 0"),
    ("w_translation", "0.0"),
    ("urgency", "0.0"),
    ("trajectory_tau", "0.28"),
    ("fc_voluntary", "2.15"),
    ("omega_task", "7.2"),
    ("zeta_task", "0.95"),
    ("c_grav_task", "0.90"),
    ("c_musc_task", "0.0"),
    ("c_disc_task", "1.90"),
    ("ki_task", "2.20"),
    ("omega_shift", "0.85"),
    ("zeta_shift", "0.06"),
    ("ki_shift", "1.50"),
    ("c_ff_shift", "0.75"),
    ("k_trans_obs_pos", "0.0"),
    ("k_trans_obs_vel", "0.0"),
    ("k_shift_obs_pos", "0.0"),
    ("k_shift_obs_vel", "0.0"),
    ("shift_station_blend", "0.0"),
    ("tau_obs_washout", "4.0"),
    ("alpha_grav_ton", "0.40"),
    ("omega_ton", "5.5"),
    ("gamma_musc_ton", "0.0"),
    ("c_disc_ton", "0.25"),
    ("p_ton", "15.0"),
    ("ki_grav_ton", "0.50"),
    ("tau_ton_int", "4.0"),
    ("omega_sc", "18.0"),
    ("beta_musc_sc", "0.0"),
    ("c_disc_sc", "0.10"),
    ("c_grav_sc", "0.0"),
    ("z_sc", "0.012"),
    ("p_sc", "25.0"),
    ("c_omega_p_sc", "0.0"),
    ("alpha_inert_phas", "2.50"),
    ("gamma_musc_phas", "0.0"),
    ("c_disc_phas", "0.0"),
    ("z_phas", "0.015"),
    ("p_phas", "4.0"),
    ("oto_reweight_a0_mps2", "10.0"),
    ("oto_reweight_tau_s", "0.18"),
    ("phasic_oto_tanh_mps2", "16.0"),
    ("phasic_oto_sat_mps2", "0.0"),
    ("phasic_torque_sat_nm", "0.0"),
    ("phasic_lead_sat_nm", "0.0"),
    ("phasic_torque_tanh_nm", "0.0"),
    ("highg_oto_knee_mps2", "0.0"),
    ("highg_transition_mps2", "0.65"),
    ("highg_irr_knee_ratio", "1.90"),
    ("highg_phas_pole_boost", "0.0"),
    ("highg_phas_sust_floor", "0.0"),
    ("highg_p_sc_floor", "60.0"),
    ("highg_tau_lead_add_s", "0.0"),
    ("highg_sc_gain_boost", "0.0"),
    ("highg_brace_phas_suppress", "0.0"),
    ("vcr_compliant_share", "0.0"),
    ("kappa_ia_local", "0.008"),
    ("kappa_ia_synergy", "0.006"),
    ("c_disc_ccr", "0.16"),
    ("gamma_musc_ccr", "0.0"),
    ("c_grav_ccr", "0.0"),
    ("z_ccr", "0.012"),
    ("p_ccr", "85.0"),
    ("K_recip_inhib", "0.0"),
    ("K_gamma_dyn", "0.90"),
    ("K_gamma_stat", "0.70"),
    ("kappa_buckle", "0.25"),
    ("beta_buckle", "0.035"),
    ("tau_lead_pole", "0.012"),
    ("K_gto_ib", "0.00045"),
    ("p_gto_ib", "150.0"),
    ("thresh_gto_ib", "0.50"),
    ("K_gto_vol_suppress", "0.85"),
    ("K_renshaw", "0.16"),
    ("tau_renshaw", "0.022"),
    ("K_lig_protective", "0.09"),
    ("p_lig_reflex", "40.0"),
    ("thresh_spindle_ia", "0.50"),
    ("thresh_spindle_ii", "0.25"),
    ("thresh_vest_tilt", "0.002"),
    ("thresh_vest_scc", "0.005"),
    ("thresh_vest_oto", "0.005"),
    ("Kp_mahony", "0.25"),
    ("Ki_mahony", "0.008"),
    ("w_oto_mahony", "0.50"),
    ("vs_leak_rate", "0.05"),
    ("tau_vel_leak", "15.0"),
    ("tau_pos_leak", "5.0"),
    ("enable_ccr", "true"),
    ("enable_ccr_spinal", "true"),
    ("enable_ccr_task", "true"),
    ("enable_vcr", "true"),
    ("enable_vcr_tonic", "true"),
    ("enable_vcr_canal", "true"),
    ("enable_vcr_phasic", "true"),
    ("enable_task_controller", "true"),
    ("enable_task_integral", "true"),
    ("enable_decoder", "true"),
    ("enable_mahony", "true"),
    ("enable_efference_cancellation", "true"),
    ("enable_fusimotor", "true"),
    ("enable_reciprocal_inhibition", "true"),
    ("enable_antibuckling", "true"),
    ("enable_lead_compensators", "true"),
    ("enable_eye_afferents", "false"),
    ("enable_gto_reflex", "true"),
    ("enable_renshaw", "true"),
    ("enable_ligament_reflex", "true"),
    ("enable_sensor_noise", "false"),
    ("enable_motor_noise", "false"),
    ("noise_seed", "42"),
    ("noise_scale", "1.0"),
]

COMMON_BODIES = ["spine", "cerv7", "cerv6", "cerv5", "cerv4", "cerv3", "cerv2", "cerv1", "skull"]


def load_opensim():
    import opensim as osim

    osim.Logger.setLevelString("Error")
    if hasattr(osim, "ModelVisualizer"):
        osim.ModelVisualizer.addDirToGeometrySearchPaths(os.path.join(REPO_ROOT, "osim_files", "Geometry"))
    osim.LoadOpenSimLibraryExact(AFF_DLL)
    osim.LoadOpenSimLibraryExact(CTRL_DLL)
    osim.LoadOpenSimLibraryExact(FIX_DLL)
    return osim


def _read_out_double(comp, out_name, state):
    return float(comp.getOutput(out_name).getValueAsString(state))


def _vec3_to_np(v):
    return np.array([float(v.get(0)), float(v.get(1)), float(v.get(2))], dtype=float)


def _rot_to_np(R):
    return np.array([[float(R.get(r, c)) for c in range(3)] for r in range(3)], dtype=float)


def build_model(osim):
    # 1. Measure neutral (q = 0) rigid transforms of all 9 shared neck/head bodies in both models
    m_kuo = osim.Model(KUO_IN)
    s_kuo = m_kuo.initSystem()
    m_kuo.realizeVelocity(s_kuo)

    m_hyo = osim.Model(HYOID_IN)
    s_hyo = m_hyo.initSystem()
    m_hyo.realizePosition(s_hyo)

    kuo_tf = {}
    hyo_tf = {}
    max_rot_diff = 0.0
    print("=== Neutral Body-Frame Registration (HYOID - Kuo, mm) ===")
    for bn in COMMON_BODIES:
        tk = m_kuo.getBodySet().get(bn).getTransformInGround(s_kuo)
        th = m_hyo.getBodySet().get(bn).getTransformInGround(s_hyo)
        pk, Rk = _vec3_to_np(tk.p()), _rot_to_np(tk.R())
        ph, Rh = _vec3_to_np(th.p()), _rot_to_np(th.R())
        kuo_tf[bn] = (pk, Rk)
        hyo_tf[bn] = (ph, Rh)
        max_rot_diff = max(max_rot_diff, float(np.max(np.abs(Rh - Rk))))
        dp_mm = (ph - pk) * 1000.0
        print(f"  {bn:8s}: d_origin = ({dp_mm[0]:+8.4f}, {dp_mm[1]:+8.4f}, {dp_mm[2]:+8.4f}) mm")
    print(f"  max |R_HYOID - R_Kuo| at q=0: {max_rot_diff:.3e}")

    # Collect Kuo reference rest stretch, tension, and Ground path points for all non-AF ligaments
    kuo_fset = m_kuo.getForceSet()
    kuo_ref = {}
    for i in range(kuo_fset.getSize()):
        frc = kuo_fset.get(i)
        if frc.getConcreteClassName() != "DynamicLigamentForce":
            continue
        nm = frc.getName()
        if nm.startswith("AF_"):
            continue
        gp = osim.GeometryPath.safeDownCast(frc.getPropertyByName("GeometryPath").getValueAsObject())
        pps = gp.getPathPointSet()
        pts_G = []
        for k in range(pps.getSize()):
            pp = osim.PathPoint.safeDownCast(pps.get(k))
            pG = pp.getParentFrame().findStationLocationInGround(s_kuo, pp.getLocation(s_kuo))
            pts_G.append(_vec3_to_np(pG))
        kuo_ref[nm] = {
            "lam0": _read_out_double(frc, "strain", s_kuo),
            "f0": _read_out_double(frc, "tension", s_kuo),
            "pts_G": pts_G,
        }

    # 2. Parse XML trees and inject the 67 transformed DynamicLigamentForce elements into HYOID
    kuo_tree = ET.parse(KUO_IN)
    kuo_root = kuo_tree.getroot()

    hyo_tree = ET.parse(HYOID_IN)
    hyo_root = hyo_tree.getroot()
    hyo_model_el = hyo_root.find("./Model")
    hyo_model_el.set("name", "HYOID_HeadNeckNeural_Model_KuoLigaments")
    hyo_forces_objs = hyo_model_el.find("./ForceSet/objects")

    # Remove any pre-existing DynamicLigamentForce elements if ever re-run
    for old in list(hyo_forces_objs.findall("DynamicLigamentForce")):
        hyo_forces_objs.remove(old)

    grp_counts = Counter()
    imported_names = []
    skipped_af = []

    for dl_el in kuo_root.findall("./Model/ForceSet/objects/DynamicLigamentForce"):
        nm = dl_el.attrib["name"]
        if nm.startswith("AF_"):
            skipped_af.append(nm)
            continue

        new_el = copy.deepcopy(dl_el)
        for pp in new_el.findall("./GeometryPath/PathPointSet/objects/PathPoint"):
            sf = (pp.findtext("socket_parent_frame") or "").strip()
            bn = sf.split("/")[-1]
            if bn not in kuo_tf:
                raise RuntimeError(f"Ligament {nm} references unexpected body frame '{sf}'")
            loc_el = pp.find("location")
            p_kuo = np.array([float(x) for x in loc_el.text.split()], dtype=float)
            pk, Rk = kuo_tf[bn]
            ph, Rh = hyo_tf[bn]
            # Exact rigid transform: p_G = pk + Rk @ p_kuo; p_hyo = Rh^T @ (p_G - ph)
            p_G = pk + Rk @ p_kuo
            p_hyo = Rh.T @ (p_G - ph)
            loc_el.text = f"{p_hyo[0]:.8f} {p_hyo[1]:.8f} {p_hyo[2]:.8f}"

        hyo_forces_objs.append(new_el)
        grp_counts[nm.split("_")[0]] += 1
        imported_names.append(nm)

    # Apply approved B_k50_c30 disc-only bushing calibration (50% rotational stiffness,
    # 30% rotational damping) on all 8 intervertebral BushingForces so they represent
    # intervertebral discs without double-counting Kuo's 67 explicit ligaments.
    BUSH_K_SCALE = 0.50
    BUSH_C_SCALE = 0.30
    n_bush_cal = 0
    for bf_el in hyo_forces_objs.findall("BushingForce"):
        rk = [float(x) * BUSH_K_SCALE for x in bf_el.findtext("rotational_stiffness").split()]
        rc = [float(x) * BUSH_C_SCALE for x in bf_el.findtext("rotational_damping").split()]
        bf_el.find("rotational_stiffness").text = f"{rk[0]:.6f} {rk[1]:.6f} {rk[2]:.6f}"
        bf_el.find("rotational_damping").text = f"{rc[0]:.6f} {rc[1]:.6f} {rc[2]:.6f}"
        n_bush_cal += 1

    # Enable real Lin02 GTO (Ib) and Cui et al. (2008) SRS on the 72 neck muscles
    # (leaving the 12 extraocular muscles false).
    n_neck_gto_srs = 0
    for mus_el in hyo_forces_objs.findall("Millard12EqMuscleWithAfferents"):
        mnm = mus_el.attrib.get("name", "")
        is_eye = mnm.startswith("r_") or mnm.startswith("l_")
        val_str = "false" if is_eye else "true"
        srs_el = mus_el.find("enable_SRS")
        if srs_el is None:
            srs_el = ET.SubElement(mus_el, "enable_SRS")
        srs_el.text = val_str
        gto_el = mus_el.find("GTO")
        if gto_el is None:
            gto_el = mus_el.find("Lin02GolgiTendonOrgan")
        if gto_el is not None:
            eg_el = gto_el.find("enable_GTO")
            if eg_el is None:
                eg_el = ET.SubElement(gto_el, "enable_GTO")
            eg_el.text = val_str
        if not is_eye:
            n_neck_gto_srs += 1

    # Replace HYOID 1.0's copy-pasted dummy vertebral inertias (0.02 0.08 0.02 kg*m^2 on all 7 vertebrae,
    # 30x-186x too large and equal to the entire 4.3 kg skull) with Kuo (2019) anatomical vertebral inertias
    # scaled by each segment's HYOID/Kuo mass ratio (preserving HYOID masses & COMs so u0 gravity equilibrium
    # stays exact, with a 0.0007 kg*m^2 floor for stiff-integrator conditioning).
    kuo_body_props = {}
    for kb in kuo_root.findall("./Model/BodySet/objects/Body"):
        bn = kb.attrib.get("name", "")
        if bn.startswith("cerv"):
            km = float(kb.findtext("mass"))
            ki_txt = (kb.findtext("inertia") or "").split()
            if len(ki_txt) >= 3:
                k_ixx, k_iyy, k_izz = float(ki_txt[0]), float(ki_txt[1]), float(ki_txt[2])
            else:
                k_ixx = float(kb.findtext("inertia_xx"))
                k_iyy = float(kb.findtext("inertia_yy"))
                k_izz = float(kb.findtext("inertia_zz"))
            kuo_body_props[bn] = (km, k_ixx, k_iyy, k_izz)

    n_vert_inertia = 0
    for body_el in hyo_model_el.findall("./BodySet/objects/Body"):
        bn = body_el.attrib.get("name", "")
        if bn in kuo_body_props:
            hm = float(body_el.findtext("mass"))
            km, k_ixx, k_iyy, k_izz = kuo_body_props[bn]
            scale = hm / max(km, 1e-6)
            ixx = max(0.00070, k_ixx * scale)
            iyy = max(0.00090, k_iyy * scale)
            izz = max(0.00070, k_izz * scale)
            in_el = body_el.find("inertia")
            if in_el is not None:
                in_el.text = f"{ixx:.6f} {iyy:.6f} {izz:.6f} 0 0 0"
                n_vert_inertia += 1
                print(f"  Updated {bn} inertia (m={hm:.3f}/{km:.3f}, x{scale:.2f}): [{ixx:.5f}, {iyy:.5f}, {izz:.5f}]")
        if bn == "skull":
            for mesh_el in body_el.findall("./attached_geometry/Mesh"):
                if (mesh_el.findtext("mesh_file") or "").strip() == "skull.vtp":
                    app_el = mesh_el.find("Appearance")
                    if app_el is None:
                        app_el = ET.SubElement(mesh_el, "Appearance")
                    op_el = app_el.find("opacity")
                    if op_el is None:
                        op_el = ET.SubElement(app_el, "opacity")
                    op_el.text = "0.3"

    # Rewrite HeadNeckNeuralController XML tags to clean coordinate-free invariants (zero RPY properties)
    ctrl_el = hyo_model_el.find("./ControllerSet/objects/HeadNeckNeuralController")
    if ctrl_el is not None:
        keep_base = {"actuator_list", "socket_actuators", "isDisabled", "enabled"}
        for ch in list(ctrl_el):
            if ch.tag not in keep_base:
                ctrl_el.remove(ch)
        for pk, pv in COORDINATE_FREE_CTRL_DEFAULTS:
            sub = ET.SubElement(ctrl_el, pk)
            sub.text = pv

    ET.indent(hyo_tree, space="\t")
    hyo_tree.write(HYOID_OUT, encoding="utf-8", xml_declaration=True)
    print(f"\nWrote {HYOID_OUT}")
    print(f"  Calibrated {n_bush_cal} BushingForces to B_k50_c30 (rot_k*{BUSH_K_SCALE:.2f}, rot_c*{BUSH_C_SCALE:.2f}).")
    print(f"  Imported {len(imported_names)} DynamicLigamentForce elements across {len(grp_counts)} groups; excluded {len(skipped_af)} AF_ elements.")
    print("  Group counts:", dict(sorted(grp_counts.items())))
    return kuo_ref


def verify_model(osim, kuo_ref):
    print("\n=== Reload & Registration Verification ===")
    m = osim.Model(HYOID_OUT)
    s = m.initSystem()
    m.realizeVelocity(s)

    n_bodies = m.getBodySet().getSize()
    n_muscles = m.getMuscles().getSize()
    n_constraints = m.getConstraintSet().getSize()
    n_controllers = m.getControllerSet().getSize()

    fset = m.getForceSet()
    n_bush = 0
    n_lim = 0
    dl_list = []
    af_list = []
    for i in range(fset.getSize()):
        frc = fset.get(i)
        cls = frc.getConcreteClassName()
        if cls == "BushingForce":
            n_bush += 1
        elif cls == "CoordinateLimitForce":
            n_lim += 1
        elif cls == "DynamicLigamentForce":
            dl_list.append(frc)
            if frc.getName().startswith("AF_"):
                af_list.append(frc.getName())

    has_skull = m.getBodySet().contains("skull")
    has_jaw = m.getBodySet().contains("jaw")
    jaw_joint = m.getJointSet().get("jawjnt").getConcreteClassName() if m.getJointSet().contains("jawjnt") else "MISSING"

    ctrl = m.getControllerSet().get("VCR_CCR_Controller")
    lig_reflex_flag = ctrl.getPropertyByName("enable_ligament_reflex").toString()

    print(f"  Bodies={n_bodies} (skull={has_skull}, jaw={has_jaw}, jawjnt={jaw_joint})")
    print(f"  Muscles={n_muscles} | Bushings={n_bush} | LimitForces={n_lim} | Constraints={n_constraints}")
    print(f"  DynamicLigamentForce={len(dl_list)} (AF_={len(af_list)}) | Controllers={n_controllers} (enable_ligament_reflex={lig_reflex_flag})")

    assert n_bodies == 17 and has_skull and has_jaw, "Unexpected body set"
    assert n_muscles == 84, f"Expected 84 muscles, got {n_muscles}"
    assert n_bush == 8, f"Expected 8 bushings, got {n_bush}"
    assert n_lim == 6, f"Expected 6 limit forces, got {n_lim}"
    assert n_constraints == 18, f"Expected 18 coupler constraints, got {n_constraints}"
    assert len(dl_list) == 67 and len(af_list) == 0, f"Expected 67 non-AF ligaments, got {len(dl_list)} (AF={len(af_list)})"

    max_pt_err_m = 0.0
    max_lam_err = 0.0
    max_f0_err = 0.0
    non_unity = []

    for lg in dl_list:
        nm = lg.getName()
        ref = kuo_ref[nm]
        gp = osim.GeometryPath.safeDownCast(lg.getPropertyByName("GeometryPath").getValueAsObject())
        pps = gp.getPathPointSet()
        assert pps.getSize() == len(ref["pts_G"]), f"Point count mismatch for {nm}"
        for k in range(pps.getSize()):
            pp = osim.PathPoint.safeDownCast(pps.get(k))
            pG = _vec3_to_np(pp.getParentFrame().findStationLocationInGround(s, pp.getLocation(s)))
            max_pt_err_m = max(max_pt_err_m, float(np.linalg.norm(pG - ref["pts_G"][k])))
        lam0 = _read_out_double(lg, "strain", s)
        f0 = _read_out_double(lg, "tension", s)
        max_lam_err = max(max_lam_err, abs(lam0 - ref["lam0"]))
        max_f0_err = max(max_f0_err, abs(f0 - ref["f0"]))
        if abs(lam0 - 1.0) > 1e-4:
            non_unity.append((nm, round(lam0, 5), round(f0, 3)))

    print(f"  max Ground PathPoint error = {max_pt_err_m:.3e} m")
    print(f"  max |lam0_HYOID - lam0_Kuo| = {max_lam_err:.3e}")
    print(f"  max |F0_HYOID - F0_Kuo|     = {max_f0_err:.3e} N")
    print(f"  Preserved non-unity rest ligaments ({len(non_unity)}): {non_unity}")

    assert max_pt_err_m < 1e-6, f"PathPoint registration error too large: {max_pt_err_m}"
    assert max_lam_err < 1e-5, f"Rest strain error too large: {max_lam_err}"
    assert max_f0_err < 1e-3, f"Rest tension error too large: {max_f0_err}"

    # 3. 0.25 s forward-dynamics smoke simulation with both controllers active
    print("\n=== Forward-Dynamics Smoke Test (0.25 s, controllers ON, 1g) ===")
    m_sim = osim.Model(HYOID_OUT)
    s_sim = m_sim.initSystem()
    m_sim.equilibrateMuscles(s_sim)

    mgr = osim.Manager(m_sim)
    mgr.setIntegratorAccuracy(1e-4)
    mgr.initialize(s_sim)
    s_end = mgr.integrate(0.25)
    m_sim.realizeVelocity(s_end)

    cset = m_sim.getCoordinateSet()
    angles = {cn: math.degrees(float(cset.get(cn).getValue(s_end))) for cn in ("pitch1", "pitch2", "roll1", "roll2", "yaw1", "yaw2")}
    for cn, val in angles.items():
        assert math.isfinite(val), f"Non-finite coordinate {cn}={val}"

    fset_sim = m_sim.getForceSet()
    max_f_end = 0.0
    max_f_name = ""
    for i in range(fset_sim.getSize()):
        frc = fset_sim.get(i)
        if frc.getConcreteClassName() == "DynamicLigamentForce":
            ft = _read_out_double(frc, "tension", s_end)
            assert math.isfinite(ft), f"Non-finite tension on {frc.getName()}"
            if ft > max_f_end:
                max_f_end = ft
                max_f_name = frc.getName()

    print(f"  t=0.25s coords (deg): " + ", ".join(f"{k}={v:+.2f}" for k, v in angles.items()))
    print(f"  t=0.25s max ligament tension: {max_f_name} = {max_f_end:.3f} N")
    print("ALL CHECKS PASSED.")


def main():
    osim = load_opensim()
    kuo_ref = build_model(osim)
    verify_model(osim, kuo_ref)


if __name__ == "__main__":
    main()
