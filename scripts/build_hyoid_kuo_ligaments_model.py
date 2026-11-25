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

AFF_DLL = os.path.join(REPO_ROOT, "osimMillard12EqWithAff.dll")
CTRL_DLL = os.path.join(REPO_ROOT, "osimHeadNeckNeural.dll")
FIX_DLL = os.path.join(REPO_ROOT, "FixationController_plugin", "build", "Release", "osimFixation.dll")

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

    # Match Kuo's translucent skull mesh (opacity=0.3) so internal vestibular canal
    # arrows/cylinders remain visible while keeping jaw.vtp opaque (opacity=1.0).
    for body_el in hyo_model_el.findall("./BodySet/objects/Body"):
        if body_el.attrib.get("name") == "skull":
            for mesh_el in body_el.findall("./attached_geometry/Mesh"):
                if (mesh_el.findtext("mesh_file") or "").strip() == "skull.vtp":
                    app_el = mesh_el.find("Appearance")
                    if app_el is None:
                        app_el = ET.SubElement(mesh_el, "Appearance")
                    op_el = app_el.find("opacity")
                    if op_el is None:
                        op_el = ET.SubElement(app_el, "opacity")
                    op_el.text = "0.3"

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
