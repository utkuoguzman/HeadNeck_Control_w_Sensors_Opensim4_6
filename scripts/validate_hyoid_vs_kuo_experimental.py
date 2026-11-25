#!/usr/bin/env python3
"""
Stage 1 & Stage 2 Validation Against Kuo et al. (2019) Experimental Cohort (Subjects A–J).

Stage 1 — Exact OpenSim 4.6 vs. OpenSim 3.3 Port Parity (Subject A, 6 conditions):
  Upgrades `subjectA_scaled.osim` to OpenSim 4.6 without altering any parameters,
  replays all 6 conditions (Flexion/Lateral x Relaxed/Stiff/Football) using Kuo's
  exact static-optimization activation `.sto` files and external-force `.mot` files,
  and compares head kinematics `q(t)`, `u(t)`, and peak ligament tensions against
  Kuo's archived OpenSim 3.3 `Results/*.sto`.

Stage 2 — Combined HYOID_KuoLigaments vs. HYOID_Detailed vs. Kuo Subjects A–J Corridor:
  1. Extracts the 10-subject (A–J) mean +/- SD corridor across all 6 conditions.
  2. Drives `HYOID_HeadNeckNeural_Model_KuoLigaments.osim` (67 ligaments + 8 Bushings)
     and baseline `HYOID_HeadNeckNeural_Model_Detailed.osim` (no ligaments) with
     Subject A's exact external-force `.mot` waveforms (skull application point shifted
     by exact `-d_origin_skull = (-12.90, 0, 0) mm`) in two modes:
       - Mode 2A: Open-loop matched tone (Kuo's 72 shared muscle activations mapped
         onto HYOID, VCR/CCR controller OFF) -> isolates passive/mechanical differences
         for bushing/limit calibration.
       - Mode 2B: Closed-loop neural reflex (`VCR_CCR_Controller` ON) -> evaluates
         active vestibular + spindle reflexes + Kuo dynamic ligaments under real impacts.
"""

import glob
import json
import math
import os
import re
import sys
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor, as_completed

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
EXP_ROOT = os.path.join(REPO_ROOT, "Ligaments_model", "Experimental_data")
SUBJ_A_DIR = os.path.join(EXP_ROOT, "Subjects A-B", "Subject A")

HYOID_KUO = os.path.join(REPO_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_KuoLigaments.osim")
HYOID_DET = os.path.join(REPO_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")

AFF_DLL = os.path.join(REPO_ROOT, "osimMillard12EqWithAff.dll")
CTRL_DLL = os.path.join(REPO_ROOT, "osimHeadNeckNeural.dll")
FIX_DLL = os.path.join(REPO_ROOT, "FixationController_plugin", "build", "Release", "osimFixation.dll")

ARTIFACT_DIR = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
SCRATCH_DIR = os.path.join(ARTIFACT_DIR, "scratch")
OUT_JSON = os.path.join(SCRATCH_DIR, "kuo_experimental_validation.json")
OUT_PNG = os.path.join(ARTIFACT_DIR, "kuo_experimental_vs_hyoid.png")

CONDITIONS = [
    ("Flex_Relax",   "Flexion Relaxed Results",  "flexion_relaxed_force.mot",   "Relaxed_Activations_20_sternhyoid_partial.sto", "pitch", False),
    ("Flex_Stiff",   "Flexion Stiff Results",    "flexion_stiff_force.mot",     "Stiff_Activations_80_sternhyoid_partial.sto",   "pitch", False),
    ("Flex_Football","Flexion Football Results", "flexion_football_impact.mot", "Stiff_Activations_80_sternhyoid_partial.sto",   "pitch", False),
    ("Lat_Relax",    "Lateral Relaxed Results",  "lateral_relaxed_force.mot",   "Relaxed_Activations_20_sternhyoid_partial.sto", "roll",  True),
    ("Lat_Stiff",    "Lateral Stiff Results",    "lateral_stiff_force.mot",     "Stiff_Activations_80_sternhyoid_partial.sto",   "roll",  True),
    ("Lat_Football", "Lateral Football Results", "lateral_football_impact.mot", "Stiff_Activations_80_sternhyoid_partial.sto",   "roll",  True),
]

T_GRID = np.linspace(0.0, 0.30, 301)  # 1 ms common grid


def init_opensim():
    import opensim as osim
    osim.Logger.setLevelString("Error")
    if hasattr(osim, "ModelVisualizer"):
        osim.ModelVisualizer.addDirToGeometrySearchPaths(os.path.join(REPO_ROOT, "osim_files", "Geometry"))
        osim.ModelVisualizer.addDirToGeometrySearchPaths(os.path.join(REPO_ROOT, "Ligaments_model", "Geometry"))
    osim.LoadOpenSimLibraryExact(AFF_DLL)
    osim.LoadOpenSimLibraryExact(CTRL_DLL)
    osim.LoadOpenSimLibraryExact(FIX_DLL)
    return osim


def load_sto(path):
    with open(path, "r", errors="replace") as fh:
        lines = fh.readlines()
    idx = next(i for i, l in enumerate(lines) if "endheader" in l.lower())
    hdr = lines[idx + 1].split()
    data = np.loadtxt(lines[idx + 2:])
    if data.ndim == 1:
        data = data[None, :]
    return hdr, data


def extract_kuo_total_angle(hdr, data, axis):
    cols = [i for i, h in enumerate(hdr) if axis in h.lower() and not h.startswith("gnd")]
    return data[:, 0], data[:, cols].sum(axis=1)


def convert_kuo_33_to_46(src_osim, dst_osim):
    """Cleanly convert a Kuo OpenSim 3.3 .osim to OpenSim 4.6 preserving all 79 DynamicLigamentForce elements."""
    osim = init_opensim()
    with open(src_osim, "r", errors="replace") as f:
        txt = f.read()

    # Fix Kuo's two unclosed XML comments safely without touching valid `parent->child` comments
    txt = txt.replace("<!-- Same as Tectorial Membrane>", "<!-- Same as Tectorial Membrane -->")
    txt = txt.replace("<!--Same rate scale as TM>", "<!--Same rate scale as TM-->")

    root = ET.fromstring(txt)

    # Sanitize '/' and spaces in component name attributes
    for el in root.iter():
        if "name" in el.attrib:
            nm = el.attrib["name"]
            if "/" in nm or " " in nm:
                el.attrib["name"] = nm.replace("/", "_").replace(" ", "_")

    # Extract DynamicLigamentForce elements before passing skeleton+muscles through OpenSim 4.6 upgrade
    fobjs = root.find("./Model/ForceSet/objects")
    dl_elems = list(fobjs.findall("DynamicLigamentForce"))
    for el in dl_elems:
        fobjs.remove(el)

    tmp_nolig = dst_osim + ".tmp_nolig.osim"
    ET.ElementTree(root).write(tmp_nolig, encoding="utf-8", xml_declaration=True)

    # Upgrade skeleton + muscles to OpenSim 4.6 XML format
    m_up = osim.Model(tmp_nolig)
    m_up.finalizeFromProperties()
    m_up.printToXML(dst_osim)
    if os.path.exists(tmp_nolig):
        os.remove(tmp_nolig)

    # Re-inject all 79 DynamicLigamentForce elements converted to OpenSim 4.6 PathPoint syntax
    up_tree = ET.parse(dst_osim)
    up_fobjs = up_tree.getroot().find("./Model/ForceSet/objects")
    for dl in dl_elems:
        new_dl = ET.Element("DynamicLigamentForce", {"name": dl.attrib["name"]})
        gp_old = dl.find("GeometryPath")
        gp_new = ET.SubElement(new_dl, "GeometryPath", {"name": "path"})
        pps_new = ET.SubElement(gp_new, "PathPointSet")
        pps_objs = ET.SubElement(pps_new, "objects")
        for pp in gp_old.findall("./PathPointSet/objects/PathPoint"):
            pp_new = ET.SubElement(pps_objs, "PathPoint", {"name": pp.attrib["name"]})
            body_nm = (pp.findtext("body") or "").strip()
            sf = ET.SubElement(pp_new, "socket_parent_frame")
            sf.text = f"/bodyset/{body_nm}"
            loc = ET.SubElement(pp_new, "location")
            loc.text = (pp.findtext("location") or "0 0 0").strip()
        for tag in ("resting_length", "cross_sectional_area", "toe_strain", "toe_modulus",
                    "linear_modulus", "rate_scale", "rate_intercept"):
            val = dl.findtext(tag)
            if val is not None:
                c = ET.SubElement(new_dl, tag)
                c.text = val.strip()
        up_fobjs.append(new_dl)

    ET.indent(up_tree, space="\t")
    up_tree.write(dst_osim, encoding="utf-8", xml_declaration=True)
    return dst_osim


def make_shifted_mot(src_mot, dst_mot, delta_pos_m):
    """Copy a Kuo External Force .mot file and shift ext_force_pos in skull frame by delta_pos_m."""
    with open(src_mot, "r", errors="replace") as fh:
        lines = fh.readlines()
    idx = next(i for i, l in enumerate(lines) if "endheader" in l.lower())
    header = lines[:idx + 2]
    data = np.loadtxt(lines[idx + 2:])
    if data.ndim == 1:
        data = data[None, :]
    data[:, 4] += delta_pos_m[0]
    data[:, 5] += delta_pos_m[1]
    data[:, 6] += delta_pos_m[2]
    with open(dst_mot, "w") as out:
        out.writelines(header)
        for row in data:
            out.write("\t".join(f"{x:.6f}" for x in row) + "\n")
    return dst_mot


def attach_external_force(osim, model, mot_path):
    """Attach a Kuo external-force .mot via PrescribedForce on skull (force in ground, point in skull)."""
    _, data = load_sto(mot_path)
    pf = osim.PrescribedForce("Ext_Force", model.getBodySet().get("skull"))
    pf.setForceIsInGlobalFrame(True)
    pf.setPointIsInGlobalFrame(False)

    def make_pwl(col_idx):
        fn = osim.PiecewiseLinearFunction()
        for row in data:
            fn.addPoint(float(row[0]), float(row[col_idx]))
        return fn

    pf.setForceFunctions(make_pwl(1), make_pwl(2), make_pwl(3))
    pf.setPointFunctions(make_pwl(4), make_pwl(5), make_pwl(6))
    model.addForce(pf)
    return pf


def kuo_to_hyoid_muscle_name(kuo_nm):
    """Map Kuo muscle name (e.g. stern_mast_r / stern_mast_l) to HYOID name (stern_mast / stern_mast_L)."""
    if kuo_nm == "cleid_mas_r":
        return "cleid_mast"
    if kuo_nm == "cleid_mast_l":
        return "cleid_mast_L"
    if kuo_nm.endswith("_r"):
        return kuo_nm[:-2]
    if kuo_nm.endswith("_l"):
        return kuo_nm[:-2] + "_L"
    return kuo_nm


def extract_cohort_corridors():
    """Parse all 10 Subjects A..J across 6 conditions to build exact mean +/- SD corridors."""
    subjs = sorted(glob.glob(os.path.join(EXP_ROOT, "Subjects *", "Subject *")))
    corridors = {}
    for cname, res_dir, mot_file, _, axis, _ in CONDITIONS:
        q_mat = []
        u_mat = []
        peaks_deg = []
        t_peaks_s = []
        peaks_dps = []
        f_peaks_N = []
        subj_names = []

        for sdir in subjs:
            sname = os.path.basename(sdir)
            rpath = os.path.join(sdir, res_dir)
            qfiles = glob.glob(os.path.join(rpath, "*_Kinematics_q.sto"))
            ufiles = glob.glob(os.path.join(rpath, "*_Kinematics_u.sto"))
            mpath = os.path.join(sdir, "External Forces", mot_file)
            if not qfiles or not ufiles:
                continue
            qhdr, qd = load_sto(qfiles[0])
            uhdr, ud = load_sto(ufiles[0])
            tq, qtot = extract_kuo_total_angle(qhdr, qd, axis)
            tu, utot = extract_kuo_total_angle(uhdr, ud, axis)

            q_interp = np.interp(T_GRID, tq, qtot)
            u_interp = np.interp(T_GRID, tu, utot)
            q_mat.append(q_interp)
            u_mat.append(u_interp)

            ipk = int(np.argmax(np.abs(q_interp)))
            peaks_deg.append(float(q_interp[ipk]))
            t_peaks_s.append(float(T_GRID[ipk]))
            peaks_dps.append(float(np.max(np.abs(u_interp))))
            subj_names.append(sname)

            if os.path.exists(mpath):
                _, md = load_sto(mpath)
                f_peaks_N.append(float(np.max(np.linalg.norm(md[:, 1:4], axis=1))))

        q_arr = np.array(q_mat)
        u_arr = np.array(u_mat)
        corridors[cname] = {
            "n_subjects": len(subj_names),
            "subjects": subj_names,
            "axis": axis,
            "q_mean_deg": np.mean(q_arr, axis=0).tolist(),
            "q_sd_deg": np.std(q_arr, axis=0).tolist(),
            "q_min_deg": np.min(q_arr, axis=0).tolist(),
            "q_max_deg": np.max(q_arr, axis=0).tolist(),
            "peak_deg_mean": round(float(np.mean(peaks_deg)), 2),
            "peak_deg_sd": round(float(np.std(peaks_deg)), 2),
            "peak_deg_range": [round(float(np.min(peaks_deg)), 2), round(float(np.max(peaks_deg)), 2)],
            "t_peak_ms_mean": round(float(np.mean(t_peaks_s) * 1000.0), 1),
            "peak_dps_mean": round(float(np.mean(peaks_dps)), 1),
            "peak_dps_sd": round(float(np.std(peaks_dps)), 1),
            "ext_force_peak_N_mean": round(float(np.mean(f_peaks_N)), 1) if f_peaks_N else 0.0,
            "ext_force_peak_N_sd": round(float(np.std(f_peaks_N)), 1) if f_peaks_N else 0.0,
            "subjA_q_deg": q_arr[0].tolist(),
            "subjA_peak_deg": round(float(peaks_deg[0]), 2),
            "subjA_peak_dps": round(float(peaks_dps[0]), 1),
        }
    return corridors


def _stage1_worker(task):
    """Replay one Subject A condition on OpenSim 4.6 and compare against Kuo OpenSim 3.3 reference."""
    osim = init_opensim()
    cname = task["cname"]
    osim46_path = task["osim46_path"]
    mot_path = task["mot_path"]
    ctrl_sto = task["ctrl_sto"]
    ref_q_sto = task["ref_q_sto"]
    ref_u_sto = task["ref_u_sto"]
    ref_f_sto = task["ref_f_sto"]
    axis = task["axis"]
    lock_non_roll = task["lock_non_roll"]

    m = osim.Model(osim46_path)

    # Lock pitch/yaw for Lateral runs to match Kuo's exact planar lateral setup
    if lock_non_roll:
        cs = m.updCoordinateSet()
        for i in range(cs.getSize()):
            c = cs.get(i)
            if "roll" not in c.getName().lower():
                c.setDefaultLocked(True)

    # Apply Kuo's exact static-optimization constant activations
    chdr, cd = load_sto(ctrl_sto)
    act_map = {chdr[i].replace("/", "_").replace(" ", "_"): float(cd[0, i]) for i in range(1, len(chdr))}
    pctrl = osim.PrescribedController()
    pctrl.setName("KuoStaticOpt")
    mus_set = m.getMuscles()
    for i in range(mus_set.getSize()):
        mu = mus_set.get(i)
        pctrl.addActuator(mu)
        u_val = act_map.get(mu.getName(), 0.01)
        pctrl.prescribeControlForActuator(mu.getName(), osim.Constant(u_val))
    m.addController(pctrl)

    # Attach exact ExternalForce
    attach_external_force(osim, m, mot_path)

    state = m.initSystem()
    # Initialize muscle activations to match Kuo's pre-impact tonic activation
    for i in range(mus_set.getSize()):
        mu = osim.Millard2012EquilibriumMuscle.safeDownCast(mus_set.get(i))
        mu.setActivation(state, act_map.get(mu.getName(), 0.01))
    m.equilibrateMuscles(state)

    # Integrate in 1 ms steps to sample exact trajectory and peak ligament tensions
    mgr = osim.Manager(m)
    mgr.setIntegratorAccuracy(1e-5)
    mgr.initialize(state)

    cs = m.getCoordinateSet()
    ax_coords = [cs.get(i) for i in range(cs.getSize())
                 if axis in cs.get(i).getName().lower() and not cs.get(i).getName().startswith("gnd")]
    fset = m.getForceSet()
    dl_list = [fset.get(i) for i in range(fset.getSize()) if fset.get(i).getConcreteClassName() == "DynamicLigamentForce"]

    q_sim = np.zeros(len(T_GRID))
    u_sim = np.zeros(len(T_GRID))
    lig_peaks_sim = {lg.getName(): 0.0 for lg in dl_list}

    for k, t_tgt in enumerate(T_GRID):
        if k > 0:
            state = mgr.integrate(float(t_tgt))
        m.realizeVelocity(state)
        q_sim[k] = sum(math.degrees(float(c.getValue(state))) for c in ax_coords)
        u_sim[k] = sum(math.degrees(float(c.getSpeedValue(state))) for c in ax_coords)
        for lg in dl_list:
            ft = float(lg.getOutput("tension").getValueAsString(state))
            if ft > lig_peaks_sim[lg.getName()]:
                lig_peaks_sim[lg.getName()] = ft

    # Compare against Kuo OpenSim 3.3 reference
    rq_hdr, rq_d = load_sto(ref_q_sto)
    ru_hdr, ru_d = load_sto(ref_u_sto)
    rtq, rq = extract_kuo_total_angle(rq_hdr, rq_d, axis)
    rtu, ru = extract_kuo_total_angle(ru_hdr, ru_d, axis)
    rq_grid = np.interp(T_GRID, rtq, rq)
    ru_grid = np.interp(T_GRID, rtu, ru)

    rmse_deg = float(np.sqrt(np.mean((q_sim - rq_grid) ** 2)))
    max_err_deg = float(np.max(np.abs(q_sim - rq_grid)))
    pk_sim = float(q_sim[int(np.argmax(np.abs(q_sim)))])
    pk_ref = float(rq_grid[int(np.argmax(np.abs(rq_grid)))])
    pk_u_sim = float(np.max(np.abs(u_sim)))
    pk_u_ref = float(np.max(np.abs(ru_grid)))

    # Compare top ligament peak forces vs ForceReporter
    rf_hdr, rf_d = load_sto(ref_f_sto)
    ref_lig_peaks = {}
    for i, h in enumerate(rf_hdr):
        if h.endswith(".force"):
            nm = h[:-6]
            if nm in lig_peaks_sim:
                ref_lig_peaks[nm] = float(np.max(np.abs(rf_d[:, i])))
    top_ligs = sorted(ref_lig_peaks.items(), key=lambda x: -x[1])[:5]
    top_cmp = [
        {"name": nm, "ref_33_N": round(val, 1), "sim_46_N": round(lig_peaks_sim.get(nm, 0.0), 1)}
        for nm, val in top_ligs
    ]

    return {
        "cname": cname,
        "axis": axis,
        "q_sim_deg": q_sim.tolist(),
        "q_ref_deg": rq_grid.tolist(),
        "peak_sim_deg": round(pk_sim, 2),
        "peak_ref_deg": round(pk_ref, 2),
        "peak_diff_deg": round(pk_sim - pk_ref, 2),
        "rmse_deg": round(rmse_deg, 2),
        "max_err_deg": round(max_err_deg, 2),
        "peak_u_sim_dps": round(pk_u_sim, 1),
        "peak_u_ref_dps": round(pk_u_ref, 1),
        "top_lig_peaks": top_cmp,
    }


def _stage2_worker(task):
    """Run one HYOID model (KuoLigaments or Detailed) in Open-Loop or Closed-Loop on a Subject A impact."""
    osim = init_opensim()
    cname = task["cname"]
    model_tag = task["model_tag"]   # hyoid_kuo | hyoid_det
    ctrl_mode = task["ctrl_mode"]   # open_loop | closed_loop
    osim_path = task["osim_path"]
    mot_path = task["mot_path"]
    ctrl_sto = task["ctrl_sto"]
    axis = task["axis"]

    m = osim.Model(osim_path)

    if ctrl_mode == "open_loop":
        # Disable built-in neural & eye fixation controllers; apply Kuo's 72 shared neck muscle activations
        cset = m.updControllerSet()
        for i in range(cset.getSize()):
            cset.get(i).setEnabled(False)

        chdr, cd = load_sto(ctrl_sto)
        hyo_act = {}
        for i in range(1, len(chdr)):
            hnm = kuo_to_hyoid_muscle_name(chdr[i])
            hyo_act[hnm] = float(cd[0, i])

        pctrl = osim.PrescribedController()
        pctrl.setName("KuoMatchedOpenLoop")
        mus_set = m.getMuscles()
        n_matched = 0
        for i in range(mus_set.getSize()):
            mu = mus_set.get(i)
            pctrl.addActuator(mu)
            if mu.getName() in hyo_act:
                u_val = hyo_act[mu.getName()]
                n_matched += 1
            else:
                u_val = 0.01  # extraocular muscles at minimal tone
            pctrl.prescribeControlForActuator(mu.getName(), osim.Constant(u_val))
        m.addController(pctrl)
    else:
        n_matched = 0
        # Closed-loop: keep VCR_CCR_Controller ON, disable Binocular_Fixation_Controller for speed & clean neck focus
        cset = m.updControllerSet()
        for i in range(cset.getSize()):
            if cset.get(i).getName() == "Binocular_Fixation_Controller":
                cset.get(i).setEnabled(False)

    attach_external_force(osim, m, mot_path)

    state = m.initSystem()
    if ctrl_mode == "open_loop":
        mus_set = m.getMuscles()
        for i in range(mus_set.getSize()):
            mu = osim.Millard2012EquilibriumMuscle.safeDownCast(mus_set.get(i))
            if mu is not None:
                mu.setActivation(state, hyo_act.get(mu.getName(), 0.01))
    m.equilibrateMuscles(state)

    mgr = osim.Manager(m)
    mgr.setIntegratorAccuracy(1e-4)
    mgr.initialize(state)

    cs = m.getCoordinateSet()
    c1 = cs.get(f"{axis}1")
    c2 = cs.get(f"{axis}2")
    p1 = cs.get("pitch1")
    p2 = cs.get("pitch2")
    fset = m.getForceSet()
    dl_list = [fset.get(i) for i in range(fset.getSize()) if fset.get(i).getConcreteClassName() == "DynamicLigamentForce"]

    q_sim = np.zeros(len(T_GRID))
    u_sim = np.zeros(len(T_GRID))
    p_sim = np.zeros(len(T_GRID))
    max_lig_N = 0.0
    max_lig_nm = ""
    stop_err = None

    for k, t_tgt in enumerate(T_GRID):
        if k > 0:
            try:
                state = mgr.integrate(float(t_tgt))
            except Exception as e:
                stop_err = f"t={t_tgt:.3f}s: {e}"
                q_sim[k:] = q_sim[k - 1]
                u_sim[k:] = 0.0
                p_sim[k:] = p_sim[k - 1]
                break
        m.realizeVelocity(state)
        q_sim[k] = math.degrees(float(c1.getValue(state) + c2.getValue(state)))
        u_sim[k] = math.degrees(float(c1.getSpeedValue(state) + c2.getSpeedValue(state)))
        p_sim[k] = math.degrees(float(p1.getValue(state) + p2.getValue(state)))
        for lg in dl_list:
            ft = float(lg.getOutput("tension").getValueAsString(state))
            if ft > max_lig_N:
                max_lig_N = ft
                max_lig_nm = lg.getName()

    ipk = int(np.argmax(np.abs(q_sim)))
    return {
        "cname": cname,
        "model_tag": model_tag,
        "ctrl_mode": ctrl_mode,
        "axis": axis,
        "n_matched_muscles": n_matched,
        "q_sim_deg": q_sim.tolist(),
        "peak_deg": round(float(q_sim[ipk]), 2),
        "t_peak_ms": round(float(T_GRID[ipk] * 1000.0), 1),
        "peak_dps": round(float(np.max(np.abs(u_sim))), 1),
        "pitch_coupling_max_deg": round(float(np.max(np.abs(p_sim))), 2),
        "max_lig_N": round(max_lig_N, 1),
        "max_lig_name": max_lig_nm,
        "stop_err": stop_err,
    }


def make_plot(corridors, s1_map, s2_map):
    fig, axes = plt.subplots(2, 3, figsize=(16.5, 9.5), dpi=150, sharex=True)
    fig.suptitle("Kuo et al. (2019) Subjects A–J Experimental Cohort vs. OpenSim 4.6 Port & HYOID Combined Model",
                 fontsize=13, fontweight="bold")
    t_ms = T_GRID * 1000.0

    for idx, (cname, _, _, _, axis, _) in enumerate(CONDITIONS):
        ax = axes[idx // 3, idx % 3]
        cor = corridors[cname]
        qm = np.array(cor["q_mean_deg"])
        qs = np.array(cor["q_sd_deg"])
        ax.fill_between(t_ms, qm - qs, qm + qs, color="#9e9e9e", alpha=0.28, label="Kuo A–J Mean ± SD (n=10)")
        ax.plot(t_ms, qm, color="#555555", lw=1.2, ls=":", label="Kuo A–J Mean")

        # Stage 1 parity (Subject A 3.3 vs 4.6)
        s1 = s1_map[cname]
        ax.plot(t_ms, s1["q_ref_deg"], color="black", lw=1.8, ls="--", label=f"Subj A 3.3 Ref ({s1['peak_ref_deg']:+.1f}°)")
        ax.plot(t_ms, s1["q_sim_deg"], color="#2ca02c", lw=1.5, ls="-.", label=f"Subj A 4.6 Port ({s1['peak_sim_deg']:+.1f}°)")

        # Stage 2 HYOID comparisons
        for mtag, cmode, lbl, clr, ls, lw in [
            ("hyoid_kuo", "open_loop",   "HYOID+Kuo67 OpenLoop", "#1f77b4", "-",  2.0),
            ("hyoid_det", "open_loop",   "HYOID Base OpenLoop",  "#7fb3d5", "--", 1.5),
            ("hyoid_kuo", "closed_loop", "HYOID+Kuo67 VCR+CCR",  "#d62728", "-",  2.0),
            ("hyoid_det", "closed_loop", "HYOID Base VCR+CCR",   "#ff9896", "--", 1.5),
        ]:
            r = s2_map[(mtag, cmode, cname)]
            ax.plot(t_ms, r["q_sim_deg"], color=clr, ls=ls, lw=lw, label=f"{lbl} ({r['peak_deg']:+.1f}°)")

        ax.axhline(0, color="k", lw=0.5, alpha=0.5)
        ax.set_title(f"{cname.replace('_', ' ')} ({cor['ext_force_peak_N_mean']:.0f} N avg peak)")
        ax.set_ylabel("Total Head Pitch (deg)" if axis == "pitch" else "Total Head Roll (deg)")
        if idx >= 3:
            ax.set_xlabel("Time (ms)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=6.8, loc="best")

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(OUT_PNG, dpi=150)
    plt.close(fig)
    print(f"Saved validation comparison figure -> {OUT_PNG}")


def main():
    t0 = time.time()
    os.makedirs(SCRATCH_DIR, exist_ok=True)

    print("1) Extracting Kuo Subjects A–J (n=10) experimental/simulation corridors...")
    corridors = extract_cohort_corridors()
    for cname, cor in corridors.items():
        print(f"  {cname:14s}: n={cor['n_subjects']} | F_peak={cor['ext_force_peak_N_mean']:6.1f}±{cor['ext_force_peak_N_sd']:5.1f}N | "
              f"Peak={cor['peak_deg_mean']:+6.2f}±{cor['peak_deg_sd']:5.2f}° (SubjA={cor['subjA_peak_deg']:+6.2f}°) | "
              f"Vel={cor['peak_dps_mean']:5.1f}±{cor['peak_dps_sd']:4.1f}°/s")

    print("\n2) Upgrading Subject A model to OpenSim 4.6 & preparing shifted .mot inputs for HYOID...")
    subjA_osim46 = os.path.join(SCRATCH_DIR, "subjectA_scaled_osim46.osim")
    convert_kuo_33_to_46(os.path.join(SUBJ_A_DIR, "subjectA_scaled.osim"), subjA_osim46)

    # Compute exact skull origin shift between HYOID and Kuo's unscaled model (same skull.vtp geometry)
    osim = init_opensim()
    kuo_unscaled = os.path.join(REPO_ROOT, "Ligaments_model", "Head_Neck_Model_OpenSim46.osim")
    mK = osim.Model(kuo_unscaled); sK = mK.initSystem(); mK.realizePosition(sK)
    mH = osim.Model(HYOID_KUO);    sH = mH.initSystem(); mH.realizePosition(sH)
    pK = mK.getBodySet().get("skull").getPositionInGround(sK)
    pH = mH.getBodySet().get("skull").getPositionInGround(sH)
    d_skull = np.array([float(pH.get(i) - pK.get(i)) for i in range(3)], dtype=float)
    print(f"  Skull origin shift (HYOID - Kuo unscaled) = ({d_skull[0]*1000:+.3f}, {d_skull[1]*1000:+.3f}, {d_skull[2]*1000:+.3f}) mm")

    shifted_mots = {}
    for cname, _, mot_file, _, _, _ in CONDITIONS:
        src_m = os.path.join(SUBJ_A_DIR, "External Forces", mot_file)
        dst_m = os.path.join(SCRATCH_DIR, f"hyoid_shifted_{mot_file}")
        # Shift skull application point by -d_skull so p_G_HYOID = r_H + (p_A - d_skull) = r_A + p_A
        make_shifted_mot(src_m, dst_m, -d_skull)
        shifted_mots[cname] = dst_m

    # Build parallel task lists: 6 Stage-1 tasks + 24 Stage-2 tasks = 30 sims
    s1_tasks = []
    s2_tasks = []
    for cname, res_dir, mot_file, ctrl_file, axis, lock_non_roll in CONDITIONS:
        rpath = os.path.join(SUBJ_A_DIR, res_dir)
        s1_tasks.append({
            "cname": cname,
            "osim46_path": subjA_osim46,
            "mot_path": os.path.join(SUBJ_A_DIR, "External Forces", mot_file),
            "ctrl_sto": os.path.join(SUBJ_A_DIR, "Controllers", ctrl_file),
            "ref_q_sto": glob.glob(os.path.join(rpath, "*_Kinematics_q.sto"))[0],
            "ref_u_sto": glob.glob(os.path.join(rpath, "*_Kinematics_u.sto"))[0],
            "ref_f_sto": glob.glob(os.path.join(rpath, "*_ForceReporter_forces.sto"))[0],
            "axis": axis,
            "lock_non_roll": lock_non_roll,
        })
        for mtag, opath in (("hyoid_kuo", HYOID_KUO), ("hyoid_det", HYOID_DET)):
            for cmode in ("open_loop", "closed_loop"):
                s2_tasks.append({
                    "cname": cname,
                    "model_tag": mtag,
                    "ctrl_mode": cmode,
                    "osim_path": opath,
                    "mot_path": shifted_mots[cname],
                    "ctrl_sto": os.path.join(SUBJ_A_DIR, "Controllers", ctrl_file),
                    "axis": axis,
                })

    n_workers = min(14, max(4, (os.cpu_count() or 4) - 2))
    print(f"\n3) Dispatching {len(s1_tasks)} Stage-1 + {len(s2_tasks)} Stage-2 simulations across {n_workers} workers...")
    s1_map = {}
    s2_map = {}
    with ProcessPoolExecutor(max_workers=n_workers) as ex:
        futs = {}
        for t in s1_tasks:
            futs[ex.submit(_stage1_worker, t)] = ("s1", t["cname"])
        for t in s2_tasks:
            futs[ex.submit(_stage2_worker, t)] = ("s2", (t["model_tag"], t["ctrl_mode"], t["cname"]))
        for fut in as_completed(futs):
            kind, key = futs[fut]
            res = fut.result()
            if kind == "s1":
                s1_map[key] = res
            else:
                s2_map[key] = res

    print("\n=== STAGE 1: SUBJECT A OPENSIM 4.6 PORT vs. KUO OPENSIM 3.3 REFERENCE ===")
    for cname, _, _, _, _, _ in CONDITIONS:
        r = s1_map[cname]
        top_str = ", ".join(f"{x['name']}:{x['sim_46_N']}/{x['ref_33_N']}N" for x in r["top_lig_peaks"][:3])
        print(f"  {cname:14s}: Ref3.3={r['peak_ref_deg']:+6.2f}° | Sim4.6={r['peak_sim_deg']:+6.2f}° | "
              f"diff={r['peak_diff_deg']:+5.2f}° RMSE={r['rmse_deg']:4.2f}° | "
              f"Vel={r['peak_u_sim_dps']:5.1f}/{r['peak_u_ref_dps']:5.1f}°/s | TopLigs: {top_str}")

    print("\n=== STAGE 2: HYOID COMBINED vs. HYOID BASE vs. KUO A–J CORRIDOR ===")
    for cname, _, _, _, _, _ in CONDITIONS:
        cor = corridors[cname]
        ko = s2_map[("hyoid_kuo", "open_loop", cname)]
        do = s2_map[("hyoid_det", "open_loop", cname)]
        kc = s2_map[("hyoid_kuo", "closed_loop", cname)]
        dc = s2_map[("hyoid_det", "closed_loop", cname)]
        print(f"  {cname:14s} | Kuo A–J: {cor['peak_deg_mean']:+6.2f}±{cor['peak_deg_sd']:4.2f}° (SubjA {cor['subjA_peak_deg']:+6.2f}°) | "
              f"OpenLoop Kuo67={ko['peak_deg']:+6.2f}° (Base={do['peak_deg']:+6.2f}°, Fmax={ko['max_lig_N']:5.1f}N {ko['max_lig_name']}) | "
              f"VCR+CCR Kuo67={kc['peak_deg']:+6.2f}° (Base={dc['peak_deg']:+6.2f}°, Fmax={kc['max_lig_N']:5.1f}N)")

    make_plot(corridors, s1_map, s2_map)

    out_payload = {
        "wall_s": round(time.time() - t0, 2),
        "skull_shift_mm": [round(float(x * 1000.0), 4) for x in d_skull],
        "corridors": {k: {kk: vv for kk, vv in v.items() if not isinstance(vv, list) or kk in ("subjects", "peak_deg_range")}
                      for k, v in corridors.items()},
        "stage1": {k: {kk: vv for kk, vv in v.items() if not isinstance(vv, list) or kk == "top_lig_peaks"}
                   for k, v in s1_map.items()},
        "stage2": {f"{k[0]}_{k[1]}_{k[2]}": {kk: vv for kk, vv in v.items() if not isinstance(vv, list)}
                   for k, v in s2_map.items()},
    }
    with open(OUT_JSON, "w") as f:
        json.dump(out_payload, f, indent=2)
    print(f"\nCompleted Stage 1 + Stage 2 in {time.time() - t0:.2f}s -> {OUT_JSON}")


if __name__ == "__main__":
    main()
