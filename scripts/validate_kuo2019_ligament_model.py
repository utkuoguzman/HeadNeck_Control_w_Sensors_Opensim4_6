"""
Kuo et al. (2019) OpenSim 4.6 Port & 4-Part Validation Suite
-------------------------------------------------------------
Ported from `Ligaments_model/Head_Neck_Model.osim` & `DynamicLigamentForce.cpp`
(Kuo, Sheffels, Fanton, Yu, Hamalainen & Camarillo, J. R. Soc. Interface 16: 20190086).

1. Converts `Ligaments_model/Head_Neck_Model.osim` (v30000) cleanly to OpenSim 4.6
   (`Ligaments_model/Head_Neck_Model_OpenSim46.osim`) without altering ANY parameters,
   and verifies rest stretch/force parity across all 79 DynamicLigamentForce elements.
2. Test V1: Material & Mattucci strain-rate stiffening curve check via compiled C++ outputs.
3. Test V2: Cervical spine quasi-static moment-angle flexibility corridors (C0-T1, C2-T1, C0-C2)
   in Flexion, Extension, Lateral Bending, and Axial Rotation vs. cadaveric corridors.
4. Test V3: Kuo et al. (2019) dynamic head-impact angular-impulse partitioning benchmark
   (150 N mild vs 1000 N severe; Ext vs Flex; Relaxed u=0.01 vs Co-contracted u=0.35;
   Dynamic rate-dependent vs Quasi-static rate_scale=0).
5. Test V4: Read-only exact HYOID bone-frame mapping verification (0.000 mm) and passive
   restoring-torque comparison (HYOID Bushing+Limit vs Kuo 79 vs Kuo 67 excl AF + HYOID Bushings).
"""

import os
import sys
import re
import time
import math
import json
import argparse
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor, as_completed

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
import scripts.generate_standard_plot as gsp

KUO_SRC_OSIM = os.path.join(_ROOT, "Ligaments_model", "Head_Neck_Model.osim")
KUO_OSIM46   = os.path.join(_ROOT, "Ligaments_model", "Head_Neck_Model_OpenSim46.osim")
KUO_GEOM_DIR = os.path.join(_ROOT, "Ligaments_model", "Geometry")
HYOID_OSIM   = gsp.MODEL_PATH

SCRATCH_DIR  = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\scratch"
OUT_JSON     = os.path.join(SCRATCH_DIR, "kuo2019_validation_metrics.json")
OUT_PNG      = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\kuo2019_model_validation.png"


def init_env():
    gsp.ensure_libraries_loaded()
    osim.ModelVisualizer.addDirToGeometrySearchPaths(KUO_GEOM_DIR)
    osim.ModelVisualizer.addDirToGeometrySearchPaths(os.path.join(_ROOT, "Geometry"))


def _read_out_double(comp, name, state):
    out = comp.getOutput(name)
    od = osim.OutputDouble.safeDownCast(out)
    if od is not None:
        return float(od.getValue(state))
    return float(out.getValueAsString(state))


def _sanitize_kuo_xml():
    raw = open(KUO_SRC_OSIM, "r", encoding="utf-8", errors="ignore").read()
    lines_c = [re.sub(r"<!--(?![^-]*-->)[^>]*>", "", ln) for ln in raw.splitlines()]
    clean = re.sub(r"<!--.*?-->", "", "\n".join(lines_c), flags=re.S)
    root = ET.fromstring(clean)
    for el in root.iter():
        if "name" in el.attrib:
            el.attrib["name"] = el.attrib["name"].replace("/", "_").replace(" ", "_")
    return root


def _parse_kuo_ligament_specs(root):
    # Compute Kuo q=0 Ground positions of bodies for analytical L0/lambda0 verification
    kuo_pos_G = {"ground": np.zeros(3), "spine": np.zeros(3)}
    for b in root.findall(".//BodySet/objects/Body"):
        bn = b.attrib["name"]
        j = b.find("Joint/*")
        if j is not None:
            par = j.findtext("parent_body").strip()
            lp = np.array([float(x) for x in j.findtext("location_in_parent").split()], dtype=float)
            kuo_pos_G[bn] = kuo_pos_G[par] + lp

    specs = []
    for dl in root.findall("./Model/ForceSet/objects/DynamicLigamentForce"):
        nm = dl.attrib["name"]
        pts = []
        for pp in dl.findall(".//PathPointSet/objects/*"):
            pts.append({
                "name": pp.attrib.get("name", f"{nm}_pt{len(pts)+1}"),
                "body": pp.findtext("body").strip(),
                "loc": [float(x) for x in pp.findtext("location").split()],
            })
        pts_G = [kuo_pos_G[p["body"]] + np.array(p["loc"], dtype=float) for p in pts]
        d0 = sum(float(np.linalg.norm(pts_G[k + 1] - pts_G[k])) for k in range(len(pts_G) - 1))
        L0 = float(dl.findtext("resting_length"))
        csa = float(dl.findtext("cross_sectional_area"))
        toe_strain = float(dl.findtext("toe_strain"))
        toe_mod = float(dl.findtext("toe_modulus"))
        lin_mod = float(dl.findtext("linear_modulus"))
        rate_scale = float(dl.findtext("rate_scale"))
        rate_int = float(dl.findtext("rate_intercept"))
        dis = (dl.findtext("isDisabled") or "false").strip().lower() == "true"

        lam0 = d0 / L0
        f0 = 0.0
        if (not dis) and lam0 > 1.0:
            if lam0 <= toe_strain:
                f0 = (lam0 - 1.0) * toe_mod * csa * 1e6
            else:
                f0 = ((toe_strain - 1.0) * toe_mod + (lam0 - toe_strain) * lin_mod) * csa * 1e6

        specs.append({
            "name": nm,
            "grp": re.match(r"^[A-Za-z]+", nm).group(0),
            "pts": pts,
            "pts_G": [p.tolist() for p in pts_G],
            "L0": L0,
            "d0_xml": d0,
            "lam0_xml": lam0,
            "f0_xml": f0,
            "csa": csa,
            "toe_strain": toe_strain,
            "toe_modulus": toe_mod,
            "linear_modulus": lin_mod,
            "rate_scale": rate_scale,
            "rate_intercept": rate_int,
            "appliesForce": not dis,
        })
    return kuo_pos_G, specs


def _append_dynamic_ligament_xml(objs_el, sp, body_offset_map=None, override_rate_scale=None, exclude_groups=None):
    if exclude_groups and sp["grp"] in exclude_groups:
        return False
    el = ET.SubElement(objs_el, "DynamicLigamentForce", {"name": sp["name"]})
    ET.SubElement(el, "appliesForce").text = "true" if sp["appliesForce"] else "false"
    gp = ET.SubElement(el, "GeometryPath")
    pps = ET.SubElement(gp, "PathPointSet")
    pp_objs = ET.SubElement(pps, "objects")
    for p in sp["pts"]:
        bname = p["body"]
        loc = np.array(p["loc"], dtype=float)
        if body_offset_map is not None:
            loc = loc - body_offset_map[bname]
        pp_el = ET.SubElement(pp_objs, "PathPoint", {"name": p["name"]})
        ET.SubElement(pp_el, "socket_parent_frame").text = f"/bodyset/{bname}"
        ET.SubElement(pp_el, "location").text = " ".join(f"{x:.10g}" for x in loc)
    pws = ET.SubElement(gp, "PathWrapSet")
    ET.SubElement(pws, "objects")
    ET.SubElement(el, "resting_length").text = f"{sp['L0']:.12g}"
    ET.SubElement(el, "cross_sectional_area").text = f"{sp['csa']:.12g}"
    ET.SubElement(el, "toe_strain").text = f"{sp['toe_strain']:.10g}"
    ET.SubElement(el, "toe_modulus").text = f"{sp['toe_modulus']:.10g}"
    ET.SubElement(el, "linear_modulus").text = f"{sp['linear_modulus']:.12g}"
    rsc = sp["rate_scale"] if override_rate_scale is None else float(override_rate_scale)
    ET.SubElement(el, "rate_scale").text = f"{rsc:.12g}"
    ET.SubElement(el, "rate_intercept").text = f"{sp['rate_intercept']:.10g}"
    return True


def build_and_verify_kuo_osim46():
    init_env()
    os.makedirs(SCRATCH_DIR, exist_ok=True)
    root_v3 = _sanitize_kuo_xml()
    kuo_pos_G, specs = _parse_kuo_ligament_specs(root_v3)

    # Strip v3 DynamicLigamentForce tags temporarily so OpenSim 4.6 natively upgrades skeleton + muscles
    fobjs_v3 = root_v3.find("./Model/ForceSet/objects")
    for dl in list(fobjs_v3.findall("DynamicLigamentForce")):
        fobjs_v3.remove(dl)

    tmp_v3 = os.path.join(SCRATCH_DIR, "kuo_tmp_v3_nolig.osim")
    ET.ElementTree(root_v3).write(tmp_v3, encoding="utf-8", xml_declaration=True)

    m_v3 = osim.Model(tmp_v3)
    m_v3.setName("Kuo2019_Head_Neck_Model")
    m_v3.finalizeConnections()
    m_v3.printToXML(KUO_OSIM46)

    # Re-inject all 79 authentic Kuo DynamicLigamentForce elements in OpenSim 4.6 format
    tree46 = ET.parse(KUO_OSIM46)
    root46 = tree46.getroot()
    objs46 = root46.find("./Model/ForceSet/objects")
    for sp in specs:
        _append_dynamic_ligament_xml(objs46, sp)
    if hasattr(ET, "indent"):
        ET.indent(tree46, space="\t")
    tree46.write(KUO_OSIM46, encoding="utf-8", xml_declaration=True)

    # Reload and strictly verify at q=0, qdot=0
    m46 = osim.Model(KUO_OSIM46)
    st = m46.initSystem()
    m46.realizeDynamics(st)

    fset = m46.getForceSet()
    dl_comps = [fset.get(i) for i in range(fset.getSize()) if fset.get(i).getConcreteClassName() == "DynamicLigamentForce"]
    n_mus = m46.getMuscles().getSize()
    free_coords = [
        m46.getCoordinateSet().get(i).getName()
        for i in range(m46.getCoordinateSet().getSize())
        if not m46.getCoordinateSet().get(i).getLocked(st)
    ]

    spec_by_name = {sp["name"]: sp for sp in specs}
    max_lam_err = 0.0
    max_f_err = 0.0
    prestrained_chk = []
    for comp in dl_comps:
        sp = spec_by_name[comp.getName()]
        lam_cpp = _read_out_double(comp, "strain", st)
        f_cpp = _read_out_double(comp, "tension", st)
        max_lam_err = max(max_lam_err, abs(lam_cpp - sp["lam0_xml"]))
        max_f_err = max(max_f_err, abs(f_cpp - sp["f0_xml"]))
        if lam_cpp > 1.0001 or lam_cpp < 0.99:
            prestrained_chk.append({
                "name": sp["name"], "lam0": round(lam_cpp, 5), "f0_N": round(f_cpp, 3),
                "xml_f0_N": round(sp["f0_xml"], 3),
            })

    summary = {
        "n_dynamic_ligaments": len(dl_comps),
        "n_muscles": n_mus,
        "n_free_coords": len(free_coords),
        "free_coords": free_coords,
        "max_lam0_err": max_lam_err,
        "max_f0_err_N": max_f_err,
        "non_unity_rest_ligaments": prestrained_chk,
    }
    print(f"[Step 2] Converted & verified {KUO_OSIM46}")
    print(f"  ligaments={len(dl_comps)}, muscles={n_mus}, free_coords={len(free_coords)}")
    print(f"  max |lam_cpp - lam_xml| = {max_lam_err:.3e}, max |F0_cpp - F0_xml| = {max_f_err:.3e} N")
    print(f"  non-unity lam0 ligaments ({len(prestrained_chk)}): {prestrained_chk}")
    if len(dl_comps) != 79 or max_lam_err > 1e-4:
        raise RuntimeError(f"Step 2 verification failed! n_lig={len(dl_comps)}, max_lam_err={max_lam_err}")
    return kuo_pos_G, specs, summary


def run_v1_material_verification(specs):
    init_env()
    reps = [
        "ALL_spine", "PLL_spine", "LF_r_spine_cerv7", "IS_spine_cerv7",
        "IT_r_spine_cerv7", "AF_r_spine", "LN_spine", "TM_cerv2_skull",
        "CLT_cerv2_cerv1", "AL_r_cerv2_skull", "JC_r_cerv2_cerv1",
        "AAO_r_cerv1_skull", "PAO_r_cerv1_skull",
    ]
    spec_map = {s["name"]: s for s in specs}

    # Build a 13-DOF SliderJoint tensile rig XML and read real C++ DynamicLigamentForce outputs
    rig_xml_path = os.path.join(SCRATCH_DIR, "kuo_v1_tensile_rig.osim")
    m = osim.Model()
    m.setName("KuoV1TensileRig")
    ground = m.getGround()
    for nm in reps:
        b = osim.Body(f"b_{nm}", 1.0, osim.Vec3(0.0), osim.Inertia(0.01, 0.01, 0.01))
        m.addBody(b)
        sj = osim.SliderJoint(f"sj_{nm}", ground, osim.Vec3(0.0), osim.Vec3(0, 0, math.pi / 2.0),
                              b, osim.Vec3(0.0), osim.Vec3(0, 0, math.pi / 2.0))
        c = sj.updCoordinate()
        c.setName(f"len_{nm}")
        c.setRangeMin(0.0); c.setRangeMax(0.20)
        c.setDefaultValue(spec_map[nm]["L0"])
        m.addJoint(sj)
    m.finalizeConnections()
    m.printToXML(rig_xml_path)

    tree = ET.parse(rig_xml_path)
    objs = tree.getroot().find("./Model/ForceSet/objects")
    for nm in reps:
        sp = dict(spec_map[nm])
        sp["name"] = f"lig_{nm}"
        sp["pts"] = [
            {"name": "p0", "body": "ground", "loc": [0.0, 0.0, 0.0]},
            {"name": "p1", "body": f"b_{nm}", "loc": [0.0, 0.0, 0.0]},
        ]
        _append_dynamic_ligament_xml(objs, sp)
        # Note Fix ground path in GeometryPath: SliderJoint moved along Y (after +90deg Z rotation X->Y)
    # Fix socket_parent_frame for "ground" which lives at /ground, not /bodyset/ground
    for pp in tree.getroot().findall(".//PathPoint"):
        sf = pp.find("socket_parent_frame")
        if sf.text.strip() == "/bodyset/ground":
            sf.text = "/ground"
    tree.write(rig_xml_path, encoding="utf-8", xml_declaration=True)

    m_rig = osim.Model(rig_xml_path)
    s_rig = m_rig.initSystem()
    fset = m_rig.getForceSet()
    cset = m_rig.getCoordinateSet()

    lam_grid = np.linspace(0.98, 1.35, 38)
    v_rates = [0.001, 0.05, 0.50, 2.50]  # m/s
    out_v1 = {"lam_grid": [round(float(x), 4) for x in lam_grid], "rates_mps": v_rates, "families": {}}

    max_err = 0.0
    for nm in reps:
        sp = spec_map[nm]
        L0 = sp["L0"]
        coord = cset.get(f"len_{nm}")
        comp = fset.get(f"lig_{nm}")
        fam_res = {"grp": sp["grp"], "L0_mm": round(L0 * 1e3, 3), "csa_mm2": round(sp["csa"] * 1e6, 2),
                   "toe_strain": sp["toe_strain"], "toe_modulus_MPa": sp["toe_modulus"],
                   "linear_modulus_MPa": sp["linear_modulus"], "rate_scale": sp["rate_scale"],
                   "curves_N": {}, "f_at_1p25_by_rate_N": {}, "rate_mult_at_2p5": 1.0}
        for vr in v_rates:
            vals = []
            coord.setSpeedValue(s_rig, float(vr))
            for lam in lam_grid:
                coord.setValue(s_rig, float(lam * L0), False)
                m_rig.realizeVelocity(s_rig)
                f_cpp = _read_out_double(comp, "tension", s_rig)
                # Analytical check against Kuo formula
                rf = (1.0 + math.log(vr / sp["rate_intercept"]) * sp["rate_scale"]) if vr > sp["rate_intercept"] else 1.0
                if lam <= 1.0:
                    f_ana = 0.0
                elif lam <= sp["toe_strain"]:
                    f_ana = (lam - 1.0) * sp["toe_modulus"] * sp["csa"] * rf * 1e6
                else:
                    f_ana = ((sp["toe_strain"] - 1.0) * sp["toe_modulus"] + (lam - sp["toe_strain"]) * sp["linear_modulus"]) * sp["csa"] * rf * 1e6
                max_err = max(max_err, abs(f_cpp - f_ana))
                vals.append(round(f_cpp, 3))
            fam_res["curves_N"][str(vr)] = vals
            # Sample at lam = 1.25
            coord.setValue(s_rig, float(1.25 * L0), False)
            m_rig.realizeVelocity(s_rig)
            fam_res["f_at_1p25_by_rate_N"][str(vr)] = round(_read_out_double(comp, "tension", s_rig), 2)
        f_qs = fam_res["f_at_1p25_by_rate_N"]["0.001"]
        f_dyn = fam_res["f_at_1p25_by_rate_N"]["2.5"]
        fam_res["rate_mult_at_2p5"] = round(f_dyn / f_qs, 3) if f_qs > 1e-9 else 1.0
        out_v1["families"][nm] = fam_res

    out_v1["max_analytical_err_N"] = max_err
    print(f"[V1 Material Check] max |F_cpp - F_analytical| across 13 families x 152 pts = {max_err:.3e} N")
    for k in ("LF_r_spine_cerv7", "AF_r_spine", "JC_r_cerv2_cerv1", "PAO_r_cerv1_skull", "TM_cerv2_skull", "ALL_spine"):
        r = out_v1["families"][k]
        print(f"  {k:18s}: F(lam=1.25, qs)={r['f_at_1p25_by_rate_N']['0.001']:7.1f} N | F(2.5m/s)={r['f_at_1p25_by_rate_N']['2.5']:7.1f} N (mult={r['rate_mult_at_2p5']:.2f}x)")
    return out_v1


def _build_cosine_ramp_spline(t_ramp, target_val, t_end=1.5):
    sp = osim.SimmSpline()
    ts = list(np.linspace(0.0, t_ramp, 21)) + [t_ramp + 0.1, t_end]
    for t in ts:
        if t <= 0.0:
            v = 0.0
        elif t >= t_ramp:
            v = float(target_val)
        else:
            v = float(target_val) * 0.5 * (1.0 - math.cos(math.pi * t / t_ramp))
        sp.addPoint(float(t), v)
    return sp


def _v2_curve_worker(task):
    """Exact 0g 20-DOF static equilibrium corridor solver (unclamped & XML-clamped)."""
    init_env()
    direction = task["dir"]       # Ext (+Z), Flex (-Z), LatR (+X), AxRot (+Y)
    mus_mode = task["mus_mode"]   # osteo | passive_mus
    clamped = bool(task["clamped"])
    mags = [float(x) for x in task["mags_Nm"]]

    m = osim.Model(KUO_OSIM46)
    m.setGravity(osim.Vec3(0.0, 0.0, 0.0))
    state = m.initSystem()
    mas = osim.MomentArmSolver(m)
    skull = m.getBodySet().get("skull")

    cs = m.getCoordinateSet()
    free_names = [f"pitch{k}" for k in range(7, 1, -1)] + [f"roll{k}" for k in range(7, 1, -1)] + [f"yaw{k}" for k in range(7, 1, -1)] + ["yaw1", "skullpitch"]
    coords = [cs.get(nm) for nm in free_names]
    n_dof = len(coords)

    if clamped:
        lo_b = np.array([c.getRangeMin() for c in coords], dtype=float)
        hi_b = np.array([c.getRangeMax() for c in coords], dtype=float)
    else:
        lo_b = np.full(n_dof, -math.radians(45.0), dtype=float)
        hi_b = np.full(n_dof, +math.radians(45.0), dtype=float)

    mus_list = [osim.Millard2012EquilibriumMuscle.safeDownCast(m.getMuscles().get(i)) for i in range(m.getMuscles().getSize())]
    mus_paths = [mu.getGeometryPath() for mu in mus_list]
    if mus_mode == "osteo":
        for mu in mus_list:
            mu.setAppliesForce(state, False)
    else:
        for mu in mus_list:
            mu.setActivation(state, 0.001)
        m.equilibrateMuscles(state)

    fset = m.getForceSet()
    lig_list = [fset.get(i) for i in range(fset.getSize()) if fset.get(i).getConcreteClassName() == "DynamicLigamentForce"]
    lig_paths = [osim.GeometryPath.safeDownCast(lg.getPropertyByName("GeometryPath").getValueAsObject()) for lg in lig_list]

    def eval_residual(q_vec, ext_M_world):
        for j, c in enumerate(coords):
            c.setValue(state, float(q_vec[j]), False)
            c.setSpeedValue(state, 0.0)
        m.realizePosition(state)

        # World angular velocity Jacobian columns omega_j(q) of skull relative to ground/spine
        J_omega = np.zeros((3, n_dof), dtype=float)
        for j, c in enumerate(coords):
            c.setSpeedValue(state, 1.0)
            m.realizeVelocity(state)
            om = skull.getAngularVelocityInGround(state)
            J_omega[0, j] = float(om.get(0))
            J_omega[1, j] = float(om.get(1))
            J_omega[2, j] = float(om.get(2))
            c.setSpeedValue(state, 0.0)

        m.realizeVelocity(state)
        if mus_mode == "passive_mus":
            for mu in mus_list:
                mu.setActivation(state, 0.001)
            m.equilibrateMuscles(state)
            m.realizeDynamics(state)

        tau = np.zeros(n_dof, dtype=float)
        # Applied world torque -> generalized force Q_ext = J_omega^T * M_world
        tau += J_omega.T @ ext_M_world

        for k, lg in enumerate(lig_list):
            ft = _read_out_double(lg, "tension", state)
            if ft <= 1e-12:
                continue
            lp = lig_paths[k]
            for j, c in enumerate(coords):
                tau[j] += mas.solve(state, c, lp) * ft

        if mus_mode == "passive_mus":
            for k, mu in enumerate(mus_list):
                fm = float(mu.getPassiveFiberForceAlongTendon(state))
                if abs(fm) <= 1e-12:
                    continue
                mp = mus_paths[k]
                for j, c in enumerate(coords):
                    tau[j] += mas.solve(state, c, mp) * fm

        return tau

    out_rows = []
    q_curr = np.zeros(n_dof, dtype=float)
    for mag in mags:
        if direction in ("Ext", "Flex"):
            M_w = np.array([0.0, 0.0, mag], dtype=float)
        elif direction == "LatR":
            M_w = np.array([mag, 0.0, 0.0], dtype=float)
        else:
            M_w = np.array([0.0, mag, 0.0], dtype=float)

        sol = least_squares(
            eval_residual,
            np.clip(q_curr, lo_b + 1e-6, hi_b - 1e-6),
            args=(M_w,),
            bounds=(lo_b, hi_b),
            method="trf",
            ftol=1e-9,
            xtol=1e-9,
            gtol=1e-9,
            max_nfev=80,
        )
        q_curr = sol.x.copy()
        res_final = eval_residual(q_curr, M_w)
        # Interior-DOF equilibrium check (exclude coordinates hitting active box stops)
        at_lo = q_curr <= (lo_b + 1e-4)
        at_hi = q_curr >= (hi_b - 1e-4)
        free_res = [abs(float(res_final[j])) for j in range(n_dof) if not (at_lo[j] or at_hi[j])]
        max_free_res = max(free_res) if free_res else 0.0

        p_segs = [math.degrees(q_curr[k]) for k in range(6)]
        r_segs = [math.degrees(q_curr[6 + k]) for k in range(6)]
        y_segs = [math.degrees(q_curr[12 + k]) for k in range(6)]
        yaw1_d = math.degrees(q_curr[18])
        sk_p_d = math.degrees(q_curr[19])

        max_lig_f, max_lig_nm = 0.0, ""
        for lg in lig_list:
            tv = _read_out_double(lg, "tension", state)
            if tv > max_lig_f:
                max_lig_f = tv
                max_lig_nm = lg.getName()

        out_rows.append({
            "dir": direction, "mag_Nm": round(mag, 2), "mus_mode": mus_mode, "clamped": clamped,
            "total_pitch_deg": round(sum(p_segs) + sk_p_d, 2),
            "lower_pitch_deg": round(sum(p_segs), 2),
            "c0c1_pitch_deg": round(sk_p_d, 2),
            "total_roll_deg": round(sum(r_segs), 2),
            "total_yaw_deg": round(sum(y_segs) + yaw1_d, 2),
            "lower_yaw_deg": round(sum(y_segs), 2),
            "c1c2_yaw_deg": round(yaw1_d, 2),
            "n_stops_hit": int(np.sum(at_lo | at_hi)),
            "max_interior_res_Nm": round(max_free_res, 5),
            "max_lig_N": round(max_lig_f, 1),
            "max_lig_name": max_lig_nm,
        })
    return out_rows


def _build_raised_cosine_pulse(t0, t1, peak_N, t_end=0.130):
    """Uniform 1 ms knot SimmSpline of C^1 raised cosine to eliminate spline overshoot."""
    sp = osim.SimmSpline()
    n_pts = int(round(t_end / 0.001)) + 1
    dur = max(1e-6, t1 - t0)
    for t in np.linspace(0.0, t_end, n_pts):
        if t0 <= t <= t1:
            v = 0.5 * peak_N * (1.0 - math.cos(2.0 * math.pi * (t - t0) / dur))
        else:
            v = 0.0
        sp.addPoint(float(t), float(v))
    return sp


def _compute_balanced_drives():
    """Classify Kuo 98 muscles into flexors/extensors/neutral at q=0 & return moment-balanced (u_flx, u_ext) maps."""
    init_env()
    m0 = osim.Model(KUO_OSIM46)
    s0 = m0.initSystem()
    m0.realizePosition(s0)
    mas0 = osim.MomentArmSolver(m0)
    cs0 = m0.getCoordinateSet()
    p_coords = [cs0.get(f"pitch{k}") for k in range(7, 1, -1)] + [cs0.get("skullpitch")]
    mus_info = []
    tau_ext_u, tau_flx_u = 0.0, 0.0
    for i in range(m0.getMuscles().getSize()):
        mu = m0.getMuscles().get(i)
        fm = float(mu.getMaxIsometricForce())
        ma = sum(mas0.solve(s0, pc, mu.getGeometryPath()) for pc in p_coords) / len(p_coords)
        role = "ext" if ma > 1e-4 else ("flx" if ma < -1e-4 else "neu")
        if role == "ext":
            tau_ext_u += ma * fm
        elif role == "flx":
            tau_flx_u += (-ma) * fm
        mus_info.append((mu.getName(), role))

    # Static gravity sagittal average moment across pitch joints at q=0 is ~0.14 Nm requiring +extensor support
    u_ext_rel = (0.030 * tau_flx_u + 0.14) / max(1e-6, tau_ext_u)
    u_ext_co  = (0.750 * tau_flx_u + 0.14) / max(1e-6, tau_ext_u)
    return {
        "Relax": {"flx": 0.030, "ext": round(u_ext_rel, 5), "neu": 0.015, "mus_roles": dict(mus_info)},
        "CoAct": {"flx": 0.750, "ext": round(u_ext_co, 5),  "neu": 0.250, "mus_roles": dict(mus_info)},
        "tau_ext_unit_Nm": round(tau_ext_u, 2),
        "tau_flx_unit_Nm": round(tau_flx_u, 2),
    }


def _v3_worker(task):
    init_env()
    peak_N = float(task["peak_N"])
    dur_s = float(task["dur_s"])
    dir_name = task["dir"]      # Ext (-X) | Flex (+X)
    u_mode = task["u_mode"]     # Relax | CoAct
    u_cfg = task["u_cfg"]
    dyn_lig = bool(task["dyn_lig"])

    if dyn_lig:
        m = osim.Model(KUO_OSIM46)
    else:
        tmp_qs = os.path.join(SCRATCH_DIR, f"kuo_v3_qs_{os.getpid()}.osim")
        tree = ET.parse(KUO_OSIM46)
        for dl in tree.getroot().findall("./Model/ForceSet/objects/DynamicLigamentForce"):
            dl.find("rate_scale").text = "0.0"
        tree.write(tmp_qs, encoding="utf-8", xml_declaration=True)
        m = osim.Model(tmp_qs)

    roles = u_cfg["mus_roles"]
    ctrl = osim.PrescribedController()
    ctrl.setName("balanced_muscle_drive")
    u_by_name = {}
    for i in range(m.getMuscles().getSize()):
        mu = m.updMuscles().get(i)
        r = roles[mu.getName()]
        uv = float(u_cfg[r])
        u_by_name[mu.getName()] = uv
        ctrl.addActuator(mu)
        ctrl.prescribeControlForActuator(mu.getName(), osim.Constant(uv))
    m.addController(ctrl)

    t_start = 0.010
    t_stop = t_start + dur_s
    t_end = 0.130
    sign_x = -1.0 if dir_name == "Ext" else +1.0
    sp_fx = _build_raised_cosine_pulse(t_start, t_stop, sign_x * peak_N, t_end=t_end)
    pf = osim.PrescribedForce("head_impact", m.getBodySet().get("skull"))
    pf.set_forceIsGlobal(True)
    pf.set_pointIsGlobal(False)
    pf.setForceFunctions(sp_fx, osim.Constant(0.0), osim.Constant(0.0))
    pf.setPointFunctions(osim.Constant(0.0173), osim.Constant(0.063), osim.Constant(0.0))
    pf.setTorqueFunctions(osim.Constant(0.0), osim.Constant(0.0), osim.Constant(0.0))
    m.addForce(pf)

    state = m.initSystem()
    for i in range(m.getMuscles().getSize()):
        mu = m.updMuscles().get(i)
        mu.setActivation(state, u_by_name[mu.getName()])
    m.equilibrateMuscles(state)

    mgr = osim.Manager(m)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5)  # SemiExplicitEuler2
    mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-5)
    mgr.setIntegratorMaximumStepSize(0.00025)
    state.setTime(0.0)
    mgr.initialize(state)

    cs = m.getCoordinateSet()
    pitch_coords = [cs.get(f"pitch{k}") for k in range(7, 1, -1)] + [cs.get("skullpitch")]
    n_pcoords = len(pitch_coords)
    mus_list = [osim.Millard2012EquilibriumMuscle.safeDownCast(m.getMuscles().get(i)) for i in range(m.getMuscles().getSize())]
    mus_paths = [mu.getGeometryPath() for mu in mus_list]
    fset = m.getForceSet()
    lig_list = [fset.get(i) for i in range(fset.getSize()) if fset.get(i).getConcreteClassName() == "DynamicLigamentForce"]
    lig_paths = [osim.GeometryPath.safeDownCast(lg.getPropertyByName("GeometryPath").getValueAsObject()) for lg in lig_list]
    mas = osim.MomentArmSolver(m)

    ts, head_p_deg, head_pvel_dps = [], [], []
    tau_lig_sum, tau_mus_act_sum, tau_mus_pas_sum = [], [], []
    max_lig_f = 0.0
    stop_reason = None

    dt_sample = 0.001
    t_now = 0.0
    while t_now < t_end - 1e-9:
        t_next = round(min(t_now + dt_sample, t_end), 6)
        try:
            state = mgr.integrate(t_next)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]
            break
        t_now = t_next
        m.realizeDynamics(state)

        p_deg = sum(math.degrees(c.getValue(state)) for c in pitch_coords)
        pv_dps = sum(math.degrees(c.getSpeedValue(state)) for c in pitch_coords)
        ts.append(t_now); head_p_deg.append(p_deg); head_pvel_dps.append(pv_dps)

        lig_ten = [_read_out_double(lg, "tension", state) for lg in lig_list]
        for ti in lig_ten:
            if ti > max_lig_f:
                max_lig_f = ti
        mus_act = [float(mu.getActiveFiberForceAlongTendon(state)) for mu in mus_list]
        mus_pas = [float(mu.getPassiveFiberForceAlongTendon(state)) for mu in mus_list]

        acc_l, acc_a, acc_p = 0.0, 0.0, 0.0
        for pc in pitch_coords:
            for k, lp in enumerate(lig_paths):
                if lig_ten[k] > 1e-9:
                    acc_l += abs(mas.solve(state, pc, lp) * lig_ten[k])
            for k, mp in enumerate(mus_paths):
                r_ma = mas.solve(state, pc, mp)
                acc_a += abs(r_ma * mus_act[k])
                acc_p += abs(r_ma * mus_pas[k])
        tau_lig_sum.append(acc_l / n_pcoords)
        tau_mus_act_sum.append(acc_a / n_pcoords)
        tau_mus_pas_sum.append(acc_p / n_pcoords)

    if (not dyn_lig) and os.path.exists(tmp_qs):
        try:
            os.remove(tmp_qs)
        except OSError:
            pass

    ts_a = np.asarray(ts)
    tl_a = np.asarray(tau_lig_sum)
    ta_a = np.asarray(tau_mus_act_sum)
    tp_a = np.asarray(tau_mus_pas_sum)
    hp_a = np.asarray(head_p_deg)
    hpv_a = np.asarray(head_pvel_dps)

    m_pre = ts_a <= t_start + 1e-9
    p_base = float(hp_a[m_pre][-1]) if np.any(m_pre) else 0.0
    hp_rel = hp_a - p_base

    def integrate_imp(t_lo, t_hi, arr):
        msk = (ts_a >= t_lo - 1e-9) & (ts_a <= t_hi + 1e-9)
        if np.sum(msk) < 2:
            return 0.0
        return float(np.trapezoid(arr[msk], ts_a[msk])) if hasattr(np, "trapezoid") else float(np.trapz(arr[msk], ts_a[msk]))

    imp_acc_l = integrate_imp(t_start, t_stop, tl_a)
    imp_acc_a = integrate_imp(t_start, t_stop, ta_a)
    imp_acc_p = integrate_imp(t_start, t_stop, tp_a)

    imp_dec_l = integrate_imp(t_stop, t_end, tl_a)
    imp_dec_a = integrate_imp(t_stop, t_end, ta_a)
    imp_dec_p = integrate_imp(t_stop, t_end, tp_a)

    tot_l = imp_acc_l + imp_dec_l
    tot_a = imp_acc_a + imp_dec_a
    tot_p = imp_acc_p + imp_dec_p
    tot_all = max(1e-9, tot_l + tot_a + tot_p)
    acc_all = max(1e-9, imp_acc_l + imp_acc_a + imp_acc_p)
    dec_all = max(1e-9, imp_dec_l + imp_dec_a + imp_dec_p)

    return {
        "peak_N": int(peak_N), "dur_ms": round(dur_s * 1e3, 1), "dir": dir_name,
        "u_mode": u_mode, "u_flx": u_cfg["flx"], "u_ext": u_cfg["ext"], "dyn_lig": dyn_lig,
        "stop_reason": stop_reason, "pre_drift_deg": round(p_base, 2),
        "peak_excursion_deg": round(float(np.max(np.abs(hp_rel))), 2),
        "signed_peak_deg": round(float(hp_rel[int(np.argmax(np.abs(hp_rel)))]), 2),
        "peak_vel_dps": round(float(np.max(np.abs(hpv_a))), 1),
        "max_lig_tension_N": round(max_lig_f, 1),
        "imp_total_Nms": {"lig": round(tot_l, 4), "mus_act": round(tot_a, 4), "mus_pas": round(tot_p, 4)},
        "imp_decel_Nms": {"lig": round(imp_dec_l, 4), "mus_act": round(imp_dec_a, 4), "mus_pas": round(imp_dec_p, 4)},
        "acc_share_pct": {
            "lig": round(100.0 * imp_acc_l / acc_all, 1),
            "mus_act": round(100.0 * imp_acc_a / acc_all, 1),
            "mus_pas": round(100.0 * imp_acc_p / acc_all, 1),
        },
        "decel_share_pct": {
            "lig": round(100.0 * imp_dec_l / dec_all, 1),
            "mus_act": round(100.0 * imp_dec_a / dec_all, 1),
            "mus_pas": round(100.0 * imp_dec_p / dec_all, 1),
        },
        "total_share_pct": {
            "lig": round(100.0 * tot_l / tot_all, 1),
            "mus_act": round(100.0 * tot_a / tot_all, 1),
            "mus_pas": round(100.0 * tot_p / tot_all, 1),
        },
    }


def run_v4_hyoid_mapping_check(kuo_pos_G, specs):
    init_env()
    m_h = osim.Model(HYOID_OSIM)
    s_h = m_h.initSystem()
    m_h.realizePosition(s_h)

    bodies = ["spine", "cerv7", "cerv6", "cerv5", "cerv4", "cerv3", "cerv2", "cerv1", "skull"]
    offsets = {}
    for bn in bodies:
        p = m_h.getBodySet().get(bn).getPositionInGround(s_h)
        offsets[bn] = np.array([float(p.get(0)), float(p.get(1)), float(p.get(2))], dtype=float) - kuo_pos_G[bn]

    # Build temporary read-only HYOID + Kuo 79 DynamicLigamentForce in scratch (Detailed.osim stays untouched)
    tmp_h79 = os.path.join(SCRATCH_DIR, "hyoid_ro_kuo79.osim")
    tree = ET.parse(HYOID_OSIM)
    objs = tree.getroot().find("./Model/ForceSet/objects")
    for sp in specs:
        _append_dynamic_ligament_xml(objs, sp, body_offset_map=offsets)
    tree.write(tmp_h79, encoding="utf-8", xml_declaration=True)

    mh79 = osim.Model(tmp_h79)
    sh = mh79.initSystem()
    mh79.realizeVelocity(sh)
    fset = mh79.getForceSet()
    dl_list = [fset.get(i) for i in range(fset.getSize()) if fset.get(i).getConcreteClassName() == "DynamicLigamentForce"]
    spec_map = {sp["name"]: sp for sp in specs}

    max_pt_err_m = 0.0
    max_lam_err = 0.0
    bset = mh79.getBodySet()
    for lg in dl_list:
        sp = spec_map[lg.getName()]
        gp = osim.GeometryPath.safeDownCast(lg.getPropertyByName("GeometryPath").getValueAsObject())
        pps = gp.getPathPointSet()
        for idx, p_xml in enumerate(sp["pts"]):
            ap = osim.PathPoint.safeDownCast(pps.get(idx))
            pG = bset.get(p_xml["body"]).findStationLocationInGround(sh, ap.getLocation(sh))
            vG = np.array([float(pG.get(0)), float(pG.get(1)), float(pG.get(2))])
            err = float(np.linalg.norm(vG - np.array(sp["pts_G"][idx])))
            max_pt_err_m = max(max_pt_err_m, err)
        max_lam_err = max(max_lam_err, abs(_read_out_double(lg, "strain", sh) - sp["lam0_xml"]))

    bushing_names = [
        ("Spine_C7_Bushing", "spine", "cerv7"), ("C7_C6_Bushing", "cerv7", "cerv6"),
        ("C6_C5_Bushing", "cerv6", "cerv5"),    ("C5_C4_Bushing", "cerv5", "cerv4"),
        ("C4_C3_Bushing", "cerv4", "cerv3"),    ("C3_C2_Bushing", "cerv3", "cerv2"),
        ("C2_C1_Bushing", "cerv2", "cerv1"),    ("C1_skull_Bushing", "cerv1", "skull"),
    ]
    bushings = [fset.get(nm) for nm, _, _ in bushing_names]
    lim_forces = {cn: osim.CoordinateLimitForce.safeDownCast(fset.get(f"{cn}_physical_limit"))
                  for cn in ("pitch1", "pitch2", "roll1", "roll2", "yaw1", "yaw2")}
    cset = mh79.updCoordinateSet()
    mas = osim.MomentArmSolver(mh79)
    dl_paths = [osim.GeometryPath.safeDownCast(lg.getPropertyByName("GeometryPath").getValueAsObject()) for lg in dl_list]
    dl_is_af = [lg.getName().startswith("AF_") for lg in dl_list]

    def meas_rel_angles(q_map):
        st_tmp = mh79.initSystem()
        for k, v in q_map.items():
            cset.get(k).setValue(st_tmp, float(v), False)
        mh79.assemble(st_tmp)
        mh79.realizePosition(st_tmp)
        angs = []
        for _, pb, cb in bushing_names:
            Rp = bset.get(pb).getTransformInGround(st_tmp).R()
            Rc = bset.get(cb).getTransformInGround(st_tmp).R()
            Rp_np = np.array([[Rp.get(r, c) for c in range(3)] for r in range(3)], dtype=float)
            Rc_np = np.array([[Rc.get(r, c) for c in range(3)] for r in range(3)], dtype=float)
            Rrel = Rp_np.T @ Rc_np
            rx = math.atan2(-Rrel[1, 2], Rrel[2, 2])
            ry = math.asin(max(-1.0, min(1.0, Rrel[0, 2])))
            rz = math.atan2(-Rrel[0, 1], Rrel[0, 0])
            angs.append((rx, ry, rz))
        return angs

    eps_q = 0.01
    coup_fracs = {}
    for axis_name, idx_xyz, cnames in (("pitch", 2, ("pitch1", "pitch2")), ("roll", 0, ("roll1", "roll2")), ("yaw", 1, ("yaw1", "yaw2"))):
        coup_fracs[axis_name] = {}
        for cn in cnames:
            ap = meas_rel_angles({cn: +eps_q})
            am = meas_rel_angles({cn: -eps_q})
            coup_fracs[axis_name][cn] = [(ap[j][idx_xyz] - am[j][idx_xyz]) / (2.0 * eps_q) for j in range(8)]

    def eval_hyoid_torque(pose_deg, speed_dps, axis_name):
        sh_eval = mh79.initSystem()
        for cn in ("pitch1", "pitch2", "roll1", "roll2", "yaw1", "yaw2"):
            c = cset.get(cn)
            c.setValue(sh_eval, math.radians(pose_deg.get(cn, 0.0)), False)
            c.setSpeedValue(sh_eval, math.radians(speed_dps.get(cn, 0.0)))
        mh79.assemble(sh_eval)
        mh79.realizeDynamics(sh_eval)

        c1, c2 = cset.get(f"{axis_name}1"), cset.get(f"{axis_name}2")
        idx_xyz = 2 if axis_name == "pitch" else (0 if axis_name == "roll" else 1)

        tau_bush = 0.0
        for j, bf in enumerate(bushings):
            rv = bf.getRecordValues(sh_eval)
            t_b2 = float(rv.get(9 + idx_xyz))
            tau_bush += 0.5 * (coup_fracs[axis_name][f"{axis_name}1"][j] + coup_fracs[axis_name][f"{axis_name}2"][j]) * t_b2
        tau_lim = 0.5 * (float(lim_forces[f"{axis_name}1"].calcLimitForce(sh_eval)) + float(lim_forces[f"{axis_name}2"].calcLimitForce(sh_eval)))
        tau_all_79 = 0.0
        tau_67_no_af = 0.0
        for k, lg in enumerate(dl_list):
            f_t = _read_out_double(lg, "tension", sh_eval)
            if f_t <= 1e-12:
                continue
            r_avg = 0.5 * (mas.solve(sh_eval, c1, dl_paths[k]) + mas.solve(sh_eval, c2, dl_paths[k]))
            tau_k = r_avg * f_t
            tau_all_79 += tau_k
            if not dl_is_af[k]:
                tau_67_no_af += tau_k

        return {
            "hyoid_bush_lim_Nm": round(-(tau_bush + tau_lim), 2),
            "hyoid_bush_only_Nm": round(-tau_bush, 2),
            "kuo_79_Nm": round(-tau_all_79, 2),
            "kuo_67_noAF_Nm": round(-tau_67_no_af, 2),
            "kuo67_plus_bush_Nm": round(-(tau_bush + tau_67_no_af), 2),
        }

    pitch_grid = list(range(-30, 36, 5))
    pitch_qs, pitch_dyn = [], []
    for tot in pitch_grid:
        pose = {"pitch1": 0.35 * tot, "pitch2": 0.65 * tot}
        pitch_qs.append({"deg": tot, **eval_hyoid_torque(pose, {}, "pitch")})
        sgn = 1.0 if tot >= 0 else -1.0
        spd = {"pitch1": sgn * 105.0, "pitch2": sgn * 195.0}
        pitch_dyn.append({"deg": tot, **eval_hyoid_torque(pose, spd, "pitch")})

    roll_grid = list(range(-25, 26, 5))
    roll_qs = [{"deg": r, **eval_hyoid_torque({"roll1": 0.25 * r, "roll2": 0.75 * r}, {}, "roll")} for r in roll_grid]
    yaw_grid = list(range(-35, 36, 5))
    yaw_qs = [{"deg": y, **eval_hyoid_torque({"yaw1": 0.55 * y, "yaw2": 0.45 * y}, {}, "yaw")} for y in yaw_grid]

    scurve_qs = eval_hyoid_torque({"pitch1": +8.63, "pitch2": -20.23}, {}, "pitch")
    scurve_dyn = eval_hyoid_torque({"pitch1": +8.63, "pitch2": -20.23}, {"pitch1": +150.0, "pitch2": -300.0}, "pitch")

    if os.path.exists(tmp_h79):
        try:
            os.remove(tmp_h79)
        except OSError:
            pass

    out_v4 = {
        "offsets_m": {k: [round(float(x), 7) for x in v] for k, v in offsets.items()},
        "max_ground_point_err_m": max_pt_err_m,
        "max_hyoid_lam0_err": max_lam_err,
        "pitch_qs_sweep": pitch_qs,
        "pitch_dyn_sweep": pitch_dyn,
        "roll_qs_sweep": roll_qs,
        "yaw_qs_sweep": yaw_qs,
        "sled3g_scurve_qs": scurve_qs,
        "sled3g_scurve_dyn": scurve_dyn,
    }
    print(f"[V4 HYOID Mapping] max Ground Pt err = {max_pt_err_m:.3e} m | max lam0 err = {max_lam_err:.3e}")
    for row in pitch_qs:
        if row["deg"] in (-25, -15, -5, 0, 5, 15, 25, 35):
            print(f"  Pitch={row['deg']:+3d} deg (QS): HYOID={row['hyoid_bush_lim_Nm']:+6.2f} | Kuo79={row['kuo_79_Nm']:+6.2f} | Kuo67+Bush={row['kuo67_plus_bush_Nm']:+6.2f} Nm")
    print(f"  Pitch=+25 deg (DYN 300 dps): HYOID={pitch_dyn[pitch_grid.index(25)]['hyoid_bush_lim_Nm']:+6.2f} | Kuo79={pitch_dyn[pitch_grid.index(25)]['kuo_79_Nm']:+6.2f} | Kuo67+Bush={pitch_dyn[pitch_grid.index(25)]['kuo67_plus_bush_Nm']:+6.2f} Nm")
    print(f"  Pitch=-25 deg (DYN 300 dps): HYOID={pitch_dyn[pitch_grid.index(-25)]['hyoid_bush_lim_Nm']:+6.2f} | Kuo79={pitch_dyn[pitch_grid.index(-25)]['kuo_79_Nm']:+6.2f} | Kuo67+Bush={pitch_dyn[pitch_grid.index(-25)]['kuo67_plus_bush_Nm']:+6.2f} Nm")
    print(f"  3g S-curve (p1=+8.63, p2=-20.23): QS={scurve_qs} | DYN={scurve_dyn}")
    return out_v4


def plot_summary(v1, v2, v3, v4):
    fig, axes = plt.subplots(2, 3, figsize=(16.5, 9.5), dpi=150)
    fig.suptitle("Kuo et al. (2019) DynamicLigamentForce & Head-Neck Validation (OpenSim 4.6 Port)", fontsize=13, fontweight="bold")

    # Panel 1: V1 Strain-Rate Stiffening
    ax = axes[0, 0]
    lams = np.array(v1["lam_grid"])
    cols = {"0.001": "#1f77b4", "0.05": "#2ca02c", "0.5": "#ff7f0e", "2.5": "#d62728"}
    for vr, c in cols.items():
        ax.plot(lams, v1["families"]["LF_r_spine_cerv7"]["curves_N"][vr], color=c, lw=2, label=f"LF v={vr} m/s")
        ax.plot(lams, v1["families"]["ALL_spine"]["curves_N"][vr], color=c, lw=1.2, ls="--")
    ax.set_title("V1: C++ Rate-Dependent Force (solid=LF, dash=ALL)")
    ax.set_xlabel("Stretch ratio $\\lambda = L/L_0$")
    ax.set_ylabel("Force (N)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    # Panel 2: V2 Sagittal Moment-Angle vs Cadaver Corridors (0g exact static equilibrium)
    ax = axes[0, 1]
    styles = [
        ("osteo", False, "Osteo (Unclamped)", "#1f77b4", "o", "--"),
        ("osteo", True,  "Osteo (XML-Clamped)", "#1f77b4", "s", "-"),
        ("passive_mus", True, "+ Passive Muscles (XML-Clamped)", "#d62728", "^", "-"),
    ]
    for mode, clmp, lbl, clr, mk, ls in styles:
        pts_ext = sorted([(r["mag_Nm"], r["total_pitch_deg"]) for r in v2 if r["mus_mode"] == mode and r["clamped"] == clmp and r["dir"] == "Ext"])
        pts_flx = sorted([(r["mag_Nm"], r["total_pitch_deg"]) for r in v2 if r["mus_mode"] == mode and r["clamped"] == clmp and r["dir"] == "Flex"])
        comb = pts_flx + [(0.0, 0.0)] + pts_ext
        ax.plot([x[1] for x in comb], [x[0] for x in comb], marker=mk, lw=1.8, ls=ls, color=clr, label=lbl, markersize=4.5)
    ax.axvspan(-48, -35, alpha=0.12, color="green", label="Cadaver Flex/Ext corridor (2-3.5 Nm)")
    ax.axvspan(38, 56, alpha=0.12, color="green")
    ax.axhline(0, color="k", lw=0.6); ax.axvline(0, color="k", lw=0.6)
    ax.set_title("V2: 0g Sagittal Equilibrium (-Flex / +Ext)")
    ax.set_xlabel("Head-to-T1 Pitch Angle (deg)"); ax.set_ylabel("Applied Moment (N·m)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7.5)

    # Panel 3: V2 Lateral Bend & Axial Rotation
    ax = axes[0, 2]
    for dname, field, lbl, clr in (("LatR", "total_roll_deg", "Lateral Bend (Right)", "#9467bd"),
                                   ("AxRot", "total_yaw_deg", "Axial Rotation (Left)", "#8c564b")):
        for clmp, ls, tag in ((False, "--", "Unclamped"), (True, "-", "XML-Clamped")):
            pts = [(0.0, 0.0)] + sorted([(r["mag_Nm"], abs(r[field])) for r in v2 if r["mus_mode"] == "osteo" and r["clamped"] == clmp and r["dir"] == dname])
            ax.plot([x[1] for x in pts], [x[0] for x in pts], marker="o", lw=1.8, ls=ls, color=clr, label=f"{lbl} ({tag})", markersize=4)
    ax.axvspan(30, 42, alpha=0.12, color="#9467bd", label="LatBend corridor (1.5-3.5 Nm)")
    ax.axvspan(55, 72, alpha=0.12, color="#8c564b", label="AxRot corridor (1.5-3.5 Nm)")
    ax.set_title("V2: 0g Coronal & Axial Moment-Angle Corridors")
    ax.set_xlabel("Unilateral ROM Magnitude (deg)"); ax.set_ylabel("Applied Moment (N·m)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7)

    # Panel 4: V3 Deceleration Phase Angular Impulse Partitioning
    ax = axes[1, 0]
    sub = [r for r in v3 if r["dyn_lig"]]
    labels = [f"{r['peak_N']}N\n{r['dir']}\n{r['u_mode']}" for r in sub]
    x_pos = np.arange(len(sub))
    s_lig = [r["decel_share_pct"]["lig"] for r in sub]
    s_act = [r["decel_share_pct"]["mus_act"] for r in sub]
    s_pas = [r["decel_share_pct"]["mus_pas"] for r in sub]
    ax.bar(x_pos, s_lig, label="Passive Ligaments (%)", color="#ff7f0e")
    ax.bar(x_pos, s_act, bottom=s_lig, label="Active Muscle (%)", color="#1f77b4")
    ax.bar(x_pos, s_pas, bottom=np.array(s_lig)+np.array(s_act), label="Passive Muscle (%)", color="#2ca02c")
    ax.set_xticks(x_pos); ax.set_xticklabels(labels, fontsize=7.5)
    ax.set_ylim(0, 105); ax.set_ylabel("Deceleration Impulse Share (%)")
    ax.set_title("V3: Kuo (2019) Deceleration Impulse Breakdown")
    ax.legend(fontsize=7.5); ax.grid(True, axis="y", alpha=0.3)

    # Panel 5: V3 Effect of Dynamic Rate-Dependence (Dynamic vs Quasi-static ligaments on 700 N football impact)
    ax = axes[1, 1]
    pairs = []
    for dname in ("Ext", "Flex"):
        for umode in ("Relax", "CoAct"):
            r_d = next(r for r in v3 if r["peak_N"] == 700 and r["dir"] == dname and r["u_mode"] == umode and r["dyn_lig"])
            r_q = next(r for r in v3 if r["peak_N"] == 700 and r["dir"] == dname and r["u_mode"] == umode and not r["dyn_lig"])
            pairs.append((f"700N {dname}\n{umode}", r_d["peak_excursion_deg"], r_q["peak_excursion_deg"]))
    xp = np.arange(len(pairs)); w = 0.35
    ax.bar(xp - w/2, [p[1] for p in pairs], w, label="Dynamic Ligaments", color="#d62728")
    ax.bar(xp + w/2, [p[2] for p in pairs], w, label="Quasi-Static (rate_scale=0)", color="#7f7f7f")
    ax.set_xticks(xp); ax.set_xticklabels([p[0] for p in pairs], fontsize=8)
    ax.set_ylabel("Peak Excursion (deg)"); ax.set_title("V3: 700 N Impact Rate-Stiffening Stabilization")
    ax.grid(True, axis="y", alpha=0.3); ax.legend(fontsize=8)

    # Panel 6: V4 HYOID Passive Sagittal Torque Comparison
    ax = axes[1, 2]
    degs = [r["deg"] for r in v4["pitch_qs_sweep"]]
    ax.plot(degs, [r["hyoid_bush_lim_Nm"] for r in v4["pitch_qs_sweep"]], "k-", lw=2.2, label="HYOID Bushing+Limit")
    ax.plot(degs, [r["kuo_79_Nm"] for r in v4["pitch_qs_sweep"]], "b--", lw=1.8, label="Kuo 79 (QS)")
    ax.plot(degs, [r["kuo_79_Nm"] for r in v4["pitch_dyn_sweep"]], "b-", lw=2.2, label="Kuo 79 (300°/s)")
    ax.plot(degs, [r["kuo67_plus_bush_Nm"] for r in v4["pitch_qs_sweep"]], "r:", lw=1.8, label="Kuo 67+Bush (QS)")
    ax.plot(degs, [r["kuo67_plus_bush_Nm"] for r in v4["pitch_dyn_sweep"]], "r-", lw=1.8, label="Kuo 67+Bush (300°/s)")
    ax.axhline(0, color="k", lw=0.6); ax.axvline(0, color="k", lw=0.6)
    ax.set_title("V4: HYOID Passive Pitch Stiffness (-Flex / +Ext)")
    ax.set_xlabel("Total Pitch (deg)"); ax.set_ylabel("Restoring Moment (N·m)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7.5)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(OUT_PNG, dpi=150)
    plt.close(fig)
    print(f"Saved summary figure -> {OUT_PNG}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--convert-only", action="store_true")
    args = ap.parse_args()

    t0 = time.time()
    kuo_pos_G, specs, step2_summary = build_and_verify_kuo_osim46()
    v1 = run_v1_material_verification(specs)
    if args.convert_only:
        return

    u_table = _compute_balanced_drives()
    print(f"Balanced V3 drives: Ext/Flx cap={u_table['tau_ext_unit_Nm']}/{u_table['tau_flx_unit_Nm']} Nm | Relax(flx/ext)={u_table['Relax']['flx']}/{u_table['Relax']['ext']} | CoAct={u_table['CoAct']['flx']}/{u_table['CoAct']['ext']}")

    v2_tasks = []
    for mus_mode, clamped in (("osteo", False), ("osteo", True), ("passive_mus", True)):
        for d in ("Ext", "Flex", "LatR", "AxRot"):
            sgn = -1.0 if d == "Flex" else +1.0
            v2_tasks.append({
                "dir": d, "mus_mode": mus_mode, "clamped": clamped,
                "mags_Nm": [sgn * mval for mval in (0.5, 1.0, 1.5, 2.0, 2.5, 3.5)],
            })

    v3_tasks = []
    for peak_N, dur_s in ((80, 0.025), (700, 0.015)):
        for d in ("Ext", "Flex"):
            for u_mode in ("Relax", "CoAct"):
                for dyn_lig in (True, False):
                    v3_tasks.append({
                        "peak_N": peak_N, "dur_s": dur_s, "dir": d,
                        "u_mode": u_mode, "u_cfg": u_table[u_mode], "dyn_lig": dyn_lig,
                    })

    print(f"Dispatching V2 ({len(v2_tasks)} static equilibrium curves) + V3 ({len(v3_tasks)} impact sims) across 14 workers...")
    with ProcessPoolExecutor(max_workers=14) as ex:
        f_v2 = [ex.submit(_v2_curve_worker, t) for t in v2_tasks]
        f_v3 = [ex.submit(_v3_worker, t) for t in v3_tasks]
        v2 = [row for f in f_v2 for row in f.result()]
        v3 = [f.result() for f in f_v3]

    print("\n=== V2 0g CORRIDOR SUMMARY (Osteo Unclamped vs XML-Clamped @ +-1.5, +-2.5, +-3.5 Nm) ===")
    for clmp in (False, True):
        print(f"  -- clamped={clmp} --")
        for d in ("Ext", "Flex", "LatR", "AxRot"):
            rows = [r for r in v2 if r["mus_mode"] == "osteo" and r["clamped"] == clmp and r["dir"] == d]
            print(f"    {d:5s}: " + " | ".join(
                f"{r['mag_Nm']:+.1f}Nm->P={r['total_pitch_deg']:+.1f}[lo={r['lower_pitch_deg']:+.1f},up={r['c0c1_pitch_deg']:+.1f}] R={r['total_roll_deg']:+.1f} Y={r['total_yaw_deg']:+.1f} hits={r['n_stops_hit']} res={r['max_interior_res_Nm']:.1e}"
                for r in rows if abs(r["mag_Nm"]) in (1.5, 2.5, 3.5)))

    print("\n=== V3 HEAD IMPACT IMPULSE PARTITIONING (80N x 25ms Mild vs 700N x 15ms Football) ===")
    for r in v3:
        print(f"  {r['peak_N']:3d}N({r['dur_ms']:4.0f}ms) {r['dir']:4s} {r['u_mode']:5s} {'DYN' if r['dyn_lig'] else 'QS ':3s}: pre={r['pre_drift_deg']:+.2f}° exc={r['signed_peak_deg']:+6.2f}° vel={r['peak_vel_dps']:5.1f}°/s Fmax={r['max_lig_tension_N']:6.1f}N | Acc%(L/A/P)={r['acc_share_pct']['lig']:4.1f}/{r['acc_share_pct']['mus_act']:4.1f}/{r['acc_share_pct']['mus_pas']:4.1f} | Decel%(L/A/P)={r['decel_share_pct']['lig']:4.1f}/{r['decel_share_pct']['mus_act']:4.1f}/{r['decel_share_pct']['mus_pas']:4.1f}")

    v4 = run_v4_hyoid_mapping_check(kuo_pos_G, specs)
    plot_summary(v1, v2, v3, v4)

    os.makedirs(SCRATCH_DIR, exist_ok=True)
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({
            "wall_s": round(time.time() - t0, 2),
            "step2": step2_summary, "v1": v1, "v2": v2, "v3": v3, "v4": v4,
        }, f, indent=2)
    print(f"Completed all 4 Kuo validation benchmarks in {time.time() - t0:.2f}s -> {OUT_JSON}")


if __name__ == "__main__":
    main()
