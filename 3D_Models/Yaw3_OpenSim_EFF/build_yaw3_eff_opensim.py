import os
import math
import opensim as osim

SUBJECT_OSIM_PATH = ""  # Path to scaled subject .osim, or "" for standalone rig verification
OUTPUT_OSIM_PATH  = "Yaw3_ElasticFoundation_Model.osim"

# Exact OpenSim joint offsets (meters) from your final audited Blender 5.3 rig:
OFF_BASE  = (0.0, 0.0, 0.0)
OFF_YAW   = (0.0, 0.0712, 0.0)
OFF_PITCH = (0.0, 0.7588, 0.0)
OFF_ROLL  = (0.0, 0.0, 0.0)
OFF_SEAT  = (-0.1458, -0.2211, 0.0)


def set_axis_function_safe(ta, func_obj):
    """Uses OpenSim 4.x value-cloning property setter so Python GC never frees a live C++ pointer."""
    if hasattr(ta, "set_function"):
        ta.set_function(func_obj)
    else:
        func_obj.thisown = 0
        ta.setFunction(func_obj)


def make_rig_body(model, name, mass_kg, vis_obj):
    b = osim.Body(name, mass_kg, osim.Vec3(0), osim.Inertia(0.5, 0.5, 0.5, 0, 0, 0))
    model.addBody(b)
    model.finalizeFromProperties()
    b.attachGeometry(osim.Mesh(vis_obj))
    return b


def add_1dof_joint(model, jname, pframe, loc_p, cbody, cname, rslot, axis_v, rmin, rmax, clamped=True, locked=False):
    # SpatialTransform() already initializes all 6 slots with orthogonal axes and Constant(0.0);
    # we only configure the single active rotational slot.
    st = osim.SpatialTransform()
    ta = getattr(st, "upd_" + rslot)()
    ta.append_coordinates(cname)
    ta.setAxis(axis_v)
    set_axis_function_safe(ta, osim.LinearFunction(1.0, 0.0))

    jnt = osim.CustomJoint(
        jname,
        pframe, osim.Vec3(*loc_p), osim.Vec3(0, 0, 0),
        cbody,  osim.Vec3(0, 0, 0), osim.Vec3(0, 0, 0),
        st
    )

    # Add joint and finalize properties FIRST so CustomJoint constructs coordinates[0]
    model.addJoint(jnt)
    model.finalizeFromProperties()

    c = jnt.upd_coordinates(0)
    c.setRangeMin(math.radians(rmin))
    c.setRangeMax(math.radians(rmax))
    c.setDefaultValue(0.0)
    c.setDefaultClamped(clamped)
    c.setDefaultLocked(locked)
    return jnt


def make_eff_group(name, geom_names, stiffness=1.5e6, dissipation=2.0, mu_s=0.75, mu_d=0.65):
    cp_cls = (
        getattr(osim, "ElasticFoundationForce_ContactParameters", None)
        or osim.ElasticFoundationForce.ContactParameters
    )
    params = cp_cls(stiffness, dissipation, mu_s, mu_d, 0.5)
    for gname in geom_names:
        params.addGeometry(gname)

    eff = osim.ElasticFoundationForce()
    eff.setName(name)
    eff.setTransitionVelocity(0.1)
    eff.upd_contact_parameters().cloneAndAppend(params)
    return eff


def build():
    here = os.path.dirname(os.path.abspath(__file__))
    os.chdir(here)
    geom_dir = os.path.join(here, "Geometry")
    osim.ModelVisualizer.addDirToGeometrySearchPaths(geom_dir)

    model = osim.Model(SUBJECT_OSIM_PATH) if SUBJECT_OSIM_PATH else osim.Model()
    model.setName("Yaw3_EFF_SoftContact")
    model.setGravity(osim.Vec3(0.0, -9.80665, 0.0))

    # 1. Build Bodies & Visual Meshes
    b_base  = make_rig_body(model, "yaw3_base",      15.0, "yaw3_base_vis.obj")
    b_yaw   = make_rig_body(model, "yaw3_turntable", 12.0, "yaw3_turntable_vis.obj")
    b_pitch = make_rig_body(model, "yaw3_pitch",     15.0, "yaw3_pitch_vis.obj")
    b_roll  = make_rig_body(model, "yaw3_roll",      10.0, "yaw3_roll_seat_vis.obj")
    b_back  = make_rig_body(model, "yaw3_backrest",   8.0, "yaw3_backrest_vis.obj")

    # 2. Build Kinematic Chain
    wj = osim.WeldJoint(
        "yaw3_ground_weld",
        model.getGround(), osim.Vec3(*OFF_BASE), osim.Vec3(0),
        b_base,            osim.Vec3(0),         osim.Vec3(0)
    )
    model.addJoint(wj)

    add_1dof_joint(model, "jnt_yaw3_yaw",   b_base,  OFF_YAW,   b_yaw,   "yaw3_yaw",          "rotation2", osim.Vec3(0, 1, 0), -3600.0, +3600.0, clamped=False, locked=False)
    add_1dof_joint(model, "jnt_yaw3_pitch", b_yaw,   OFF_PITCH, b_pitch, "yaw3_pitch",        "rotation3", osim.Vec3(0, 0, 1),   -15.0,   +55.0, clamped=True,  locked=False)
    add_1dof_joint(model, "jnt_yaw3_roll",  b_pitch, OFF_ROLL,  b_roll,  "yaw3_roll",         "rotation1", osim.Vec3(1, 0, 0),   -21.0,   +21.0, clamped=True,  locked=False)
    add_1dof_joint(model, "jnt_yaw3_seat",  b_roll,  OFF_SEAT,  b_back,  "yaw3_seat_recline", "rotation3", osim.Vec3(0, 0, -1),  -10.0,   +20.0, clamped=True,  locked=True)

    # 3. Add Watertight Retopo ContactMeshes
    cm_pan  = osim.ContactMesh("Geometry/yaw3_seatpan_contact.obj",  osim.Vec3(0), osim.Vec3(0), b_roll, "contact_seatpan")
    cm_back = osim.ContactMesh("Geometry/yaw3_backrest_contact.obj", osim.Vec3(0), osim.Vec3(0), b_back, "contact_backrest")
    model.addContactGeometry(cm_pan)
    model.addContactGeometry(cm_back)

    pan_geoms  = ["contact_seatpan"]
    back_geoms = ["contact_backrest"]

    # 4. Pair With Subject Trunk ContactSpheres if Subject Model Is Provided
    bset = model.getBodySet()
    if bset.contains("pelvis"):
        pelvis = bset.get("pelvis")
        for sname, sloc, srad in [
            ("sph_ischial_l", (-0.03, -0.08, -0.065), 0.06),
            ("sph_ischial_r", (-0.03, -0.08, +0.065), 0.06),
            ("sph_hip_lat_l", (-0.01, -0.04, -0.125), 0.05),
            ("sph_hip_lat_r", (-0.01, -0.04, +0.125), 0.05),
        ]:
            model.addContactGeometry(osim.ContactSphere(srad, osim.Vec3(*sloc), pelvis, sname))
            pan_geoms.append(sname)

        for sname, sloc in [("sph_sacrum_l", (-0.085, 0.02, -0.055)), ("sph_sacrum_r", (-0.085, 0.02, +0.055))]:
            model.addContactGeometry(osim.ContactSphere(0.055, osim.Vec3(*sloc), pelvis, sname))
            back_geoms.append(sname)

    tname = "torso" if bset.contains("torso") else ("thorax" if bset.contains("thorax") else "")
    if tname:
        tbody = bset.get(tname)
        for sname, sloc, srad in [
            ("sph_lumbar_l",  (-0.085, 0.13, -0.06), 0.055),
            ("sph_lumbar_r",  (-0.085, 0.13, +0.06), 0.055),
            ("sph_scapula_l", (-0.095, 0.30, -0.08), 0.065),
            ("sph_scapula_r", (-0.095, 0.30, +0.08), 0.065),
        ]:
            model.addContactGeometry(osim.ContactSphere(srad, osim.Vec3(*sloc), tbody, sname))
            back_geoms.append(sname)

    if len(pan_geoms) > 1:
        model.addForce(make_eff_group("EFF_SeatPan", pan_geoms))
    if len(back_geoms) > 1:
        model.addForce(make_eff_group("EFF_Backrest", back_geoms))

    # 5. Save .osim XML and verify full reload + Simbody TriangleMesh initialization
    model.finalizeConnections()
    out_path = os.path.join(here, OUTPUT_OSIM_PATH)
    model.printToXML(out_path)
    print("Saved ->", out_path)

    verify_model = osim.Model(out_path)
    verify_model.initSystem()
    print("[OK] Reloaded .osim and initialized Simbody multibody + ContactMesh cleanly!")


if __name__ == "__main__":
    build()