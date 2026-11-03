import java
try:
    
    import os
    import sys
    import math
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
    except:
        script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
    os.chdir(script_dir)
    
    import org.opensim.modeling as modeling
    import time
    
    print "Preparing 10-second simulation with eyes..."
    
    log_path = os.path.join(script_dir, "logs", "log_full_motion_eyes.txt")
    out_file = open(log_path, "w")
    out_file.write("Starting 10-second Head/Neck/Eye Simulation...\n")
    out_file.close()
    
    def log(msg):
        f = open(log_path, "a")
        f.write(msg + "\n")
        f.close()
        print msg
    
    # Load plugins explicitly to be safe
    modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "proprioception_plugin/build/Release/osimMillard12EqWithAff.dll"))
    modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "vestibular_plugin/build/Release/osimVestibular.dll"))
    modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll"))
    modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "FixationController_plugin/build/Release/osimFixation.dll"))
    
    model_path = os.path.join(script_dir, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")
    model = modeling.Model(model_path)
    
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl
            break
    
    if controller == None:
        raise Exception("HeadNeckNeuralController not found!")
    
    # Initialize system
    state = model.initSystem()
    manager = modeling.Manager(model)
    manager.setIntegratorMethod(5) # Implicit SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-8)
    manager.initialize(state)
    
    def get_targets(t):
        pitch, roll, yaw = 0.0, 0.0, 0.0
        def profile(t_local, max_val):
            if t_local < 0.8333: return max_val * (t_local / 0.8333)
            elif t_local < 2.5: return max_val - 2 * max_val * ((t_local - 0.8333) / 1.6667)
            elif t_local < 3.3333: return -max_val + max_val * ((t_local - 2.5) / 0.8333)
            else: return 0.0
    
        # Stay safely away from absolute limits (60, 40, 70) to avoid slow limit springs
        if t <= 3.3333: pitch = math.radians(profile(t, -45.0))
        elif t <= 6.6666: roll = math.radians(profile(t - 3.3333, 30.0))
        elif t <= 10.0: yaw = math.radians(profile(t - 6.6666, 55.0))
        return pitch, roll, yaw
    
    prop_pitch = controller.updPropertyByName("desired_pitch")
    prop_roll = controller.updPropertyByName("desired_roll")
    prop_yaw = controller.updPropertyByName("desired_yaw")
    
    dt = 0.05
    current_t = 0.0
    start_wall = time.clock()
    
    # Dynamic target based on FixationController property
    ground = model.getGround()
    skull = model.getBodySet().get("skull")
    r_eye = model.getBodySet().get("r_eye")
    l_eye = model.getBodySet().get("l_eye")
    
    # Extract the fixation controller to read its target location
    fixation = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            fixation = ctrl
            break
    
    log("Running integration... check logs/log_full_motion_eyes.txt")
    
    while current_t <= 10.0:
        p, r, y = get_targets(current_t)
        
        modeling.PropertyHelper.setValueDouble(p, prop_pitch)
        modeling.PropertyHelper.setValueDouble(r, prop_roll)
        modeling.PropertyHelper.setValueDouble(y, prop_yaw)
        
        next_t = current_t + dt
        manager.integrate(next_t)
        current_t = next_t
        
        if abs(round(current_t / 0.25) - (current_t / 0.25)) < 0.01:
            curr_state = manager.getState()
            cset = model.getCoordinateSet()
            
            act_p = cset.get("pitch1").getValue(curr_state) + cset.get("pitch2").getValue(curr_state)
            act_r = cset.get("roll1").getValue(curr_state) + cset.get("roll2").getValue(curr_state)
            act_y = cset.get("yaw1").getValue(curr_state) + cset.get("yaw2").getValue(curr_state)
            
            act_r_yaw = cset.get("r_eye_add_abd").getValue(curr_state)
            act_r_pitch = cset.get("r_eye_sup_inf").getValue(curr_state)
            act_l_yaw = cset.get("l_eye_add_abd").getValue(curr_state)
            act_l_pitch = cset.get("l_eye_sup_inf").getValue(curr_state)
            act_r_torsion = cset.get("r_eye_inc_exc").getValue(curr_state)
            act_l_torsion = cset.get("l_eye_inc_exc").getValue(curr_state)
            
            # Explicit biological inverse attitude
            target_skull = modeling.Vec3(1.0, 0.0, 0.0)
            
            r_eye_orig = ground.findStationLocationInAnotherFrame(curr_state, r_eye.getPositionInGround(curr_state), skull)
            r_to_target = modeling.Vec3(target_skull.get(0) - r_eye_orig.get(0), target_skull.get(1) - r_eye_orig.get(1), target_skull.get(2) - r_eye_orig.get(2))
            des_r_yaw = math.atan2(-r_to_target.get(2), r_to_target.get(0))
            des_r_pitch = math.atan2(r_to_target.get(1), r_to_target.get(0))
            
            l_eye_orig = ground.findStationLocationInAnotherFrame(curr_state, l_eye.getPositionInGround(curr_state), skull)
            l_to_target = modeling.Vec3(target_skull.get(0) - l_eye_orig.get(0), target_skull.get(1) - l_eye_orig.get(1), target_skull.get(2) - l_eye_orig.get(2))
            des_l_yaw = math.atan2(-l_to_target.get(2), l_to_target.get(0))
            des_l_pitch = math.atan2(l_to_target.get(1), l_to_target.get(0))
            
            # Add head attitude inverse
            des_r_yaw += act_y
            des_l_yaw += act_y
            des_r_pitch -= act_p
            des_l_pitch -= act_p
            des_torsion = -act_r
    
            wall_time = time.clock() - start_wall
            
            msg = "Time: %.2fs (Wall: %.1fs)\n" % (current_t, wall_time)
            msg += "  Neck Target [P: %5.1f, R: %5.1f, Y: %5.1f] | Actual [P: %5.1f, R: %5.1f, Y: %5.1f]\n" % (
                math.degrees(p), math.degrees(r), math.degrees(y),
                math.degrees(act_p), math.degrees(act_r), math.degrees(act_y)
            )
            msg += "  R_Eye Target [Abd: %5.2f, Ele: %5.2f, Tor: %5.2f] | Actual [Abd: %5.2f, Ele: %5.2f, Tor: %5.2f]\n" % (
                math.degrees(des_r_yaw), math.degrees(des_r_pitch), math.degrees(des_torsion), 
                math.degrees(act_r_yaw), math.degrees(act_r_pitch), math.degrees(act_r_torsion)
            )
            msg += "  L_Eye Target [Abd: %5.2f, Ele: %5.2f, Tor: %5.2f] | Actual [Abd: %5.2f, Ele: %5.2f, Tor: %5.2f]\n" % (
                math.degrees(des_l_yaw), math.degrees(des_l_pitch), math.degrees(des_torsion), 
                math.degrees(act_l_yaw), math.degrees(act_l_pitch), math.degrees(act_l_torsion)
            )
            log(msg)
    
    log("Simulation successfully finished!")
    
    states_table = manager.getStatesTable()
    output_file = os.path.join(script_dir, "state_files", "detailed_full_10s_motion_with_eyes.sto")
    modeling.STOFileAdapter.write(states_table, output_file)
    log("Success! Full 10s motion saved to: " + output_file)
    
    
except Exception, e:
    import traceback
    f = open(log_path, "a")
    f.write("ERROR OCCURRED: " + str(e) + "\n")
    f.write("TRACEBACK:\n")
    f.write(traceback.format_exc() + "\n")
    f.close()
except java.lang.Exception, e:
    import traceback
    f = open(log_path, "a")
    f.write("JAVA ERROR OCCURRED: " + str(e) + "\n")
    f.write("TRACEBACK:\n")
    f.write(traceback.format_exc() + "\n")
    f.close()
except:
    import sys
    import traceback
    f = open(log_path, "a")
    f.write("UNKNOWN ERROR OCCURRED!\n")
    f.write("TRACEBACK:\n")
    f.write(traceback.format_exc() + "\n")
    f.close()
