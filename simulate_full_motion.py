import org.opensim.modeling as modeling
import math
import os

print "Preparing 10-second simulation..."

model = getCurrentModel()
if model == None:
    raise Exception("No model loaded in the GUI!")

# Get the custom controller
controller = None
for i in range(model.getControllerSet().getSize()):
    ctrl = model.getControllerSet().get(i)
    if ctrl.getConcreteClassName() == "HeadNeckNeuralController":
        controller = ctrl
        break

if controller == None:
    raise Exception("HeadNeckNeuralController not found in the model!")

# Initialize system
state = model.initSystem()
manager = modeling.Manager(model)
manager.setIntegratorAccuracy(1e-3)
manager.initialize(state)

# Prepare output log
log_path = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\simulation_progress.txt"
out_file = open(log_path, "w")
out_file.write("Starting 10-second Head/Neck Simulation...\n")

def get_targets(t):
    pitch, roll, yaw = 0.0, 0.0, 0.0
    
    # helper for trapeze profile
    # T0 to T0+0.833: 0 to MAX
    # T0+0.833 to T0+2.5: MAX to -MAX
    # T0+2.5 to T0+3.333: -MAX to 0
    def profile(t_local, max_val):
        if t_local < 0.8333:
            return max_val * (t_local / 0.8333)
        elif t_local < 2.5:
            return max_val - 2 * max_val * ((t_local - 0.8333) / 1.6667)
        elif t_local < 3.3333:
            return -max_val + max_val * ((t_local - 2.5) / 0.8333)
        else:
            return 0.0

    if t <= 3.3333:
        # Part 1: Dorsal extension (-60) then ventral flexion (+60)
        pitch = math.radians(profile(t, -60.0))
    elif t <= 6.6666:
        # Part 2: Lateral bending right (+40) then left (-40)
        roll = math.radians(profile(t - 3.3333, 40.0))
    elif t <= 10.0:
        # Part 3: Rotation right (+70) then left (-70)
        yaw = math.radians(profile(t - 6.6666, 70.0))
        
    return pitch, roll, yaw

prop_pitch = controller.updPropertyByName("desired_pitch")
prop_roll = controller.updPropertyByName("desired_roll")
prop_yaw = controller.updPropertyByName("desired_yaw")

dt = 0.05
current_t = 0.0

print "Running integration... check simulation_progress.txt"

while current_t <= 10.0:
    p, r, y = get_targets(current_t)
    
    # Update properties dynamically using PropertyHelper (to bypass SWIG downcasting limitations)
    modeling.PropertyHelper.setValueDouble(p, prop_pitch)
    modeling.PropertyHelper.setValueDouble(r, prop_roll)
    modeling.PropertyHelper.setValueDouble(y, prop_yaw)
    
    next_t = current_t + dt
    next_t = round(next_t, 3)
    manager.integrate(next_t)
    current_t = next_t
    
    # Log progress every ~0.25s
    if abs(current_t % 0.25) < 0.025:
        # Get current actual state from manager
        curr_state = manager.getState()
        cset = model.getCoordinateSet()
        
        act_p = cset.get("pitch1").getValue(curr_state) + cset.get("pitch2").getValue(curr_state)
        act_r = cset.get("roll1").getValue(curr_state) + cset.get("roll2").getValue(curr_state)
        act_y = cset.get("yaw1").getValue(curr_state) + cset.get("yaw2").getValue(curr_state)
        
        msg = "Time: %.2fs | Target [P: %5.1f, R: %5.1f, Y: %5.1f] | Actual [P: %5.1f, R: %5.1f, Y: %5.1f]\n" % (
            current_t, 
            math.degrees(p), math.degrees(r), math.degrees(y),
            math.degrees(act_p), math.degrees(act_r), math.degrees(act_y)
        )
        out_file.write(msg)
        out_file.flush()

out_file.write("Simulation successfully finished!\n")
out_file.close()

try:
    states_table = manager.getStatesTable()
    output_file = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\full_10s_motion.sto"
    
    out_file = open(log_path, "a")
    out_file.write("Writing STO file to disk...\n")
    modeling.STOFileAdapter.write(states_table, output_file)
    out_file.write("Success! Full 10s motion saved to: " + output_file + "\n")
    out_file.close()

except Exception, e:
    import traceback
    out_file = open(log_path, "a")
    out_file.write("ERROR OCCURRED: " + str(e) + "\n")
    out_file.write("TRACEBACK:\n")
    out_file.write(traceback.format_exc() + "\n")
    out_file.close()

