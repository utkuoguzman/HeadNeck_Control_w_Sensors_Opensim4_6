import opensim as osim
osim.LoadOpenSimLibrary("osimMillard12EqWithAff")
model = osim.Model()
muscle = osim.Millard2012EquilibriumMuscle("muscle", 1000.0, 0.1, 0.05, 0.0)
muscle.addNewPathPoint("origin", model.getGround(), osim.Vec3(0, 0, 0))
muscle.addNewPathPoint("insertion", model.getGround(), osim.Vec3(1.0, 0, 0))
model.addForce(muscle)
model.finalizeConnections()
model.printToXML("temp_out.osim")
with open("temp_out.osim", 'r') as f:
    xml = f.read()
xml = xml.replace('Millard2012EquilibriumMuscle', 'Millard12EqMuscleWithAfferents')
with open("temp_out.osim", 'w') as f:
    f.write(xml)
model = osim.Model("temp_out.osim")
state = model.initSystem()
model.realizeAcceleration(state)
muscle = model.getForceSet().get("muscle")
try:
    val = osim.OutputDouble.safeDownCast(muscle.getOutput("primary_Ia")).getValue(state)
    print("USING OutputDouble.safeDownCast: ", val)
except Exception as e:
    print("FAILED OutputDouble: ", e)

try:
    val = float(muscle.getOutput("primary_Ia").getValueAsString(state))
    print("USING getValueAsString: ", val)
except Exception as e:
    print("FAILED getValueAsString: ", e)
