import os
import opensim as osim
import numpy as np

# Load plugins
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model_path = 'osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim'
model = osim.Model(model_path)
# Remove non-muscle forces and controllers so PolynomialPathFitter only evaluates pure muscle paths
fset = model.updForceSet()
for i in reversed(range(fset.getSize())):
    f = fset.get(i)
    if 'Bushing' in f.getConcreteClassName() or not f.getConcreteClassName().startswith('Millard'):
        fset.remove(i)
model.updControllerSet().clearAndDestroy()
model.initSystem()

fitter = osim.PolynomialPathFitter()
fitter.setModel(osim.ModelProcessor(model))

# Create a coordinate values table across representative motion (pitch, roll, yaw)
# For our cervical spine, the main active DOFs during perturbation are pitch1, pitch2, roll1, roll2, yaw1, yaw2
# And spine_tx is the torso translation.
cset = model.getCoordinateSet()
labels = osim.StdVectorString()
for i in range(cset.getSize()):
    c = cset.get(i)
    # OpenSim state path format: /jointset/<joint>/<coord>/value
    # Let's check coordinate path
    labels.append(c.getAbsolutePathString() + '/value')

# Create a table with 50 nominal time frames sampling cervical range of motion
# pitch1, pitch2 in [-15 deg, +15 deg] = [-0.26, +0.26] rad
# roll1, roll2 in [-10 deg, +10 deg] = [-0.17, +0.17] rad
# yaw1, yaw2 in [-10 deg, +10 deg] = [-0.17, +0.17] rad
table = osim.TimeSeriesTable()
table.addTableMetaDataString('inDegrees', 'no')
table.setColumnLabels(labels)

times = np.linspace(0.0, 5.0, 51)
for t in times:
    row = osim.RowVector(cset.getSize(), 0.0)
    # Add gentle sinusoidal variations for the independent coordinates
    for i in range(cset.getSize()):
        c = cset.get(i)
        name = c.getName()
        val = 0.0
        if 'pitch' in name:
            val = 0.15 * np.sin(2.0 * np.pi * 0.5 * t)
        elif 'roll' in name:
            val = 0.05 * np.cos(2.0 * np.pi * 0.5 * t)
        elif 'yaw' in name:
            val = 0.05 * np.sin(2.0 * np.pi * 0.7 * t)
        row[i] = float(val)
    table.appendRow(float(t), row)

print(f"Created coordinate table with {table.getNumRows()} rows and {table.getNumColumns()} columns.")
fitter.setCoordinateValues(osim.TableProcessor(table))

os.makedirs('scratch/path_fitter_results', exist_ok=True)
fitter.setOutputDirectory('scratch/path_fitter_results')
fitter.setMaximumPolynomialOrder(4)
fitter.setGlobalCoordinateSamplingBounds(osim.Vec2(-15, 15))
fitter.setPathLengthTolerance(2e-3)
fitter.setMomentArmTolerance(2e-3)
fitter.setNumSamplesPerFrame(5)

print("Starting PolynomialPathFitter...")
try:
    fitter.run()
    print("Fitting completed successfully!")
except Exception as e:
    import traceback
    traceback.print_exc()
