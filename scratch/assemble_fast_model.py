import os
import opensim as osim

# Load all custom plugins
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

original_model_file = 'osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim'
function_based_paths_file = 'scratch/path_fitter_results/HYOID_Scaled_FunctionBasedPathSet.xml'
fast_model_file = 'osim_files/HYOID_HeadNeckNeural_Model_FastGeometry.osim'

print("Assembling fast model with FunctionBasedPaths...")
modelProcessor = osim.ModelProcessor(original_model_file)
modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(function_based_paths_file))

fast_model = modelProcessor.process()
state = fast_model.initSystem()

# Verify that muscles now use FunctionBasedPath
muscles = fast_model.getMuscles()
print(f"Total muscles in fast model: {muscles.getSize()}")
fb_count = 0
for i in range(muscles.getSize()):
    mus = muscles.get(i)
    path = mus.getPath()
    if path.getConcreteClassName() == "FunctionBasedPath":
        fb_count += 1

print(f"FunctionBasedPaths replaced: {fb_count} / {muscles.getSize()} muscles!")

# Save to XML
fast_model.printToXML(fast_model_file)
print(f"Saved fast model to {fast_model_file} successfully!")
