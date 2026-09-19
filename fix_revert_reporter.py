
import glob

for file in glob.glob("run_*_*.py"):
    with open(file, "r") as f:
        content = f.read()
    
    # Remove the reporter lines
    content = content.replace("reporter = modeling.StatesReporter()\nreporter.setName(\"states_reporter\")\nreporter.set_report_time_interval(0.01)\nmodel.addComponent(reporter)\n", "")
    content = content.replace("statesTable = reporter.getStatesTable()", "statesTable = manager.getStatesTable()")
    
    with open(file, "w") as f:
        f.write(content)

