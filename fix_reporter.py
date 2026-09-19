
import glob

for file in glob.glob("run_*_*.py"):
    with open(file, "r") as f:
        content = f.read()
    
    if "modeling.StatesReporter()" not in content:
        # Insert reporter before initSystem
        content = content.replace(
            "state = model.initSystem()",
            "reporter = modeling.StatesReporter()\nreporter.setName(\"states_reporter\")\nreporter.set_report_time_interval(0.01)\nmodel.addComponent(reporter)\nstate = model.initSystem()"
        )
        # Use reporter table instead of manager table
        content = content.replace(
            "statesTable = manager.getStatesTable()",
            "statesTable = reporter.getStatesTable()"
        )
        
        with open(file, "w") as f:
            f.write(content)

