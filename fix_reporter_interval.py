
import glob

for file in glob.glob("run_vestibular_detailed_*.py"):
    with open(file, "r") as f:
        code = f.read()
    
    code = code.replace("reporter.setName(\"vestibular_reporter\")", "reporter.setName(\"vestibular_reporter\")\nreporter.set_report_time_interval(0.01)")
    
    with open(file, "w") as f:
        f.write(code)

