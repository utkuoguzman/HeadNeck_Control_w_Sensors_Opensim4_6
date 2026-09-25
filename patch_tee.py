with open('optimize_PID_cmaes_parallel.py', 'r') as f:
    text = f.read()

if 'class Tee' not in text:
    tee_code = '''
import os
import datetime

class Tee(object):
    def __init__(self, name, mode):
        os.makedirs(os.path.dirname(name), exist_ok=True)
        self.file = open(name, mode)
        self.stdout = sys.stdout
        sys.stdout = self
    def __del__(self):
        sys.stdout = self.stdout
        self.file.close()
    def write(self, data):
        self.file.write(data)
        self.file.flush()
        self.stdout.write(data)
        self.stdout.flush()
    def flush(self):
        self.file.flush()
        self.stdout.flush()

timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
sys.stdout = Tee(f"logs/cmaes_optimization_{timestamp}.log", "a")
'''
    text = text.replace('import random', 'import random\n' + tee_code)
    
    with open('optimize_PID_cmaes_parallel.py', 'w') as f:
        f.write(text)
