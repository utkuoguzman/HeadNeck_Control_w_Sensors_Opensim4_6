import cma
import numpy as np

x0 = np.ones(12)
sigma0 = 0.2
opts = {'popsize': 18, 'maxiter': 5}

es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

# Test ask with number
pool = es.ask(number=72)
print(f"Generated pool of {len(pool)} candidates from distribution.")

# Select top 18
top_18 = pool[:18]
scores = [np.sum(c**2) for c in top_18]

try:
    es.tell(top_18, scores)
    print("es.tell(top_18, scores) succeeded cleanly!")
except Exception as e:
    print(f"es.tell failed: {e}")

