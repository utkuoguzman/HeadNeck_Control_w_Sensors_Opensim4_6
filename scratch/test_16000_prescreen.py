import cma
import numpy as np
import time

x0 = np.ones(12)
sigma0 = 0.2
opts = {'popsize': 18, 'maxiter': 5}

es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

for it in range(3):
    t0 = time.perf_counter()
    pool = es.ask(number=16000)
    t_ask = time.perf_counter() - t0
    
    # Simple dummy score
    scores_pool = [np.sum((c - 2.0)**2) for c in pool[:1000]] # test 1000 first
    
    top_18 = pool[:18]
    scores_18 = [np.sum((c - 2.0)**2) for c in top_18]
    
    t1 = time.perf_counter()
    es.tell(top_18, scores_18)
    t_tell = time.perf_counter() - t1
    
    print(f"Iter {it+1}: Asked 16,000 candidates in {t_ask*1000:.1f} ms. es.tell with 18 candidates in {t_tell*1000:.1f} ms. Best: {es.result.fbest:.4f}")

print("Multi-iteration 16,000 pre-screening test passed!")

