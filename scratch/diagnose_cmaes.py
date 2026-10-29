import json, glob, re, numpy as np, cma, sys, os
sys.path.append(os.getcwd())

# 1. Which candidate index wins each generation?
print("== Winner index per generation ==")
wins = []
for g in range(1, 16):
    s = json.load(open(f"logs/phase2_4_gen_{g}_summary.json"))
    sc = np.array(s["all_scores"])
    wins.append(s["best_candidate_idx"])
    valid = sc[sc < 250000]
    print(f"gen{g:2d} win=W{s['best_candidate_idx']:2d} best={sc.min():8.0f} "
          f"median={np.median(sc):9.0f}  TPA(W1,W2)={sc[0]:.0f},{sc[1]:.0f}  "
          f"median(W3-14)={np.median(sc[2:]):.0f}")
print("winners:", wins, " -> fraction in W1/W2:", np.mean([w in (1, 2) for w in wins]))

# 2. Does es.mean live in the same space as es.ask() output?
p = json.load(open("data/prescreen_rational_champion.json"))
x0 = np.array(p["x0"]); sig = np.array(p["sigma0"])
es = cma.CMAEvolutionStrategy(x0, 1.0, {'CMA_stds': sig, 'bounds': [p["lower_bounds"], p["upper_bounds"]],
                                        'popsize': 14, 'seed': 1, 'verbose': -9})
X = np.array(es.ask())
print("\n== Space check ==")
print("x0           :", np.round(x0, 3))
print("es.mean      :", np.round(np.array(es.mean), 3))
print("ask() mean   :", np.round(X.mean(0), 3))
d0 = X[0] - np.array(es.mean)
print("|X[0]-es.mean| / |X[0]-x0| =", np.linalg.norm(d0) / np.linalg.norm(X[0] - x0))

# 3. Reproduce the custom orthogonal sampler and compare against genuine ask()
from importlib import util
spec = util.spec_from_file_location("camp", "scripts/run_phase2_4_campaign.py")
src = open("scripts/run_phase2_4_campaign.py", encoding="utf-8").read()
src = src.replace('Tee("logs/phase2_4_campaign.log", "a")', "")
ns = {"__name__": "camp"}
exec(compile(src, "camp", "exec"), ns)
Xc = np.array(ns["sample_candidates_tpa_orthogonal"](es, 1))
rel = lambda A: np.abs(A - x0) / sig
print("\nnormalised distance from x0 (in units of sigma0), per candidate:")
print("candidate  :", " ".join(f"{i+1:5d}" for i in range(14)))
print("rms |dx|/s :", " ".join(f"{np.sqrt((rel(Xc)[i]**2).mean()):5.1f}" for i in range(14)))
print("(first two are genuine es.ask() TPA samples, other 12 are custom-built)")

