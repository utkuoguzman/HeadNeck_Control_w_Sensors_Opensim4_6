import json, numpy as np, cma, sys, os
sys.path.append(os.getcwd())
src = open("scripts/run_phase2_4_campaign.py", encoding="utf-8").read().replace('Tee("logs/phase2_4_campaign.log", "a")', "")
ns = {"__name__": "camp"}
exec(compile(src, "camp", "exec"), ns)
sampler = ns["sample_candidates_tpa_orthogonal"]

p = json.load(open("data/prescreen_rational_champion.json"))
x0 = np.array(p["x0"]); sig = np.array(p["sigma0"])
lo, up = np.array(p["lower_bounds"]), np.array(p["upper_bounds"])
es = cma.CMAEvolutionStrategy(x0, 1.0, {'CMA_stds': sig, 'bounds': [lo, up], 'popsize': 14,
                                        'AdaptSigma': cma.sigma_adaptation.CMAAdaptSigmaTPA,
                                        'seed': 2026, 'verbose': -9})
np.random.seed(0)
ok = True
print("RMS distance from centroid in sigma0 units (expect ~1.0-1.8 for all 14):")
for trial in range(5):
    X = np.array(sampler(es, 1))
    r = np.sqrt((((X - np.array(es.mean)) / (sig * es.sigma)) ** 2).mean(axis=1))
    print(f" trial {trial}: min={r.min():.2f} max={r.max():.2f} | " + " ".join(f"{v:.1f}" for v in r))
    ok &= (r.max() < 2.6) and (r.min() > 0.3)

# antipodal pair symmetry + mutual orthogonality in z-space
B, D, s = es.sm.B, es.sm.D, es.sigma
Z = np.array([(B.T @ ((x - np.array(es.mean)) / sig)) / (D * s) for x in X])
sym = max(np.linalg.norm(Z[2 + 2*k] + Z[3 + 2*k]) / np.linalg.norm(Z[2 + 2*k]) for k in range(6))
U = np.array([Z[2 + 2*k] / np.linalg.norm(Z[2 + 2*k]) for k in range(6)])
orth = np.abs(U @ U.T - np.eye(6)).max()
print(f"antipodal asymmetry={sym:.2e}  max|<ui,uj>|={orth:.2e}")
ok &= sym < 1e-6 and orth < 1e-6

# several full tell cycles with synthetic scores incl. GP branch; verify es stays sane
archX, archY = [], []
rng = np.random.default_rng(1)
target = x0 * (1 + 0.1 * rng.standard_normal(12))
f = lambda x: float(np.sum(((np.array(x) - target) / sig) ** 2))
for g in range(1, 13):
    X = sampler(es, g, archX, archY)
    Xc = [np.clip(c, lo + 1e-4, up - 1e-4).tolist() for c in X]
    y = [f(c) for c in Xc]
    es.tell(Xc, y)
    archX += Xc; archY += y
    if g in (1, 6, 9, 12):
        print(f" synthetic gen {g}: best={min(y):9.1f} median={np.median(y):9.1f} sigma={es.sigma:.3f}")
print("PASS" if ok else "FAIL")

