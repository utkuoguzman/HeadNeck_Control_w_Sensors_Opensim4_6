with open("scripts/run_phase2_4_campaign.py", "r", encoding="utf-8") as f:
    src = f.read()

old_targets = """# 6 Canonical Aligned Frequencies (LPT / early-pruning order)
TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (5.583, 503.30, -201.38),
    (2.446, 423.34, -161.34),
    (7.927, 572.69, -247.62)
]"""

new_targets = """# 6 Canonical Aligned Frequencies (Fastest-First + High-Sensitivity Early-Pruning Order)
TARGETS = [
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62),
    (1.099, 319.74, -109.23),
    (0.783, 236.93,  -96.96),
    (0.324,  86.55,   -7.03)
]"""

assert old_targets in src, "old_targets not found"
src = src.replace(old_targets, new_targets, 1)

old_gp_use = """    if use_gp:
        # Fit local GP on nearest 80 historical models
        dists = np.linalg.norm(np.array(archive_X) - mean, axis=1)
        idx = np.argsort(dists)[:80]
        local_X = [archive_X[i] for i in idx]
        local_y = [archive_y[i] for i in idx]
        
        l_scale = max(0.1, float(np.median(dists[idx])))
        gp = SimpleGP(length_scale=l_scale)
        gp.fit(local_X, local_y)
        
        # Sample candidate orthogonal bases and evaluate direction utility
        best_score = float('inf')
        best_Q = None
        for _ in range(12):
            G = np.random.randn(12, 6)
            G = G - np.outer(u_tpa, u_tpa @ G)
            Q, _ = np.linalg.qr(G)
            
            test_pts = []
            for k in range(6):
                zk = Q[:, k] * np.sqrt(12.0)
                dx = sigma * stds * (B @ (D * zk))
                test_pts.append(mean + dx)
                test_pts.append(mean - dx)
            mu, sig = gp.predict(test_pts)"""

new_gp_use = """    if use_gp:
        # Fit local GP on nearest 80 historical models in stds-normalized space (all 12 parameters weighted equally)
        X_arch_norm = (np.array(archive_X) - mean) / stds
        dists = np.linalg.norm(X_arch_norm, axis=1)
        idx = np.argsort(dists)[:80]
        local_X_norm = X_arch_norm[idx]
        # Winsorize crash penalties (>200,000) so GP variance is not distorted by early-stop penalties
        local_y = np.minimum(np.array([archive_y[i] for i in idx], dtype=float), 200000.0)
        
        l_scale = max(0.1, float(np.median(dists[idx])))
        gp = SimpleGP(length_scale=l_scale)
        gp.fit(local_X_norm, local_y)
        
        # Sample candidate orthogonal bases and evaluate direction utility
        best_score = float('inf')
        best_Q = None
        for _ in range(12):
            G = np.random.randn(12, 6)
            G = G - np.outer(u_tpa, u_tpa @ G)
            Q, _ = np.linalg.qr(G)
            
            test_pts_norm = []
            for k in range(6):
                zk = Q[:, k] * np.sqrt(12.0)
                dx_norm = sigma * (B @ (D * zk))
                test_pts_norm.append(dx_norm)
                test_pts_norm.append(-dx_norm)
            mu, sig = gp.predict(test_pts_norm)"""

assert old_gp_use in src, "old_gp_use not found"
src = src.replace(old_gp_use, new_gp_use, 1)

with open("scripts/run_phase2_4_campaign.py", "w", encoding="utf-8", newline="\r\n") as f:
    f.write(src)

print("Patched scripts/run_phase2_4_campaign.py successfully!")
