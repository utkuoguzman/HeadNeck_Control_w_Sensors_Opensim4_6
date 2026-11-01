w = open("cmaes_worker_p22.py", "r", encoding="utf-8", newline="").read()
a = "    # Biological Ratio Coupling: r_ton = K_ton / K_sc"
assert w.count(a) == 1
w = w.replace(a, "    # K_ton is optimized directly (no ratio coupling to K_sc)")
b = "    K_ton = raw_p0\r\n    r_ton = K_ton / K_sc"
assert w.count(b) == 1
w = w.replace(b, "    K_ton = raw_p0")
open("cmaes_worker_p22.py", "w", encoding="utf-8", newline="").write(w)

c = open("scripts/run_phase2_4_campaign.py", "r", encoding="utf-8", newline="").read().replace("\r\n", "\n")
for old in ["        r_ton = K_ton / K_sc  # reporting only\n",
            '            "r_ton": float(r_ton),\n',
            '                "r_ton": float(r_ton),\n']:
    assert c.count(old) >= 1, old
    c = c.replace(old, "")
open("scripts/run_phase2_4_campaign.py", "w", encoding="utf-8", newline="").write(c.replace("\n", "\r\n"))
print("done")

