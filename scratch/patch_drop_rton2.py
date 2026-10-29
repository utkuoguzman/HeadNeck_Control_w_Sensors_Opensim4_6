import re
p = "scripts/run_phase2_4_campaign.py"
c = open(p, "r", encoding="utf-8", newline="").read().replace("\r\n", "\n")
n0 = c.count("r_ton")
c = re.sub(r'^[ \t]*r_ton = K_ton / K_sc  # reporting only\n', '', c, flags=re.M)
c = re.sub(r'^[ \t]*"r_ton": float\(r_ton\),\n', '', c, flags=re.M)
print("r_ton mentions:", n0, "->", c.count("r_ton"))
open(p, "w", encoding="utf-8", newline="").write(c.replace("\n", "\r\n"))

