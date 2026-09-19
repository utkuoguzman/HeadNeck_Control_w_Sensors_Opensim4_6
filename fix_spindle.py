
import re

with open(r"proprioception_plugin\Mileusnic06Spindle.cpp", "r") as f:
    content = f.read()

# 1. Smooth Cb1, Cb2, Cc (ternary operators)
content = content.replace("Cb1 = (Lp>0.0) ? bag1.C_L : bag1.C_S;", "Cb1 = bag1.C_S + (bag1.C_L - bag1.C_S) * 0.5 * (1.0 + std::tanh(1000.0 * Lp));")
content = content.replace("Cb2 = (Lp>0.0) ? bag2.C_L : bag2.C_S;", "Cb2 = bag2.C_S + (bag2.C_L - bag2.C_S) * 0.5 * (1.0 + std::tanh(1000.0 * Lp));")
content = content.replace("Cc\t= (Lp>0.0) ? chain.C_L : chain.C_S;", "Cc = chain.C_S + (chain.C_L - chain.C_S) * 0.5 * (1.0 + std::tanh(1000.0 * Lp));")

# 2. Smooth sign() and abs() in tension derivatives
def smooth_tension(match):
    fiber = match.group(1) # bag1, bag2, chain
    return f"double V_diff_{fiber} = Lp - (Tp/{fiber}.K_SR);\n\tterm1 = C * beta_{fiber} * V_diff_{fiber} * std::pow(std::sqrt(V_diff_{fiber}*V_diff_{fiber} + 1e-4), {fiber}.a - 1.0) * (L - {fiber}.L_0SR - (T/{fiber}.K_SR) - {fiber}.R);"

content = re.sub(r"term1 = C \* beta_([a-z0-9]+) \* sgn\(Lp - \(Tp/[a-z0-9]+\.K_SR\)\) \* std::pow\(std::abs\(Lp - \(Tp/[a-z0-9]+\.K_SR\)\), [a-z0-9]+\.a\) \* \(L - [a-z0-9]+\.L_0SR - \(T/[a-z0-9]+\.K_SR\) - [a-z0-9]+\.R\);", smooth_tension, content)

# 3. Smooth max and min in primary afferent calculation
content = content.replace(
    "primary = max(APbag1, pri_stat) + S * min(APbag1, pri_stat);",
    "double diff = APbag1 - pri_stat;\n\t// Smooth Max and Min\n\tdouble s_max = 0.5 * (APbag1 + pri_stat + std::sqrt(diff*diff + 1e-4));\n\tdouble s_min = 0.5 * (APbag1 + pri_stat - std::sqrt(diff*diff + 1e-4));\n\tprimary = s_max + S * s_min;"
)

# 4. Same for Newton iteration loop
content = content.replace("double sig = (double)sgn(Lp);", "double eps = 1e-4;\n\tdouble sig = Lp / std::sqrt(Lp*Lp + eps);")

with open(r"proprioception_plugin\Mileusnic06Spindle.cpp", "w") as f:
    f.write(content)

