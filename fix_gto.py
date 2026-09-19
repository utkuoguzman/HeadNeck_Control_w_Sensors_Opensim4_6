
import re

with open(r"proprioception_plugin\Lin02GolgiTendonOrgan.cpp", "r") as f:
    content = f.read()

# 1. Replace computeStateVariableDerivatives
new_compute = """void Lin02GolgiTendonOrgan::computeStateVariableDerivatives(const SimTK::State& s) const
{
	double non_lin = Gg*std::log((musclePtr->getFiberForce(s) / Gf) + 1);
	
	// Cascaded filter for smooth derivatives without finite-differencing
	double curr_diff0 = (non_lin - getX(s)) / getLPFtau();
	double curr_diff1 = (curr_diff0 - getXp(s)) / getLPFtau();
	
	setStateVariableDerivativeValue(s, "nonlinear", curr_diff0);
	setStateVariableDerivativeValue(s, "nonlinear_deriv", curr_diff1);
	
	double Xpp = curr_diff1;
	
	setStateVariableDerivativeValue(s, "filter_out", getZ(s));
	setStateVariableDerivativeValue(s, "filter_out_deriv", -2.2*getZ(s) - 0.4*getY(s) + 68.0*Xpp + 103.2*getXp(s) + 16.0*getX(s));
			  
	// Smooth Max for GTO output
	double y_diff = getY(s) - thr;
	double smooth_out = 0.5 * (getY(s) + thr + std::sqrt(y_diff*y_diff + 1e-4));
	setGTOout(s, smooth_out);
}
"""

content = re.sub(r"void Lin02GolgiTendonOrgan::computeStateVariableDerivatives\(const SimTK::State& s\) const\s*\{.*?\n\}\n", new_compute, content, flags=re.DOTALL)

# 2. Replace calculateDerivatives
new_calc = """SimTK::Vec<2> Lin02GolgiTendonOrgan::calculateDerivatives(const SimTK::State& s) const
{
	SimTK::Vec<2> diff;
	double non_lin = Gg*std::log((musclePtr->getFiberForce(s) / Gf) + 1);
	diff(0) = (non_lin - getX(s)) / getLPFtau();
	diff(1) = (diff(0) - getXp(s)) / getLPFtau();
	return diff;
}
"""

content = re.sub(r"SimTK::Vec<2> Lin02GolgiTendonOrgan::\s*calculateDerivatives\(const SimTK::State& s\) const\s*\{.*?\n\}\n", new_calc, content, flags=re.DOTALL)

# 3. Clean up initFromMuscle
content = re.sub(r"// the work variables too\s*nl = 0; Dnl = 0;\s*ts\[2\] = 0\.0; ts\[1\] = -0\.01; ts\[0\] = -0\.02;\s*C0 = 0\.0; C1 = 0\.0;", "// removed historical variables", content)
content = re.sub(r"nl = 0; Dnl = 0;\s*ts\[2\] = 0\.0; ts\[1\] = -0\.01; ts\[0\] = -0\.02;\s*C0 = 0\.0; C1 = 0\.0;", "// removed historical variables", content)

with open(r"proprioception_plugin\Lin02GolgiTendonOrgan.cpp", "w") as f:
    f.write(content)

with open(r"proprioception_plugin\Lin02GolgiTendonOrgan.h", "r") as f:
    header = f.read()
header = re.sub(r"mutable SimTK::Vec<3> nl, Dnl, ts;\s*mutable SimTK::Mat33 C0;\s*mutable SimTK::Mat44 C1;", "// no mutable arrays", header)
with open(r"proprioception_plugin\Lin02GolgiTendonOrgan.h", "w") as f:
    f.write(header)

