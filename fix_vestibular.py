
import re

with open(r"vestibular_plugin\Schneider15VestibularAfferent.cpp", "r") as f:
    content = f.read()

new_deadband = """    auto apply_deadband = [](double value, double threshold) -> double {
        double eps = 1e-4;
        double diff_pos = value - threshold;
        double s_max_pos = 0.5 * (diff_pos + std::sqrt(diff_pos*diff_pos + eps));
        
        double diff_neg = -value - threshold;
        double s_max_neg = 0.5 * (diff_neg + std::sqrt(diff_neg*diff_neg + eps));
        
        return s_max_pos - s_max_neg;
    };"""

content = re.sub(r"    auto apply_deadband = \[\]\(double value, double threshold\) -> double \{\s*if \(value > threshold\) return value - threshold;\s*if \(value < -threshold\) return value \+ threshold;\s*return 0\.0;\s*\};", new_deadband, content)

with open(r"vestibular_plugin\Schneider15VestibularAfferent.cpp", "w") as f:
    f.write(content)

