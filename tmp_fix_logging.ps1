import pathlib, re, sys
p = pathlib.Path(r'd:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\cmaes_step_worker.py')
text = p.read_text(encoding='utf-8')
new_block = (
    "                line = (f\"t={t:.2f}s | P:{np.degrees(act_p):5.1f} | R:{np.degrees(act_r):5.1f} | \"\n"
    "                        f\"Y:{np.degrees(act_y):5.1f}\")\n"
    "                print(f\"[{worker_id}] {line}\")\n"
    "                print(line, file=log_f)\n"
    "                log_f.flush()"
)
# Regex to match the old block (including the two print/flush lines)
pattern = r"(?s)                line = \(f\\\"t=\{t:.2f\}s \| P:\{np\.degrees\(act_p\):5\.1f\} \| R:\{np\.degrees\(act_r\):5\.1f\} \| \\"\n                f\\\"Y:\{np\.degrees\(act_y\):5\.1f\}\\\"\)\n                print\(line, file=log_f\)\n                log_f.flush\(\)"
text = re.sub(pattern, new_block, text)
p.write_text(text, encoding='utf-8')
