import re

# 1. Patch cmaes_worker_p22.py
worker_path = 'cmaes_worker_p22.py'
with open(worker_path, 'r', encoding='utf-8') as f:
    worker_code = f.read()

pattern_worker = r'coord_p1\s*=\s*model\.getCoordinateSet\(\)\.get\("pitch1"\).*?pitch_history\.append\(head_pitch_deg\)'

replacement_worker = """coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")
    coord_y2 = model.getCoordinateSet().get("yaw2")

    t_history = []
    pitch_history = []

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        p1_deg = math.degrees(coord_p1.getValue(state))
        p2_deg = math.degrees(coord_p2.getValue(state))
        r1_deg = math.degrees(coord_r1.getValue(state))
        r2_deg = math.degrees(coord_r2.getValue(state))
        y1_deg = math.degrees(coord_y1.getValue(state))
        y2_deg = math.degrees(coord_y2.getValue(state))

        head_pitch_deg = p1_deg + p2_deg
        head_roll_deg  = r1_deg + r2_deg
        head_yaw_deg   = y1_deg + y2_deg

        # Premature stopping guard: Transition Zone Entry (End of Deadband)
        # If any joint coordinate leaves its zero-force deadband and enters its respective transition zone,
        # terminate immediately because posture control is weak or unstable.
        # Deadband boundaries (75% of anatomical voluntary ROM):
        #   pitch1: [-12.0°, +18.0°]
        #   pitch2: [-24.75°, +36.0°]
        #   head_pitch: [-36.75°, +54.0°]
        #   roll1:  ±4.5°
        #   roll2:  ±24.75°
        #   head_roll: ±29.25°
        #   yaw1:   ±28.5°
        #   yaw2:   ±20.25°
        #   head_yaw: ±48.75°
        if (p1_deg < -12.0 or p1_deg > 18.0 or
            p2_deg < -24.75 or p2_deg > 36.0 or
            head_pitch_deg < -36.75 or head_pitch_deg > 54.0 or
            abs(r1_deg) > 4.5 or abs(r2_deg) > 24.75 or abs(head_roll_deg) > 29.25 or
            abs(y1_deg) > 28.5 or abs(y2_deg) > 20.25 or abs(head_yaw_deg) > 48.75):
            penalty = 1000000.0 + (target_time - current_time) * 10000.0
            print(f"[ABORT TRANSITION] {worker_name} entered transition zone at t={current_time:.2f}s "
                  f"(p1={p1_deg:.1f}°, p2={p2_deg:.1f}°, pitch={head_pitch_deg:.1f}°, "
                  f"r1={r1_deg:.1f}°, r2={r2_deg:.1f}°, roll={head_roll_deg:.1f}°, "
                  f"y1={y1_deg:.1f}°, y2={y2_deg:.1f}°, yaw={head_yaw_deg:.1f}°)", file=sys.stderr)
            return 50000.0, 0.0

        t_history.append(current_time)
        pitch_history.append(head_pitch_deg)"""

match = re.search(pattern_worker, worker_code, re.DOTALL)
if match:
    worker_code = worker_code[:match.start()] + replacement_worker + worker_code[match.end():]
    with open(worker_path, 'w', encoding='utf-8') as f:
        f.write(worker_code)
    print("Successfully patched cmaes_worker_p22.py with transition abort logic.")
else:
    print("ERROR: Could not find target pattern in cmaes_worker_p22.py")

# 2. Patch cervical_fast_sim.cpp
fast_sim_path = 'HeadNeckNeuralController_plugin/cervical_fast_sim.cpp'
with open(fast_sim_path, 'r', encoding='utf-8') as f:
    fast_code = f.read()

pattern_fast = r'const Coordinate& coord_p1 = model\.getCoordinateSet\(\)\.get\("pitch1"\);.*?pitch_history\.push_back\(head_pitch_deg\);'

replacement_fast = """const Coordinate& coord_p1 = model.getCoordinateSet().get("pitch1");
    const Coordinate& coord_p2 = model.getCoordinateSet().get("pitch2");
    const Coordinate& coord_r1 = model.getCoordinateSet().get("roll1");
    const Coordinate& coord_r2 = model.getCoordinateSet().get("roll2");
    const Coordinate& coord_y1 = model.getCoordinateSet().get("yaw1");
    const Coordinate& coord_y2 = model.getCoordinateSet().get("yaw2");

    std::vector<double> t_history;
    std::vector<double> pitch_history;

    bool diverged = false;
    while (current_time < target_time) {
        current_time = std::min(current_time + step, target_time);
        const SimTK::State& s = manager.integrate(current_time);

        double p1 = coord_p1.getValue(s);
        double p2 = coord_p2.getValue(s);
        double r1 = coord_r1.getValue(s);
        double r2 = coord_r2.getValue(s);
        double y1 = coord_y1.getValue(s);
        double y2 = coord_y2.getValue(s);

        double p1_deg = p1 * (180.0 / SimTK::Pi);
        double p2_deg = p2 * (180.0 / SimTK::Pi);
        double r1_deg = r1 * (180.0 / SimTK::Pi);
        double r2_deg = r2 * (180.0 / SimTK::Pi);
        double y1_deg = y1 * (180.0 / SimTK::Pi);
        double y2_deg = y2 * (180.0 / SimTK::Pi);

        double head_pitch_deg = p1_deg + p2_deg;
        double head_roll_deg  = r1_deg + r2_deg;
        double head_yaw_deg   = y1_deg + y2_deg;

        // Premature stopping guard: Transition Zone Entry (End of Deadband)
        if (p1_deg < -12.0 || p1_deg > 18.0 ||
            p2_deg < -24.75 || p2_deg > 36.0 ||
            head_pitch_deg < -36.75 || head_pitch_deg > 54.0 ||
            std::abs(r1_deg) > 4.5 || std::abs(r2_deg) > 24.75 || std::abs(head_roll_deg) > 29.25 ||
            std::abs(y1_deg) > 28.5 || std::abs(y2_deg) > 20.25 || std::abs(head_yaw_deg) > 48.75) {
            diverged = true;
            break;
        }

        t_history.push_back(current_time);
        pitch_history.push_back(head_pitch_deg);"""

match_fast = re.search(pattern_fast, fast_code, re.DOTALL)
if match_fast:
    fast_code = fast_code[:match_fast.start()] + replacement_fast + fast_code[match_fast.end():]
    with open(fast_sim_path, 'w', encoding='utf-8') as f:
        f.write(fast_code)
    print("Successfully patched cervical_fast_sim.cpp with transition abort logic.")
else:
    print("ERROR: Could not find target pattern in cervical_fast_sim.cpp")
