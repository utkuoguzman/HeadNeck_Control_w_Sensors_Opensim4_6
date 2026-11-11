import base64
import os

art_dir = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
p1 = os.path.join(art_dir, "adaptive_w04_s4_g02_tracking_and_bode.png")
p2 = os.path.join(art_dir, "adaptive_w04_s4_g02_convergence.png")

with open(p1, "rb") as f:
    b64_1 = base64.b64encode(f.read()).decode("utf-8")
with open(p2, "rb") as f:
    b64_2 = base64.b64encode(f.read()).decode("utf-8")

html = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Adaptive Controller Simulation Plots</title>
  <script src="https://www.gstatic.com/antigravity/web/dev/tailwindcss.min.js"></script>
  <style>
    .tab-btn.active {
      background: #2563eb !important;
      color: #ffffff !important;
      font-weight: 600;
    }
  </style>
</head>
<body class="bg-transparent text-[var(--foreground,#0f172a)] antialiased p-3 font-sans">
  <div class="bg-[var(--card,#ffffff)] text-[var(--foreground,#0f172a)] border border-[var(--border,#e2e8f0)] rounded-xl p-4 shadow-sm max-w-6xl mx-auto">
    <div class="flex flex-wrap items-center justify-between gap-3 mb-4 pb-3 border-b border-[var(--border,#e2e8f0)]">
      <div>
        <h2 class="font-bold text-lg flex items-center gap-2">
          <span>📊</span> Synced Adaptive Controller Plots (W04_S4_G02 Seed)
        </h2>
        <p class="text-xs text-slate-500 mt-0.5">
          Evaluation on 11D CMA-ES Champion | t_adapt = 1.8s &rarr; 3.0s | True 6-DOF OpenSim Dynamics
        </p>
      </div>
      <div class="flex items-center gap-2 bg-slate-100 p-1 rounded-lg border border-slate-200">
        <button id="btn-tab1" onclick="showTab(1)" class="tab-btn active px-3 py-1.5 rounded-md text-xs transition-colors">
          1. Tracking &amp; Bode Diagram
        </button>
        <button id="btn-tab2" onclick="showTab(2)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          2. Parameter Convergence
        </button>
        <button id="btn-tab-both" onclick="showTab(3)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          Both (Stacked)
        </button>
      </div>
    </div>

    <!-- Tab 1: Tracking & Bode -->
    <div id="tab1-content" class="space-y-3">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,""" + b64_1 + """" alt="Adaptive W04_S4_G02 Tracking and Bode" class="w-full max-h-[85vh] object-contain rounded-md shadow-sm border border-slate-200 bg-white" />
      </div>
      <div class="grid grid-cols-1 md:grid-cols-3 gap-2 text-xs">
        <div class="p-2.5 bg-slate-50 rounded border border-slate-200">
          <span class="font-semibold text-blue-600">Top Panel:</span> Pitch Tracking scrubbed from +3.39° down to <strong>+1.65° at t=3.0s</strong>, achieving stationary t=4.5s fidelity early.
        </div>
        <div class="p-2.5 bg-slate-50 rounded border border-slate-200">
          <span class="font-semibold text-emerald-600">Middle Panel:</span> 6-DOF Cross-Axis Roll/Yaw strictly bounded (|Roll| &lt; 0.66°, |Yaw| &lt; 0.93°).
        </div>
        <div class="p-2.5 bg-slate-50 rounded border border-slate-200">
          <span class="font-semibold text-red-600">Bottom Panel:</span> 6-Frequency Bode curve closely matches experimental targets (2,313.7 pts).
        </div>
      </div>
    </div>

    <!-- Tab 2: Parameter Convergence -->
    <div id="tab2-content" class="hidden space-y-3">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,""" + b64_2 + """" alt="Adaptive Parameter Convergence" class="w-full max-h-[85vh] object-contain rounded-md shadow-sm border border-slate-200 bg-white" />
      </div>
      <div class="p-2.5 bg-slate-50 rounded border border-slate-200 text-xs text-slate-600">
        <strong>Parameter Trajectory:</strong> Gains adapt smoothly during t &isin; [1.8, 3.0]s and freeze stably at t = 3.0s with zero drift.
      </div>
    </div>
  </div>

  <script>
    function showTab(tabIndex) {
      const t1 = document.getElementById('tab1-content');
      const t2 = document.getElementById('tab2-content');
      const b1 = document.getElementById('btn-tab1');
      const b2 = document.getElementById('btn-tab2');
      const bb = document.getElementById('btn-tab-both');

      [b1, b2, bb].forEach(b => b.classList.remove('active'));

      if (tabIndex === 1) {
        t1.classList.remove('hidden');
        t2.classList.add('hidden');
        b1.classList.add('active');
      } else if (tabIndex === 2) {
        t1.classList.add('hidden');
        t2.classList.remove('hidden');
        b2.classList.add('active');
      } else {
        t1.classList.remove('hidden');
        t2.classList.remove('hidden');
        bb.classList.add('active');
      }
    }
  </script>
</body>
</html>"""

out_file = os.path.join(art_dir, "adaptive_plots_viewer.html")
with open(out_file, "w", encoding="utf-8") as f:
    f.write(html)
print(f"Viewer written successfully: {out_file} ({os.path.getsize(out_file) / 1024:.1f} KB)")
