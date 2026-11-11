import base64
import os

art_dir = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
p1 = os.path.join(art_dir, "pre_adaptive_champion_w04_s4_g02_4panel.png")
p2 = os.path.join(art_dir, "current_adaptive_model_w04_s4_g02_4panel.png")
p3 = os.path.join(art_dir, "velocity_deg_per_sec_happee_comparison.png")

with open(p1, "rb") as f:
    b64_1 = base64.b64encode(f.read()).decode("utf-8")
with open(p2, "rb") as f:
    b64_2 = base64.b64encode(f.read()).decode("utf-8")
with open(p3, "rb") as f:
    b64_3 = base64.b64encode(f.read()).decode("utf-8")

html = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Pre-Adaptive vs Current Adaptive Controller Comparison</title>
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
          <span>📊</span> Pre-Adaptive Champion vs. Current Adaptive Controller
        </h2>
        <p class="text-xs text-slate-500 mt-0.5">
          4-Panel Analysis: Pitch Angle &bull; Angular Velocity (deg/s) &bull; Roll/Yaw Cross-Axis &bull; 6-Frequency Bode
        </p>
      </div>
      <div class="flex items-center gap-1.5 bg-slate-100 p-1 rounded-lg border border-slate-200">
        <button id="btn-tab-pre" onclick="showTab(1)" class="tab-btn active px-3 py-1.5 rounded-md text-xs transition-colors">
          1. Pre-Adaptive Champion (W04_S4_G02)
        </button>
        <button id="btn-tab-curr" onclick="showTab(2)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          2. Current Adaptive Controller
        </button>
        <button id="btn-tab-vel" onclick="showTab(3)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          3. Velocity vs Happee B8EC
        </button>
      </div>
    </div>

    <!-- Tab 1: Pre-Adaptive Champion -->
    <div id="tab1-content" class="space-y-3">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,""" + b64_1 + """" alt="Pre-Adaptive Champion 4-Panel" class="w-full max-h-[85vh] object-contain rounded-md shadow-sm border border-slate-200 bg-white" />
      </div>
      <div class="grid grid-cols-1 md:grid-cols-4 gap-2 text-xs">
        <div class="p-2.5 bg-blue-50 border border-blue-200 rounded">
          <strong class="text-blue-900 block mb-0.5">1. Pitch Angle</strong>
          <span class="text-blue-800">Pitch(3.0s) = +3.39°, Mean = +2.45°. Delayed settling transient until ~4.5s.</span>
        </div>
        <div class="p-2.5 bg-slate-50 border border-slate-200 rounded">
          <strong class="text-slate-900 block mb-0.5">2. Pitch Velocity</strong>
          <span class="text-slate-700">RMS = 32.4°/s (Ref: 31.0°/s). Peaks reach +90.4°/s and -75.5°/s.</span>
        </div>
        <div class="p-2.5 bg-emerald-50 border border-emerald-200 rounded">
          <strong class="text-emerald-900 block mb-0.5">3. Roll / Yaw</strong>
          <span class="text-emerald-800">|Roll| &le; 0.80°, |Yaw| &le; 0.98°. Zero coronal/axial instability.</span>
        </div>
        <div class="p-2.5 bg-purple-50 border border-purple-200 rounded">
          <strong class="text-purple-900 block mb-0.5">4. Bode Score</strong>
          <span class="text-purple-800">Score = <strong>1,638.71 pts</strong> (Top 4 frequencies &le; 1 dB).</span>
        </div>
      </div>
    </div>

    <!-- Tab 2: Current Adaptive Controller -->
    <div id="tab2-content" class="hidden space-y-3">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,""" + b64_2 + """" alt="Current Adaptive Model 4-Panel" class="w-full max-h-[85vh] object-contain rounded-md shadow-sm border border-slate-200 bg-white" />
      </div>
      <div class="grid grid-cols-1 md:grid-cols-4 gap-2 text-xs">
        <div class="p-2.5 bg-emerald-50 border border-emerald-200 rounded">
          <strong class="text-emerald-900 block mb-0.5">1. Pitch Angle</strong>
          <span class="text-emerald-800">Pitch(3.0s) = <strong>+1.65°</strong> (scrubbed by 1.74°), Mean = <strong>+1.36°</strong>.</span>
        </div>
        <div class="p-2.5 bg-slate-50 border border-slate-200 rounded">
          <strong class="text-slate-900 block mb-0.5">2. Pitch Velocity</strong>
          <span class="text-slate-700">RMS = <strong>31.1°/s</strong> (exact match with Ref: 31.0°/s). corr r = <strong>+0.866</strong>.</span>
        </div>
        <div class="p-2.5 bg-blue-50 border border-blue-200 rounded">
          <strong class="text-blue-900 block mb-0.5">3. Roll / Yaw</strong>
          <span class="text-blue-800">|Roll| &le; 0.66°, |Yaw| &le; 0.93°. Cross-axis drift reduced by ~15%.</span>
        </div>
        <div class="p-2.5 bg-purple-50 border border-purple-200 rounded">
          <strong class="text-purple-900 block mb-0.5">4. Bode Score</strong>
          <span class="text-purple-800">Score = <strong>2,313.67 pts</strong> (0.783 Hz = 85.7 pts, 2.446 Hz = 28.6 pts).</span>
        </div>
      </div>
    </div>

    <!-- Tab 3: Velocity vs Happee B8EC -->
    <div id="tab3-content" class="hidden space-y-3">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,""" + b64_3 + """" alt="Velocity deg per sec comparison" class="w-full max-h-[85vh] object-contain rounded-md shadow-sm border border-slate-200 bg-white" />
      </div>
    </div>
  </div>

  <script>
    function showTab(tabIndex) {
      const t1 = document.getElementById('tab1-content');
      const t2 = document.getElementById('tab2-content');
      const t3 = document.getElementById('tab3-content');
      const b1 = document.getElementById('btn-tab-pre');
      const b2 = document.getElementById('btn-tab-curr');
      const b3 = document.getElementById('btn-tab-vel');

      [b1, b2, b3].forEach(b => b.classList.remove('active'));
      [t1, t2, t3].forEach(t => t.classList.add('hidden'));

      if (tabIndex === 1) {
        t1.classList.remove('hidden');
        b1.classList.add('active');
      } else if (tabIndex === 2) {
        t2.classList.remove('hidden');
        b2.classList.add('active');
      } else if (tabIndex === 3) {
        t3.classList.remove('hidden');
        b3.classList.add('active');
      }
    }
  </script>
</body>
</html>"""

out_file = os.path.join(art_dir, "adaptive_plots_viewer.html")
with open(out_file, "w", encoding="utf-8") as f:
    f.write(html)
print("Updated viewer with both 4-panel figures:", out_file)
