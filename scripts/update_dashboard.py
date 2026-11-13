import base64
import os

art_dir = r"C:/Users/User/.gemini/antigravity/brain/89fb8d56-9d12-4051-a256-02497ef8d6ae"

def b64(fn):
    p = os.path.join(art_dir, fn)
    if os.path.exists(p):
        with open(p, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")
    return ""

b64_run4 = b64("pure_converged_adaptation_run4_21s_4panel.png")
b64_run3 = b64("pure_converged_adaptation_run3_21s_4panel.png")
b64_run2 = b64("pure_converged_adaptation_run2_21s_4panel.png")
b64_run1 = b64("pure_converged_adaptation_run1_21s_4panel.png")
b64_pre  = b64("pre_adaptive_champion_w04_s4_g02_4panel.png")
b64_vel  = b64("velocity_deg_per_sec_happee_comparison.png")

html = f"""<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Adaptive Multisine Progression & Extended Stationary Benchmarks</title>
  <script src="https://www.gstatic.com/antigravity/web/dev/tailwindcss.min.js"></script>
  <style>
    .tab-btn.active {{
      background: #059669 !important;
      color: #ffffff !important;
      font-weight: 600;
    }}
  </style>
</head>
<body class="bg-transparent text-[var(--foreground,#0f172a)] antialiased p-3 font-sans">
  <div class="bg-[var(--card,#ffffff)] text-[var(--foreground,#0f172a)] border border-[var(--border,#e2e8f0)] rounded-xl p-4 shadow-sm max-w-6xl mx-auto">
    <div class="flex flex-wrap items-center justify-between gap-3 mb-4 pb-3 border-b border-[var(--border,#e2e8f0)]">
      <div>
        <h2 class="font-bold text-lg flex items-center gap-2">
          <span>🚀</span> Adaptation Progression — 21.52s Pure Stationary Simulations
        </h2>
        <p class="text-xs text-slate-500 mt-0.5">
          Pure stationary evaluations matching Happee (2017) trial duration (21.52s, 6 cycles) &bull; Zero time-constant &amp; 1.099 Hz gain progression
        </p>
      </div>
      <div class="flex flex-wrap items-center gap-1.5 bg-slate-100 p-1 rounded-lg border border-slate-200">
        <button id="btn-tab-1" onclick="showTab(1)" class="tab-btn active px-3 py-1.5 rounded-md text-xs transition-colors">
          &starf; Run 4 (Pure 21.5s — 991 pts)
        </button>
        <button id="btn-tab-2" onclick="showTab(2)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          Run 3 (2,713 pts)
        </button>
        <button id="btn-tab-3" onclick="showTab(3)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          Run 2 (3,791 pts)
        </button>
        <button id="btn-tab-4" onclick="showTab(4)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          Run 1 (3,234 pts)
        </button>
        <button id="btn-tab-5" onclick="showTab(5)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          Pre-Adaptive (W04)
        </button>
        <button id="btn-tab-6" onclick="showTab(6)" class="tab-btn px-3 py-1.5 rounded-md text-xs text-slate-600 hover:text-slate-900 transition-colors">
          Velocity vs Happee B8EC
        </button>
      </div>
    </div>

    <!-- Tab 1: Run 4 Pure 21.5s Simulation -->
    <div id="tab1-content" class="space-y-3">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,{b64_run4}" class="max-w-full rounded shadow-sm" alt="Adaptation Run 4 Pure 21.5s 4-Panel Plot" />
      </div>
      <div class="grid grid-cols-2 md:grid-cols-4 gap-2 text-xs">
        <div class="bg-emerald-50 border border-emerald-200 p-2.5 rounded-lg">
          <span class="font-bold text-emerald-900 block">1.099 Hz Target Surpassed</span>
          <span class="text-emerald-700 text-sm font-black">285.3 deg/m (-0.99 dB)</span>
          <span class="text-slate-500 block text-[11px]">From -2.44 dB (Run 1) &rarr; -1.40 dB (Run 3) &rarr; -0.99 dB</span>
        </div>
        <div class="bg-purple-50 border border-purple-200 p-2.5 rounded-lg">
          <span class="font-bold text-purple-900 block">High-Freq Phase Recovered</span>
          <span class="text-purple-700 text-sm font-black">7.93Hz: 3.8&deg; | 5.58Hz: 16.9&deg;</span>
          <span class="text-slate-500 block text-[11px]">Cut from 28.4&deg; &amp; 28.7&deg; via zero time-constant (z_sc=5.54ms)</span>
        </div>
        <div class="bg-blue-50 border border-blue-200 p-2.5 rounded-lg">
          <span class="font-bold text-blue-900 block">Mid &amp; High Gain Precision</span>
          <span class="text-blue-700 text-sm font-black">0.78Hz: -0.06dB | 7.93Hz: +0.07dB</span>
          <span class="text-slate-500 block text-[11px]">Zero 6-DOF roll/yaw or HF tremor penalty (hf=0.0)</span>
        </div>
        <div class="bg-amber-50 border border-amber-200 p-2.5 rounded-lg">
          <span class="font-bold text-amber-900 block">Sub-1,000 Pure 21.5s Score</span>
          <span class="text-amber-700 text-sm font-black">991.11 pts</span>
          <span class="text-slate-500 block text-[11px]">63.5% better than Run 3 (2,713 pts) &bull; 3.26&times; better than Run 1</span>
        </div>
      </div>
    </div>

    <!-- Tab 2: Run 3 Pure 21.5s Simulation -->
    <div id="tab2-content" class="space-y-3 hidden">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,{b64_run3}" class="max-w-full rounded shadow-sm" alt="Adaptation Run 3 Pure 21.5s 4-Panel Plot" />
      </div>
      <div class="bg-slate-100 p-3 rounded-lg text-xs text-slate-700">
        <strong>Run 3 Stationary Evaluation:</strong> 1.099 Hz gain at 272.0 deg/m (-1.40 dB), high-freq phase errors 28.7&deg; (5.58 Hz) &amp; 28.4&deg; (7.93 Hz). Total score: 2,713.21 pts.
      </div>
    </div>

    <!-- Tab 3: Run 2 Pure 21.5s Simulation -->
    <div id="tab3-content" class="space-y-3 hidden">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,{b64_run2}" class="max-w-full rounded shadow-sm" alt="Adaptation Run 2 Pure 21.5s 4-Panel Plot" />
      </div>
      <div class="bg-slate-100 p-3 rounded-lg text-xs text-slate-700">
        <strong>Run 2 Stationary Evaluation:</strong> 1.099 Hz gain recovered to 263.0 deg/m (-1.70 dB). Total score: 3,790.61 pts.
      </div>
    </div>

    <!-- Tab 4: Run 1 Pure 21.5s Simulation -->
    <div id="tab4-content" class="space-y-3 hidden">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,{b64_run1}" class="max-w-full rounded shadow-sm" alt="Adaptation Run 1 Pure 21.5s 4-Panel Plot" />
      </div>
      <div class="bg-slate-100 p-3 rounded-lg text-xs text-slate-700">
        <strong>Run 1 Stationary Evaluation:</strong> 1.099 Hz was over-damped at 241.4 deg/m (-2.44 dB) due to excess velocity feedback. Total score: 3,234.15 pts.
      </div>
    </div>

    <!-- Tab 5: Pre-Adaptive Champion (W04_S4_G02) -->
    <div id="tab5-content" class="space-y-3 hidden">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,{b64_pre}" class="max-w-full rounded shadow-sm" alt="Pre-Adaptive Champion W04 4-Panel Plot" />
      </div>
    </div>

    <!-- Tab 6: Head Pitch Velocity Comparison vs. Happee -->
    <div id="tab6-content" class="space-y-3 hidden">
      <div class="bg-slate-50 rounded-lg p-2 border border-slate-200 flex justify-center">
        <img src="data:image/png;base64,{b64_vel}" class="max-w-full rounded shadow-sm" alt="Velocity deg/s Comparison vs Happee B8EC" />
      </div>
    </div>
  </div>

  <script>
    function showTab(n) {{
      for (let i = 1; i <= 6; i++) {{
        const el = document.getElementById('tab' + i + '-content');
        const btn = document.getElementById('btn-tab-' + i);
        if (el) el.classList.add('hidden');
        if (btn) {{
          btn.classList.remove('active');
          btn.classList.add('text-slate-600');
        }}
      }}
      const sel = document.getElementById('tab' + n + '-content');
      const selBtn = document.getElementById('btn-tab-' + n);
      if (sel) sel.classList.remove('hidden');
      if (selBtn) {{
        selBtn.classList.add('active');
        selBtn.classList.remove('text-slate-600');
      }}
    }}
  </script>
</body>
</html>"""

out_html = os.path.join(art_dir, "adaptive_plots_viewer.html")
with open(out_html, "w", encoding="utf-8") as f:
    f.write(html)
print("Successfully wrote", out_html)
