import os
import json
import shutil
import numpy as np
import matplotlib.pyplot as plt

art_dir = r'C:/Users/User/.gemini/antigravity/brain/89fb8d56-9d12-4051-a256-02497ef8d6ae'

def plot_saved_pure_run(worker_label, title_prefix, color_main='#059669'):
    json_path = os.path.join("logs", "candidates_multisine", f"{worker_label}.json")
    with open(json_path) as f:
        data = json.load(f)
    total_score = data["total_score"]
    freq_results = data["frequencies"]
    
    ts_file = os.path.join("logs", "candidates_multisine", f"{worker_label}_ts.npz")
    ts = np.load(ts_file)
    t = ts["t"]
    pitch = ts["pitch"]
    roll = ts["roll"]
    yaw = ts["yaw"]
    
    a_ts = np.load("logs/candidates_multisine/Adaptive_W04_S4_G02_ts.npz")
    ref_p = a_ts["ref_pitch"]
    ref_v = np.gradient(ref_p, t)
    vel = np.gradient(pitch, t)
    
    mask_ss = t >= 3.0
    mean_pitch_ss = float(np.mean(pitch[mask_ss]))
    pitch_3s = float(pitch[np.argmin(np.abs(t - 3.0))])
    rms_vel_ss = float(np.sqrt(np.mean(vel[mask_ss]**2)))
    ref_rms_v = float(np.sqrt(np.mean(ref_v[mask_ss]**2)))
    corr_vel_ss = float(np.corrcoef(vel[mask_ss], ref_v[mask_ss])[0, 1])
    max_roll = float(np.max(np.abs(roll)))
    max_yaw = float(np.max(np.abs(yaw)))
    
    freqs = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]
    tgt_gains = [86.55, 236.93, 319.74, 423.34, 503.30, 572.69]
    tgt_phases = [-7.03, -96.96, -109.23, -161.34, -201.38, -247.62]
    sim_gains = [freq_results[f"{f:.3f}"]["sim_gain"] for f in freqs]
    # Continuous phase unwrapping relative to target phase curve to prevent modulo-360° jump
    sim_phases = []
    for f, tp in zip(freqs, tgt_phases):
        raw_p = freq_results[f"{f:.3f}"]["sim_phase"]
        unwrapped_p = ((raw_p - tp + 180.0) % 360.0) - 180.0 + tp
        sim_phases.append(unwrapped_p)
    
    fig = plt.figure(figsize=(15, 14), dpi=150)
    gs = fig.add_gridspec(4, 1, height_ratios=[1.1, 1.1, 0.85, 1.15], hspace=0.30)
    
    # Subplot 1
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, ref_p, 'k--', lw=1.6, label='Ideal Happee Target Reference Pitch (0° mean)')
    ax1.plot(t, pitch, color=color_main, lw=2.0, label=f'Pure Converged Model [Mean = {mean_pitch_ss:+.2f}°, Pitch(3.0s) = {pitch_3s:+.2f}°]')
    ax1.axvline(1.5, color='#64748b', ls=':', lw=1.3, label='Perturbation Ramp Start (t = 1.5s)')
    ax1.axvline(3.0, color='#dc2626', ls='--', lw=1.4, label='Steady-State Window (t >= 3.0s)')
    ax1.set_xlim(0, 9.17)
    ax1.set_ylabel('Pitch Angle [deg]', fontweight='bold', fontsize=11)
    ax1.set_title(f'1. Head Pitch Angle vs. Time — {title_prefix}', fontweight='bold', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=9)
    
    # Subplot 2
    ax2 = fig.add_subplot(gs[1])
    ax2.axhspan(-50, 50, color='#f1f5f9', alpha=0.9, label=r'Happee Characteristic Operating Envelope ($\pm 50^\circ$/s)')
    ax2.axhline(0, color='#94a3b8', lw=0.8)
    ax2.plot(t, ref_v, 'k--', lw=1.5, alpha=0.85, label=f'Ideal Target Ref Velocity [RMS = {ref_rms_v:.1f}°/s]')
    ax2.plot(t, vel, color=color_main, lw=1.8, label=f'Pure Model Velocity [RMS = {rms_vel_ss:.1f}°/s, corr r = {corr_vel_ss:+.3f}]')
    ax2.axvline(1.5, color='#64748b', ls=':', lw=1.3)
    ax2.axvline(3.0, color='#dc2626', ls='--', lw=1.4)
    ax2.set_xlim(0, 9.17)
    ax2.set_ylim(-110, 115)
    ax2.set_ylabel(r'Pitch Velocity $\dot{\theta}$ [°/s]', fontweight='bold', fontsize=11)
    ax2.set_title(r'2. Head Angular Pitch Velocity vs. Time (Scaled to Happee B8EC [$\pm 100^\circ$/s] Range)', fontweight='bold', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=8.8)
    
    # Subplot 3
    ax3 = fig.add_subplot(gs[2])
    ax3.axhline(0, color='#94a3b8', lw=0.8)
    ax3.plot(t, roll, color='#dc2626', lw=1.4, label=f'Head Roll Error (r1+r2) [Max |Roll| = {max_roll:.2f}°]')
    ax3.plot(t, yaw, color='#15803d', lw=1.4, label=f'Head Yaw Error (y1+y2) [Max |Yaw| = {max_yaw:.2f}°]')
    ax3.axvline(1.5, color='#64748b', ls=':', lw=1.3)
    ax3.axvline(3.0, color='#dc2626', ls='--', lw=1.4)
    ax3.set_xlim(0, 9.17)
    ax3.set_ylim(-1.5, 1.5)
    ax3.set_ylabel('Roll / Yaw [deg]', fontweight='bold', fontsize=11)
    ax3.set_title('3. Head 6-DOF Cross-Axis Stability (Roll & Yaw strictly bounded)', fontweight='bold', fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', fontsize=9)
    
    # Subplot 4
    ax4 = fig.add_subplot(gs[3])
    ax4_r = ax4.twinx()
    ax4.plot(freqs, tgt_gains, 'ko--', lw=1.8, label='Target Bode Gain (Happee 2017) [deg/m]')
    ax4.plot(freqs, sim_gains, 's-', color=color_main, lw=2.2, label='Pure Model Bode Gain [deg/m]')
    ax4_r.plot(freqs, tgt_phases, '^--', color='#92400e', lw=1.8, label='Target Bode Phase (Happee 2017) [deg]')
    ax4_r.plot(freqs, sim_phases, 'D-', color='#b91c1c', lw=2.0, label='Pure Model Bode Phase [deg]')
    
    for f, tg, mg, tp, mp in zip(freqs, tgt_gains, sim_gains, tgt_phases, sim_phases):
        db = 20.0 * np.log10(mg / tg)
        pe = mp - tp
        ax4.annotate(f'{db:+.1f} dB', (f, mg), textcoords='offset points', xytext=(0, 9), ha='center', fontsize=8.5, fontweight='bold', color=color_main)
        ax4_r.annotate(f'{pe:+.1f}°', (f, mp), textcoords='offset points', xytext=(0, -14), ha='center', fontsize=8.5, fontweight='bold', color='#b91c1c')
        
    ax4.set_xscale('log')
    ax4.set_xticks(freqs)
    ax4.set_xticklabels([f'{f:.3f} Hz' for f in freqs], fontweight='bold')
    ax4.set_ylim(0, 660)
    ax4_r.set_ylim(-315, 30)
    ax4.set_ylabel('Bode Gain [deg/m]', color=color_main, fontweight='bold', fontsize=11)
    ax4_r.set_ylabel('Bode Phase [deg]', color='#b91c1c', fontweight='bold', fontsize=11)
    ax4.set_xlabel('Multisine Excitation Frequency [Hz] (Log Scale)', fontweight='bold', fontsize=11)
    ax4.set_title(f'4. Pure 6-Frequency Bode Response — Total Score: {total_score:,.2f} pts', fontweight='bold', fontsize=12)
    ax4.grid(True, which='both', alpha=0.3)
    
    lines1, labels1 = ax4.get_legend_handles_labels()
    lines2, labels2 = ax4_r.get_legend_handles_labels()
    ax4.legend(lines1 + lines2, labels1 + labels2, loc='center left', fontsize=8.5)
    
    fig.suptitle(f'{title_prefix} — Pure Converged Model Performance (Stationary Simulation)', fontweight='bold', fontsize=13, y=0.98)
    
    out_fig_name = f"{worker_label}_4panel.png"
    out_art = os.path.join(art_dir, out_fig_name)
    fig.savefig(out_art, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved figure to artifact: {out_art}", flush=True)
    
    ws_out = os.path.join(".", out_fig_name)
    shutil.copyfile(out_art, ws_out)
    print(f"Copied figure to workspace root: {ws_out}", flush=True)
    return total_score, out_art, ws_out

if __name__ == "__main__":
    plot_saved_pure_run("pure_converged_adaptation_run1", "Adaptation Run 1 (Pure Converged Parameters)")
