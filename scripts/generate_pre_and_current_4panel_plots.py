import os
import json
import shutil
import base64
import numpy as np
import matplotlib.pyplot as plt

art_dir = r'C:/Users/User/.gemini/antigravity/brain/89fb8d56-9d12-4051-a256-02497ef8d6ae'

# 1. Load data
w = np.load('logs/candidates_multisine/W04_S4_G02_ts.npz')
a = np.load('logs/candidates_multisine/Adaptive_W04_S4_G02_ts.npz')

with open('logs/candidates_multisine/W04_S4_G02.json') as f:
    jw = json.load(f)
with open('logs/adaptive_updated_15params_trajectory.json') as f:
    ja = json.load(f)

t = a['t']
ref_p = a['ref_pitch']
ref_v = np.gradient(ref_p, t)

freqs = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]
tgt_gains = [86.55, 236.93, 319.74, 423.34, 503.30, 572.69]
tgt_phases = [-7.03, -96.96, -109.23, -161.34, -201.38, -247.62]

mask_ss = t >= 3.0
ref_rms_v = np.sqrt(np.mean(ref_v[mask_ss]**2))

def make_4panel_figure(title_prefix, model_label, pitch, roll, yaw, gains, phases, total_score, color_main, out_name):
    vel = np.gradient(pitch, t)
    
    mean_pitch_ss = np.mean(pitch[mask_ss])
    pitch_3s = pitch[np.argmin(np.abs(t - 3.0))]
    rms_vel_ss = np.sqrt(np.mean(vel[mask_ss]**2))
    corr_vel_ss = np.corrcoef(vel[mask_ss], ref_v[mask_ss])[0, 1]
    max_roll = np.max(np.abs(roll))
    max_yaw = np.max(np.abs(yaw))
    
    fig = plt.figure(figsize=(15, 14), dpi=150)
    gs = fig.add_gridspec(4, 1, height_ratios=[1.1, 1.1, 0.85, 1.15], hspace=0.30)
    
    # -------------------------------------------------------------
    # Subplot 1: Head Pitch Angle vs Time
    # -------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, ref_p, 'k--', lw=1.6, label='Ideal Happee Target Reference Pitch (0° mean)')
    ax1.plot(t, pitch, color=color_main, lw=2.0, label=f'{model_label} [Mean = {mean_pitch_ss:+.2f}°, Pitch(3.0s) = {pitch_3s:+.2f}°]')
    ax1.axvline(1.5, color='#64748b', ls=':', lw=1.3, label='Perturbation Ramp Start (t = 1.5s)')
    ax1.axvline(3.0, color='#dc2626', ls='--', lw=1.4, label='Steady-State Evaluation Window (t >= 3.0s)')
    ax1.set_xlim(0, 9.17)
    ax1.set_ylabel('Pitch Angle [deg]', fontweight='bold', fontsize=11)
    ax1.set_title(f'1. Head Pitch Angle vs. Time — {title_prefix}', fontweight='bold', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=9)
    
    # -------------------------------------------------------------
    # Subplot 2: Angular Velocity vs Time
    # -------------------------------------------------------------
    ax2 = fig.add_subplot(gs[1])
    ax2.axhspan(-50, 50, color='#f1f5f9', alpha=0.9, label=r'Happee Characteristic Operating Envelope ($\pm 50^\circ$/s)')
    ax2.axhline(0, color='#94a3b8', lw=0.8)
    ax2.plot(t, ref_v, 'k--', lw=1.5, alpha=0.85, label=f'Ideal Target Ref Velocity [RMS = {ref_rms_v:.1f}°/s]')
    ax2.plot(t, vel, color=color_main, lw=1.8, label=f'{model_label} Velocity [RMS = {rms_vel_ss:.1f}°/s, corr r = {corr_vel_ss:+.3f}]')
    ax2.axvline(1.5, color='#64748b', ls=':', lw=1.3)
    ax2.axvline(3.0, color='#dc2626', ls='--', lw=1.4)
    ax2.set_xlim(0, 9.17)
    ax2.set_ylim(-110, 115)
    ax2.set_ylabel(r'Pitch Velocity $\dot{\theta}$ [°/s]', fontweight='bold', fontsize=11)
    ax2.set_title(r'2. Head Angular Pitch Velocity vs. Time (Scaled to Happee B8EC [$\pm 100^\circ$/s] Range)', fontweight='bold', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=8.8)
    
    # -------------------------------------------------------------
    # Subplot 3: Cross-Axis Roll & Yaw Angles vs Time
    # -------------------------------------------------------------
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
    
    # -------------------------------------------------------------
    # Subplot 4: 6-Frequency Bode Response (Gain & Phase)
    # -------------------------------------------------------------
    ax4 = fig.add_subplot(gs[3])
    ax4_r = ax4.twinx()
    
    ax4.plot(freqs, tgt_gains, 'ko--', lw=1.8, label='Target Bode Gain (Happee 2017) [deg/m]')
    ax4.plot(freqs, gains, 's-', color=color_main, lw=2.2, label=f'{model_label} Bode Gain [deg/m]')
    
    ax4_r.plot(freqs, tgt_phases, '^--', color='#92400e', lw=1.8, label='Target Bode Phase (Happee 2017) [deg]')
    ax4_r.plot(freqs, phases, 'D-', color='#b91c1c', lw=2.0, label=f'{model_label} Bode Phase [deg]')
    
    for f, tg, mg, tp, mp in zip(freqs, tgt_gains, gains, tgt_phases, phases):
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
    ax4.set_title(f'4. 6-Frequency Bode Response — Total Score: {total_score:,.2f} pts', fontweight='bold', fontsize=12)
    ax4.grid(True, which='both', alpha=0.3)
    
    lines1, labels1 = ax4.get_legend_handles_labels()
    lines2, labels2 = ax4_r.get_legend_handles_labels()
    ax4.legend(lines1 + lines2, labels1 + labels2, loc='center left', fontsize=8.5)
    
    fig.suptitle(f'{title_prefix} — Complete Performance Breakdown (Angle, Velocity, Cross-Axis, Bode)', fontweight='bold', fontsize=13, y=0.98)
    
    out_art_path = os.path.join(art_dir, out_name)
    fig.savefig(out_art_path, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved {out_art_path}')
    
    ws_path = os.path.join('.', out_name)
    shutil.copyfile(out_art_path, ws_path)
    print(f'Copied to {ws_path}')
    return out_art_path

# Figure 1: Pre-Adaptive Champion (W04_S4_G02)
w04_gains = [83.19, 217.33, 273.21, 446.22, 508.55, 562.74]
w04_phases = [-9.82, -102.53, -120.02, -168.97, -224.86, -272.02]
p1_art = make_4panel_figure(
    title_prefix='Pre-Adaptive Champion (W04_S4_G02)',
    model_label='Stationary W04_S4_G02',
    pitch=w['pitch'],
    roll=w['roll'],
    yaw=w['yaw'],
    gains=w04_gains,
    phases=w04_phases,
    total_score=1638.71,
    color_main='#2563eb',
    out_name='pre_adaptive_champion_w04_s4_g02_4panel.png'
)

# Figure 2: Current Adaptive Model (Adaptive_W04_S4_G02)
adp_gains = [77.81, 217.52, 244.43, 413.97, 488.28, 536.65]
adp_phases = [-18.39, -102.49, -119.53, -166.33, -226.16, -273.73]
p2_art = make_4panel_figure(
    title_prefix='Current Adaptive Controller (Adaptive_W04_S4_G02)',
    model_label='Adaptive Model (t_adapt=1.8s)',
    pitch=a['pitch'],
    roll=a['roll'],
    yaw=a['yaw'],
    gains=adp_gains,
    phases=adp_phases,
    total_score=2313.67,
    color_main='#059669',
    out_name='current_adaptive_model_w04_s4_g02_4panel.png'
)
