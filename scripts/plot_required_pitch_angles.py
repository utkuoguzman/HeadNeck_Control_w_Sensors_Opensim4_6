import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

def generate_pitch_angle_plausibility_plots():
    csv_path = "extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv"
    df = pd.read_csv(csv_path)
    
    freqs = df['Frequency_Hz'].values
    exp_gain = df['Exp_Gain_deg_per_m'].values
    happee_gain = df['HappeeModel_Gain_deg_per_m'].values
    exp_phase = df['Exp_Phase_deg'].values
    happee_phase = df['HappeeModel_Phase_deg'].values
    
    # Sled perturbation kinematics:
    # Displacement amplitude x_amp = 0.0215 * (0.835 / f)^2 [m]
    # Sled acceleration a_amp = x_amp * (2 * pi * f)^2 = 0.0215 * (2 * pi * 0.835)^2 = 0.5918 m/s^2 (~0.060 g)
    x_amp_m = 0.0215 * (0.835 / freqs)**2
    x_amp_cm = x_amp_m * 100.0
    omega = 2.0 * np.pi * freqs
    v_amp_mps = x_amp_m * omega
    a_amp_mps2 = x_amp_m * (omega**2)
    a_amp_g = a_amp_mps2 / 9.81
    
    # Head pitch angle amplitude theta_amp = Gain * x_amp
    theta_amp_exp_deg = exp_gain * x_amp_m
    theta_amp_happee_deg = happee_gain * x_amp_m
    
    # Continuous curve for plotting smooth theoretical trend
    f_cont = np.geomspace(0.2, 10.0, 300)
    x_amp_cont_m = 0.0215 * (0.835 / f_cont)**2
    x_amp_cont_cm = x_amp_cont_m * 100.0
    
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.patch.set_facecolor('#ffffff')
    
    # -------------------------------------------------------------
    # Subplot 1: Sled Input Kinematics vs Frequency
    # -------------------------------------------------------------
    ax1 = axes[0, 0]
    ax1.plot(f_cont, x_amp_cont_cm, color='#1f77b4', lw=2.5, label=r'Sled Stroke $x_{\mathrm{sled}}(f)$ [cm]')
    ax1.scatter(freqs, x_amp_cm, color='#1f77b4', s=60, zorder=5)
    for f, x in zip(freqs, x_amp_cm):
        ax1.annotate(f"{x:.2f} cm", (f, x), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=9, fontweight='bold', color='#1f77b4')
                     
    ax1.set_xscale('log')
    ax1.set_yscale('log')
    ax1.set_title("A: Input Sled Perturbation Amplitude vs. Frequency", fontsize=12, fontweight='bold', pad=10)
    ax1.set_xlabel("Frequency [Hz]", fontsize=11, fontweight='semibold')
    ax1.set_ylabel("Sled Stroke Amplitude [cm]", fontsize=11, fontweight='semibold', color='#1f77b4')
    ax1.grid(True, which="both", ls="--", alpha=0.5)
    
    # Secondary axis for constant acceleration
    ax1_twin = ax1.twinx()
    ax1_twin.set_ylabel("Peak Acceleration [g]", fontsize=11, fontweight='semibold', color='#d62728')
    ax1_twin.plot(f_cont, np.full_like(f_cont, 0.0603), color='#d62728', ls='--', lw=1.8, label=r'Acc: $\approx 0.06\ \mathrm{g}\ (0.59\ \mathrm{m/s}^2)$')
    ax1_twin.set_ylim(0.0, 0.12)
    ax1_twin.tick_params(axis='y', labelcolor='#d62728')
    
    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax1_twin.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper right', frameon=True, facecolor='white', framealpha=0.9)
    
    # -------------------------------------------------------------
    # Subplot 2: Required Head Pitch Oscillation Amplitude (The Key Metric!)
    # -------------------------------------------------------------
    ax2 = axes[0, 1]
    ax2.plot(freqs, theta_amp_exp_deg, 'o-', color='#2ca02c', lw=2.5, markersize=8, label=r'Experimental Target $\theta_{\mathrm{amp}}^{\mathrm{exp}}$')
    ax2.plot(freqs, theta_amp_happee_deg, 's--', color='#ff7f0e', lw=2.0, markersize=7, label=r'Happee (2017) Model $\theta_{\mathrm{amp}}^{\mathrm{model}}$')
    
    for f, th_exp, th_hap in zip(freqs, theta_amp_exp_deg, theta_amp_happee_deg):
        ax2.annotate(f"{th_exp:.2f}°", (f, th_exp), textcoords="offset points", xytext=(0, 9),
                     ha='center', fontsize=9.5, fontweight='bold', color='#1b611b')
        ax2.annotate(f"({th_hap:.2f}°)", (f, th_hap), textcoords="offset points", xytext=(0, -14),
                     ha='center', fontsize=8.5, fontstyle='italic', color='#b85300')
                     
    ax2.set_xscale('log')
    ax2.set_yscale('log')
    ax2.set_title("B: Physical Head Pitch Angle Amplitude $\\theta_{\\mathrm{amp}}(f)$ [deg]", fontsize=12, fontweight='bold', pad=10)
    ax2.set_xlabel("Frequency [Hz]", fontsize=11, fontweight='semibold')
    ax2.set_ylabel("Head Pitch Oscillation Amplitude [deg]", fontsize=11, fontweight='semibold')
    ax2.set_ylim(0.05, 25.0)
    ax2.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f°'))
    ax2.grid(True, which="both", ls="--", alpha=0.5)
    ax2.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9)
    
    # Shaded zones of physical biomechanics
    ax2.axhspan(10.0, 20.0, color='yellow', alpha=0.15, label='Mild conscious nodding (10-20°)')
    ax2.axhspan(1.0, 10.0, color='green', alpha=0.12, label='Comfortable reflexive nodding (1-10°)')
    ax2.axhspan(0.05, 1.0, color='cyan', alpha=0.10, label='Micro-vibrations (<1°)')
    
    # -------------------------------------------------------------
    # Subplot 3: Angular Velocity Bode Gain [deg/m]
    # -------------------------------------------------------------
    ax3 = axes[1, 0]
    ax3.plot(freqs, exp_gain, 'o-', color='#2ca02c', lw=2.5, markersize=8, label='Experimental Target (Fig 5)')
    ax3.plot(freqs, happee_gain, 's--', color='#ff7f0e', lw=2.0, markersize=7, label='Happee (2017) Model')
    
    for f, g_exp in zip(freqs, exp_gain):
        ax3.annotate(f"{g_exp:.1f}", (f, g_exp), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=9, fontweight='bold', color='#1b611b')
                     
    ax3.set_xscale('log')
    ax3.set_yscale('linear')
    ax3.set_title("C: Angular Head Velocity Bode Gain [deg/m] (= [deg/s] / [m/s])", fontsize=12, fontweight='bold', pad=10)
    ax3.set_xlabel("Frequency [Hz]", fontsize=11, fontweight='semibold')
    ax3.set_ylabel("Gain [deg/m]", fontsize=11, fontweight='semibold')
    ax3.set_ylim(0, 750)
    ax3.grid(True, which="both", ls="--", alpha=0.5)
    ax3.legend(loc='lower right', frameon=True, facecolor='white', framealpha=0.9)
    
    # -------------------------------------------------------------
    # Subplot 4: Anatomical & Biomechanical Plausibility Reference
    # -------------------------------------------------------------
    ax4 = axes[1, 1]
    categories = [
        'Anatomical ROM\n(Flexion-Extension)',
        'Driving / Transit\nHead Jerk',
        '0.324 Hz Target\n(Exp: 12.3°)',
        '0.783 Hz Target\n(Exp: 5.8°)',
        '1.099 Hz Target\n(Exp: 4.0°)',
        '2.446 Hz Target\n(Exp: 1.1°)',
        '5.583 Hz Target\n(Exp: 0.24°)',
        '7.927 Hz Target\n(Exp: 0.14°)'
    ]
    angles = [130.0, 25.0, 12.36, 5.79, 3.97, 1.06, 0.24, 0.14]
    colors = ['#7f7f7f', '#bcbd22', '#d62728', '#e377c2', '#1f77b4', '#17becf', '#2ca02c', '#8c564b']
    
    bars = ax4.barh(categories[::-1], angles[::-1], color=colors[::-1], edgecolor='black', alpha=0.85, height=0.6)
    ax4.set_xscale('log')
    ax4.set_title("D: Biomechanical Plausibility Benchmark vs. Human Neck Limits", fontsize=12, fontweight='bold', pad=10)
    ax4.set_xlabel("Pitch Angle Amplitude [degrees] (Log Scale)", fontsize=11, fontweight='semibold')
    ax4.set_xlim(0.05, 200.0)
    ax4.grid(True, which="both", ls="--", alpha=0.5)
    
    for bar in bars:
        w = bar.get_width()
        ax4.text(w * 1.15, bar.get_y() + bar.get_height()/2.0, f"{w:.2f}°" if w < 100 else f"{w:.0f}°",
                 va='center', fontsize=9.5, fontweight='bold', color='#222222')
                 
    plt.tight_layout()
    
    out_dir = "extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel"
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "required_pitch_angle_plausibility.png")
    fig.savefig(out_path, dpi=300, bbox_inches='tight')
    print(f"[OK] Saved plot to {out_path}")
    
    # Also save to conversation artifacts directory
    artifact_dir = "C:/Users/User/.gemini/antigravity/brain/89fb8d56-9d12-4051-a256-02497ef8d6ae"
    artifact_path = os.path.join(artifact_dir, "required_pitch_angle_plausibility.png")
    fig.savefig(artifact_path, dpi=300, bbox_inches='tight')
    print(f"[OK] Saved plot to artifact directory: {artifact_path}")
    
    # Print tabular breakdown
    print("\n--- EXACT TABULAR BREAKDOWN ACROSS FREQUENCIES ---")
    print(f"{'Freq [Hz]':<10} | {'Sled Stroke [cm]':<16} | {'Exp Gain [deg/m]':<16} | {'Req Exp Angle [deg]':<20} | {'Happee Angle [deg]':<18} | {'Physiological Feel'}")
    print("-" * 110)
    descriptors = [
        "Gentle large rocking / nodding",
        "Natural compliant seated sway",
        "Moderate neck compliance",
        "Subtle stiffness response",
        "High-frequency micro-shudder",
        "Barely perceptible tremor"
    ]
    for f, x_cm, g_e, th_e, th_h, desc in zip(freqs, x_amp_cm, exp_gain, theta_amp_exp_deg, theta_amp_happee_deg, descriptors):
        print(f"{f:<10.3f} | {x_cm:<16.2f} | {g_e:<16.1f} | {th_e:<20.2f} | {th_h:<18.2f} | {desc}")

if __name__ == "__main__":
    generate_pitch_angle_plausibility_plots()

