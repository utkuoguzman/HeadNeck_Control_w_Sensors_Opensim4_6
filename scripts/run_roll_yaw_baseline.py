import os
import sys
import csv
import math
import json
import shutil
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
_UTIL = os.path.join(_ROOT, "util")
if _UTIL not in sys.path:
    sys.path.insert(0, _UTIL)

import opensim as osim
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0

ART_DIR = r"C:/Users/User/.gemini/antigravity/brain/89fb8d56-9d12-4051-a256-02497ef8d6ae"

# Champion Pitch Parameter Set (Run 4, 991.11 pts):
P15_RUN4 = {
    "K_ton": 5.2598427596916935,
    "p_ton": 16.227225,
    "K_sc": 4.101868521564445,
    "z_sc": 0.0055444407969709435,
    "p_sc": 53.284141,
    "K_phas": 15.420486774711076,
    "z_phas": 0.013306657912730264,
    "p_phas": 4.711937,
    "kp_prop": 0.44351132582628877,
    "z_ccr": 0.0,
    "p_ccr": 99.919586,
    "k_gamma_dyn": 1.0,
    "k_gamma_stat": 1.0,
    "Kp_pitch": 3.7901174594192497,
    "Kd_pitch": 0.49457962404720823,
    "Ki_pitch": 3.7214919179870036,
    "Kp_roll": 10.21,
    "Kd_roll": 1.29,
    "Ki_roll": 0.0,
    "Kp_yaw": 0.24,
    "Kd_yaw": 0.12,
    "Ki_yaw": 0.0,
}

def _read_csv_cols(path):
    cols = {}
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for fn in reader.fieldnames:
            cols[fn] = []
        for row in reader:
            for fn in reader.fieldnames:
                cols[fn].append(float(row[fn]))
    return {k: np.array(v) for k, v in cols.items()}

def load_aligned_data():
    df_lat = _read_csv_cols(os.path.join(_ROOT, "extracted_data/Happee_2023/LatEC_Fig5/aligned_LatEC_Roll_data.csv"))
    df_yaw = _read_csv_cols(os.path.join(_ROOT, "extracted_data/Happee_2023/YawMA_Fig9/aligned_YawMA_Yaw_data.csv"))
    return df_lat, df_yaw

def get_latec_amplitudes(freqs):
    """
    LatEC (Lateral T1 translation -> Head Roll [deg/m]):
    In Happee et al. (2023) Fig. 5, the 2-second time-domain panels show:
      - Lateral seat velocity Y_dot(t) peaking at +/- 0.02 m/s (2.0 cm/s)
      - Head roll angular velocity phi_dot(t) peaking at +/- 10 deg/s
    Using our constant-velocity multisine law A(f_k) = (A_0 / sqrt(K)) * (f_ref / f_k)^1.0
    calibrated so the 8-frequency sum matches Fig. 5's +/- 0.02 m/s seat velocity and +/- 10 deg/s roll rate:
      A(f_k) = (0.0055 / sqrt(8)) * (0.835 / f_k) = 0.0016237 / f_k [m]
    Peak harmonic velocity per component = 2 * pi * 0.0016237 = 0.01020 m/s (1.02 cm/s),
    giving total multisine peak seat velocity ~ 0.022 m/s and peak target roll velocity ~ 10.5 deg/s!
    """
    K = len(freqs)
    scale = 1.0 / math.sqrt(K)
    return np.array([0.0055 * (0.835 / f) * scale for f in freqs])

def get_yawma_amplitudes_deg(freqs):
    """
    YawMA (Axial T1 yaw rotation -> Head Global Yaw [deg/deg]):
    In Stensdotter et al. (2016) / Keshner & Peterson (1995) / Happee (2023) Fig. 9,
    rotational platform sum-of-sines scales as 1/f (constant angular velocity per harmonic)
    with total RMS trunk yaw ~ 1.5 - 2.0 deg (peak ~ +/- 3.5 deg) and peak yaw rate ~ +/- 15 deg/s:
      A_deg(f_k) = (3.0 / sqrt(9)) * (0.50 / f_k) = 0.50 / f_k [deg]
    At f_1 = 0.2016 Hz, A_1 = 2.48 deg; at f_9 = 4.0528 Hz, A_9 = 0.123 deg.
    Each harmonic has constant peak angular velocity = 2 * pi * 0.50 = 3.14 deg/s.
    """
    K = len(freqs)
    scale = 1.0 / math.sqrt(K)
    return np.array([3.0 * (0.50 / f) * scale for f in freqs])

def generate_plausibility_plots(df_lat, df_yaw):
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    
    # 1. LatEC Roll Plausibility Plot
    f_lat = df_lat["Frequency_Hz"]
    g_lat = df_lat["Exp_Roll_Gain_deg_per_m"]
    p_lat = df_lat["Exp_Roll_Phase_deg"]
    a_lat_m = get_latec_amplitudes(f_lat)
    a_lat_cm = a_lat_m * 100.0
    v_lat_cms = 2.0 * np.pi * f_lat * a_lat_cm
    req_roll_deg = g_lat * a_lat_m
    req_roll_vel_degs = 2.0 * np.pi * f_lat * req_roll_deg

    fig, axes = plt.subplots(2, 2, figsize=(14, 10), dpi=150)
    fig.patch.set_facecolor('#ffffff')

    f_cont = np.geomspace(0.10, 4.5, 300)
    a_cont_cm = (0.0055 * (0.835 / f_cont) / math.sqrt(len(f_lat))) * 100.0
    v_cont_cms = 2.0 * np.pi * f_cont * a_cont_cm

    ax1 = axes[0, 0]
    ax1.plot(f_cont, a_cont_cm, color='#1f77b4', lw=2.5, label=r'Lateral T1 Stroke $A_{\mathrm{LatEC}}(f) \propto 1/f$ [cm]')
    ax1.scatter(f_lat, a_lat_cm, color='#1f77b4', s=60, zorder=5)
    for f, x in zip(f_lat, a_lat_cm):
        ax1.annotate(f"{x:.2f} cm", (f, x), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=8.5, fontweight='bold', color='#1f77b4')
    ax1.set_xscale('log')
    ax1.set_yscale('log')
    ax1.set_title("A: Input Lateral T1 Stroke $A(f)$ vs. Frequency (LatEC)", fontsize=11.5, fontweight='bold')
    ax1.set_xlabel("Frequency [Hz]", fontweight='semibold')
    ax1.set_ylabel("Lateral T1 Amplitude [cm]", fontweight='semibold', color='#1f77b4')
    ax1.grid(True, which="both", ls="--", alpha=0.5)
    ax1_r = ax1.twinx()
    ax1_r.plot(f_cont, v_cont_cms, color='#d62728', ls='--', lw=1.8, label=f'Peak Harmonic Vel = {v_lat_cms[0]:.2f} cm/s')
    ax1_r.set_ylabel("Peak Harmonic Velocity [cm/s]", fontweight='semibold', color='#d62728')
    ax1_r.set_ylim(0, 5.0)
    l1, lb1 = ax1.get_legend_handles_labels()
    l2, lb2 = ax1_r.get_legend_handles_labels()
    ax1.legend(l1 + l2, lb1 + lb2, loc='upper right', fontsize=8.5, frameon=True)

    ax2 = axes[0, 1]
    ax2.plot(f_lat, req_roll_deg, 'o-', color='#2ca02c', lw=2.5, markersize=7, label=r'Expected Head Roll Amplitude $\phi_{\mathrm{amp}}^{\mathrm{exp}}(f)$ [deg]')
    for f, th in zip(f_lat, req_roll_deg):
        ax2.annotate(f"{th:.2f}°", (f, th), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=8.5, fontweight='bold', color='#1b611b')
    ax2.set_xscale('log')
    ax2.set_yscale('log')
    ax2.set_ylim(0.05, 10.0)
    ax2.set_title("B: Expected Physical Head Roll Oscillation Amplitude [deg]", fontsize=11.5, fontweight='bold')
    ax2.set_xlabel("Frequency [Hz]", fontweight='semibold')
    ax2.set_ylabel("Head Roll Amplitude [deg]", fontweight='semibold')
    ax2.grid(True, which="both", ls="--", alpha=0.5)
    ax2.legend(loc='upper right', fontsize=8.5, frameon=True)

    ax3 = axes[1, 0]
    ax3.plot(f_lat, g_lat, 'o-', color='#2ca02c', lw=2.5, markersize=7, label='Exp Target Roll Gain (Happee 2023 Fig. 5) [deg/m]')
    for f, g in zip(f_lat, g_lat):
        ax3.annotate(f"{g:.1f}", (f, g), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=8.5, fontweight='bold', color='#1b611b')
    ax3.set_xscale('log')
    ax3.set_ylim(0, 620)
    ax3.set_title("C: Aligned LatEC Head Roll Bode Gain [deg/m]", fontsize=11.5, fontweight='bold')
    ax3.set_xlabel("Frequency [Hz]", fontweight='semibold')
    ax3.set_ylabel("Roll Gain [deg/m]", fontweight='semibold')
    ax3.grid(True, which="both", ls="--", alpha=0.5)
    ax3.legend(loc='upper left', fontsize=8.5, frameon=True)

    ax4 = axes[1, 1]
    ax4.plot(f_lat, p_lat, 'D-', color='#92400e', lw=2.5, markersize=7, label='Exp Target Roll Phase (Happee 2023 Fig. 5) [deg]')
    for f, ph in zip(f_lat, p_lat):
        ax4.annotate(f"{ph:.1f}°", (f, ph), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=8.5, fontweight='bold', color='#92400e')
    ax4.set_xscale('log')
    ax4.set_ylim(-400, -180)
    ax4.set_title("D: Aligned LatEC Head Roll Bode Phase [deg]", fontsize=11.5, fontweight='bold')
    ax4.set_xlabel("Frequency [Hz]", fontweight='semibold')
    ax4.set_ylabel("Roll Phase [deg]", fontweight='semibold')
    ax4.grid(True, which="both", ls="--", alpha=0.5)
    ax4.legend(loc='upper right', fontsize=8.5, frameon=True)

    fig.suptitle("LatEC (Lateral Seat Translation, Eyes Closed — Happee 2023 Fig. 5): Input $A(f)$ & Aligned Roll Targets", fontsize=13, fontweight='bold', y=0.99)
    plt.tight_layout()
    out_lat_plaus = os.path.join(ART_DIR, "latec_roll_plausibility_and_targets.png")
    fig.savefig(out_lat_plaus, bbox_inches='tight')
    shutil.copyfile(out_lat_plaus, os.path.join(_ROOT, "extracted_data/Happee_2023/LatEC_Fig5/latec_roll_plausibility_and_targets.png"))
    plt.close(fig)

    # 2. YawMA Yaw Plausibility Plot
    f_yaw = df_yaw["Frequency_Hz"]
    g_yaw = df_yaw["Exp_Yaw_Gain_deg_per_deg"]
    p_yaw = df_yaw["Exp_Yaw_Phase_deg"]
    a_yaw_deg = get_yawma_amplitudes_deg(f_yaw)
    v_yaw_degs = 2.0 * np.pi * f_yaw * a_yaw_deg
    req_yaw_deg = g_yaw * a_yaw_deg

    fig, axes = plt.subplots(2, 2, figsize=(14, 10), dpi=150)
    fig.patch.set_facecolor('#ffffff')

    f_cont_y = np.geomspace(0.18, 4.5, 300)
    a_cont_y = (3.0 * (0.50 / f_cont_y) / math.sqrt(len(f_yaw)))
    v_cont_y = 2.0 * np.pi * f_cont_y * a_cont_y

    ax1 = axes[0, 0]
    ax1.plot(f_cont_y, a_cont_y, color='#1f77b4', lw=2.5, label=r'Trunk Yaw Rotation $A_{\mathrm{YawMA}}(f) \propto 1/f$ [deg]')
    ax1.scatter(f_yaw, a_yaw_deg, color='#1f77b4', s=60, zorder=5)
    for f, x in zip(f_yaw, a_yaw_deg):
        ax1.annotate(f"{x:.2f}°", (f, x), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=8.5, fontweight='bold', color='#1f77b4')
    ax1.set_xscale('log')
    ax1.set_yscale('log')
    ax1.set_title("A: Input Trunk Yaw Rotation Amplitude $A(f)$ vs. Frequency (YawMA)", fontsize=11.5, fontweight='bold')
    ax1.set_xlabel("Frequency [Hz]", fontweight='semibold')
    ax1.set_ylabel("Trunk Yaw Amplitude [deg]", fontweight='semibold', color='#1f77b4')
    ax1.grid(True, which="both", ls="--", alpha=0.5)
    ax1_r = ax1.twinx()
    ax1_r.plot(f_cont_y, v_cont_y, color='#d62728', ls='--', lw=1.8, label=f'Peak Harmonic Angular Vel = {v_yaw_degs[0]:.2f} °/s')
    ax1_r.set_ylabel("Peak Harmonic Angular Velocity [°/s]", fontweight='semibold', color='#d62728')
    ax1_r.set_ylim(0, 6.0)
    l1, lb1 = ax1.get_legend_handles_labels()
    l2, lb2 = ax1_r.get_legend_handles_labels()
    ax1.legend(l1 + l2, lb1 + lb2, loc='upper right', fontsize=8.5, frameon=True)

    ax2 = axes[0, 1]
    ax2.plot(f_yaw, req_yaw_deg, 'o-', color='#2ca02c', lw=2.5, markersize=7, label=r'Expected Global Head Yaw Amplitude $\psi_{\mathrm{amp}}^{\mathrm{exp}}(f)$ [deg]')
    for f, th in zip(f_yaw, req_yaw_deg):
        ax2.annotate(f"{th:.2f}°", (f, th), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=8.5, fontweight='bold', color='#1b611b')
    ax2.set_xscale('log')
    ax2.set_yscale('log')
    ax2.set_ylim(0.05, 5.0)
    ax2.set_title("B: Expected Global Head Yaw Oscillation Amplitude [deg]", fontsize=11.5, fontweight='bold')
    ax2.set_xlabel("Frequency [Hz]", fontweight='semibold')
    ax2.set_ylabel("Global Head Yaw Amplitude [deg]", fontweight='semibold')
    ax2.grid(True, which="both", ls="--", alpha=0.5)
    ax2.legend(loc='upper right', fontsize=8.5, frameon=True)

    ax3 = axes[1, 0]
    ax3.plot(f_yaw, g_yaw, 'o-', color='#2ca02c', lw=2.5, markersize=7, label='Exp Target Yaw Gain (Happee 2023 Fig. 9) [deg/deg]')
    for f, g in zip(f_yaw, g_yaw):
        ax3.annotate(f"{g:.2f}", (f, g), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=8.5, fontweight='bold', color='#1b611b')
    ax3.set_xscale('log')
    ax3.set_ylim(0, 2.0)
    ax3.set_title("C: Aligned YawMA Global Head Yaw Bode Gain [deg/deg]", fontsize=11.5, fontweight='bold')
    ax3.set_xlabel("Frequency [Hz]", fontweight='semibold')
    ax3.set_ylabel("Yaw Gain [deg/deg]", fontweight='semibold')
    ax3.grid(True, which="both", ls="--", alpha=0.5)
    ax3.legend(loc='upper left', fontsize=8.5, frameon=True)

    ax4 = axes[1, 1]
    ax4.plot(f_yaw, p_yaw, 'D-', color='#92400e', lw=2.5, markersize=7, label='Exp Target Yaw Phase (Happee 2023 Fig. 9) [deg]')
    for f, ph in zip(f_yaw, p_yaw):
        ax4.annotate(f"{ph:+.1f}°", (f, ph), textcoords="offset points", xytext=(0, 8),
                     ha='center', fontsize=8.5, fontweight='bold', color='#92400e')
    ax4.set_xscale('log')
    ax4.set_ylim(-130, 40)
    ax4.set_title("D: Aligned YawMA Global Head Yaw Bode Phase [deg]", fontsize=11.5, fontweight='bold')
    ax4.set_xlabel("Frequency [Hz]", fontweight='semibold')
    ax4.set_ylabel("Yaw Phase [deg]", fontweight='semibold')
    ax4.grid(True, which="both", ls="--", alpha=0.5)
    ax4.legend(loc='upper right', fontsize=8.5, frameon=True)

    fig.suptitle("YawMA (Axial Seat Yaw Rotation, Mental Arithmetic — Happee 2023 Fig. 9): Input $A(f)$ & Aligned Yaw Targets", fontsize=13, fontweight='bold', y=0.99)
    plt.tight_layout()
    out_yaw_plaus = os.path.join(ART_DIR, "yawma_yaw_plausibility_and_targets.png")
    fig.savefig(out_yaw_plaus, bbox_inches='tight')
    shutil.copyfile(out_yaw_plaus, os.path.join(_ROOT, "extracted_data/Happee_2023/YawMA_Fig9/yawma_yaw_plausibility_and_targets.png"))
    plt.close(fig)
    print("[OK] Saved LatEC and YawMA plausibility plots!", flush=True)


def build_c2_multisine_spline(freqs, amps, phases, delay, ramp_duration, target_time, spline_dt=0.0025):
    """
    Constructs a C^2 acceleration-windowed SimmSpline (Solution A) so that
    for t >= delay + ramp_duration, the signal has zero velocity drift and
    exact harmonic amplitudes amps * sin(2*pi*f*(t - delay) + phase).
    """
    ms_spline = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        ms_spline.addPoint(float(t), 0.0)

    omegas = 2.0 * np.pi * freqs
    sub = 10
    dt_fine = spline_dt / sub
    N_fine = int(round(ramp_duration / dt_fine))
    t_fine = np.linspace(0.0, ramp_duration, N_fine + 1)
    tau_fine = t_fine / ramp_duration
    w_fine = 10.0 * (tau_fine ** 3) - 15.0 * (tau_fine ** 4) + 6.0 * (tau_fine ** 5)
    phi_v = (tau_fine ** 2) * ((1.0 - tau_fine) ** 2)
    phi_v /= np.sum(0.5 * (phi_v[:-1] + phi_v[1:]) * dt_fine)

    psi_fine = np.outer(t_fine, omegas) + phases
    y_fine = np.sin(psi_fine) @ amps
    dy_fine = np.cos(psi_fine) @ (amps * omegas)
    ddy_fine = -np.sin(psi_fine) @ (amps * (omegas ** 2))

    ddx_raw = w_fine * ddy_fine
    v_raw_end = float(np.sum(0.5 * (ddx_raw[:-1] + ddx_raw[1:]) * dt_fine))
    dv_match = float(dy_fine[-1]) - v_raw_end
    ddx_fine = ddx_raw + dv_match * phi_v

    dx_fine = np.zeros_like(t_fine)
    dx_fine[1:] = np.cumsum(0.5 * (ddx_fine[:-1] + ddx_fine[1:]) * dt_fine)
    x_fine = np.zeros_like(t_fine)
    x_fine[1:] = np.cumsum(0.5 * (dx_fine[:-1] + dx_fine[1:]) * dt_fine)
    x_offset = float(x_fine[-1] - y_fine[-1])

    N_ramp_knots = int(round(ramp_duration / spline_dt))
    for i_k in range(N_ramp_knots):
        t = delay + i_k * spline_dt
        ms_spline.addPoint(float(t), float(x_fine[i_k * sub]))
    for t in np.arange(delay + ramp_duration, target_time + 0.02, spline_dt):
        dt_rel = t - delay
        y = float(np.sum(amps * np.sin(omegas * dt_rel + phases)))
        ms_spline.addPoint(float(t), float(y + x_offset))

    return ms_spline, x_offset


def run_axis_perturbation_sim(mode, df_target, params=P15_RUN4, eval_cycles=2.0):
    """
    mode: 'LatEC' (prescribes spine_tz in meters, measures global head roll = roll1 + roll2 in deg)
       or 'YawMA' (prescribes gndyaw in radians, measures global head yaw = gndyaw_deg + yaw1 + yaw2 in deg)
    """
    ensure_libraries_loaded()
    model = osim.Model(os.path.join(_ROOT, "osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim"))

    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    if controller is None:
        raise RuntimeError("HeadNeckNeuralController not found!")

    osim.PropertyHelper.setValueDouble(params["K_ton"], controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(params["p_ton"], controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(params["K_sc"], controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(params["z_sc"], controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(params["p_sc"], controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(params["K_phas"], controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(params["z_phas"], controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(params["p_phas"], controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(-1.0, controller.updPropertyByName("p_phas2"))
    osim.PropertyHelper.setValueDouble(params["kp_prop"], controller.updPropertyByName("k_p"))

    osim.PropertyHelper.setValueDouble(params["Kp_pitch"], controller.updPropertyByName("Kp_task_pitch"))
    osim.PropertyHelper.setValueDouble(params["Kd_pitch"], controller.updPropertyByName("Kd_task_pitch"))
    osim.PropertyHelper.setValueDouble(params["Kp_roll"], controller.updPropertyByName("Kp_task_roll"))
    osim.PropertyHelper.setValueDouble(params["Kd_roll"], controller.updPropertyByName("Kd_task_roll"))
    osim.PropertyHelper.setValueDouble(params["Kp_yaw"], controller.updPropertyByName("Kp_task_yaw"))
    osim.PropertyHelper.setValueDouble(params["Kd_yaw"], controller.updPropertyByName("Kd_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    osim.PropertyHelper.setValueDouble(params["Ki_pitch"], controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(params["Ki_pitch"], controller.updPropertyByName("Ki_task_pitch"))
    osim.PropertyHelper.setValueDouble(params["Ki_roll"], controller.updPropertyByName("Ki_task_roll"))
    osim.PropertyHelper.setValueDouble(params["Ki_yaw"], controller.updPropertyByName("Ki_task_yaw"))

    osim.PropertyHelper.setValueDouble(params["kp_prop"], controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(params["z_ccr"], controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(params["p_ccr"], controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(params["k_gamma_dyn"], controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(params["k_gamma_stat"], controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("use_smith_predictor"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_smith_proprio"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_afferent_proprio"))

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    eye_list = ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]
    for eye_c in eye_list:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    freqs = df_target["Frequency_Hz"]
    K = len(freqs)
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)])

    delay = 1.5
    ramp_duration = 1.5
    eval_duration = eval_cycles / freqs[0]
    target_time = delay + ramp_duration + eval_duration

    if mode == "LatEC":
        # Input is lateral T1 translation spine_tz in meters:
        amps_input_unit = get_latec_amplitudes(freqs) # meters
        amps_sim_unit = amps_input_unit.copy()        # spine_tz is in meters
        prescribed_coord_name = "spine_tz"
        tgt_gains = df_target["Exp_Roll_Gain_deg_per_m"]
        tgt_phases = df_target["Exp_Roll_Phase_deg"]
    elif mode == "YawMA":
        # Input is axial T1 rotation gndyaw in degrees (OpenSim coordinate is in radians!):
        amps_input_unit = get_yawma_amplitudes_deg(freqs) # degrees (for Bode gain [deg/deg])
        amps_sim_unit = np.radians(amps_input_unit)       # radians (for OpenSim gndyaw)
        prescribed_coord_name = "gndyaw"
        tgt_gains = df_target["Exp_Yaw_Gain_deg_per_deg"]
        tgt_phases = df_target["Exp_Yaw_Phase_deg"]
    else:
        raise ValueError(f"Unknown mode: {mode}")

    ms_spline, x_offset = build_c2_multisine_spline(freqs, amps_sim_unit, phases, delay, ramp_duration, target_time)
    coord_pert = model.updCoordinateSet().get(prescribed_coord_name)
    coord_pert.set_locked(False)
    coord_pert.setDefaultIsPrescribed(True)
    coord_pert.setPrescribedFunction(ms_spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_pert.setSpeedValue(state, 0.0)

    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(name, 0.030)
        m.setActivation(state, a0)
        k_gdyn_eff = min(max(params["k_gamma_dyn"], 0.0), 1.0)
        k_gstat_eff = min(max(params["k_gamma_stat"], 0.0), 1.0)
        gd0 = 100.0 * k_gdyn_eff * (0.25 + 0.75 * a0)
        gs0 = 100.0 * k_gstat_eff * a0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{name}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{name}", fstat0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setWriteToStorage(False)
    manager.setPerformAnalyses(False)
    manager.setIntegratorMethod(5)  # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.001)

    state.setTime(0.0)
    manager.initialize(state)

    step = 0.005  # 200 Hz
    current_time = 0.0

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")
    coord_y2 = model.getCoordinateSet().get("yaw2")
    coord_gndyaw = model.getCoordinateSet().get("gndyaw")
    coord_tz = model.getCoordinateSet().get("spine_tz")

    t_list, pitch_list, roll_list, yaw_rel_list, yaw_glob_list, pert_list = [], [], [], [], [], []
    aborted = False

    print(f"\n[SIM] Starting {mode} simulation (target_time = {target_time:.2f} s, K = {K} freqs)...", flush=True)
    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        p_deg = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        r_deg = math.degrees(coord_r1.getValue(state) + coord_r2.getValue(state))
        y_rel_deg = math.degrees(coord_y1.getValue(state) + coord_y2.getValue(state))
        gnd_y_deg = math.degrees(coord_gndyaw.getValue(state))
        y_glob_deg = gnd_y_deg + y_rel_deg

        if mode == "LatEC":
            pert_val = coord_tz.getValue(state) # meters
        else:
            pert_val = gnd_y_deg                # degrees

        t_list.append(current_time)
        pitch_list.append(p_deg)
        roll_list.append(r_deg)
        yaw_rel_list.append(y_rel_deg)
        yaw_glob_list.append(y_glob_deg)
        pert_list.append(pert_val)

        if abs(p_deg) > 45.0 or abs(r_deg) > 35.0 or abs(y_rel_deg) > 48.0:
            print(f"[WARN] {mode} exceeded range at t={current_time:.2f}s: P={p_deg:.2f}, R={r_deg:.2f}, Y_rel={y_rel_deg:.2f}", flush=True)
            aborted = True
            break

    t_arr = np.array(t_list)
    pitch_arr = np.array(pitch_list)
    roll_arr = np.array(roll_list)
    yaw_rel_arr = np.array(yaw_rel_list)
    yaw_glob_arr = np.array(yaw_glob_list)
    pert_arr = np.array(pert_list)

    # Demodulate primary output axis over steady-state window t >= delay + ramp_duration
    m_post = t_arr >= (delay + ramp_duration)
    t_fit = t_arr[m_post]
    t_rel = t_fit - delay

    # Note on sign convention for LatEC:
    # In Happee (2023) ISO frame (+X forward, +Y left, +Z up), lateral translation is +Y (leftward),
    # and +Roll is right-ear-down (clockwise looking forward).
    # In OpenSim (+X forward, +Y up, +Z right), spine_tz is +Z (rightward = -Y_ISO),
    # and roll1+roll2 is around +X (counter-clockwise looking from front / right-ear-up or down).
    # Let's demodulate both raw and inverted-sign to check phase alignment with Happee's [-304, -366] deg!
    y_fit_deg = roll_arr[m_post] if mode == "LatEC" else yaw_glob_arr[m_post]

    cols = [np.ones_like(t_fit), t_fit - (delay + ramp_duration)]
    for k in range(K):
        omega_k = 2.0 * math.pi * freqs[k]
        psi_k = omega_k * t_rel + phases[k]
        cols.append(np.sin(psi_k))
        cols.append(np.cos(psi_k))

    A_mat = np.column_stack(cols)
    coeffs, _, _, _ = np.linalg.lstsq(A_mat, y_fit_deg, rcond=None)

    freq_results = {}
    for k in range(K):
        f_k = float(freqs[k])
        a_sin = float(coeffs[2 + 2 * k])
        b_cos = float(coeffs[2 + 2 * k + 1])
        amp_out_deg = math.hypot(a_sin, b_cos)
        sim_gain = amp_out_deg / float(amps_input_unit[k])

        phase_rad = math.atan2(b_cos, a_sin)
        phase_deg_raw = math.degrees(phase_rad)
        # Also compute phase if input/output sign convention has a 180 deg flip:
        phase_deg_inv = phase_deg_raw + 180.0

        tp = float(tgt_phases[k])
        err_raw = ((phase_deg_raw - tp + 180.0) % 360.0) - 180.0
        err_inv = ((phase_deg_inv - tp + 180.0) % 360.0) - 180.0

        freq_results[f"{f_k:.4f}"] = {
            "freq_hz": f_k,
            "input_amp": float(amps_input_unit[k]),
            "output_amp_deg": amp_out_deg,
            "sim_gain": sim_gain,
            "target_gain": float(tgt_gains[k]),
            "gain_db_diff": 20.0 * math.log10(max(sim_gain, 1e-6) / float(tgt_gains[k])),
            "sim_phase_raw": tp + err_raw,
            "sim_phase_inv": tp + err_inv,
            "target_phase": tp,
            "phase_err_raw": err_raw,
            "phase_err_inv": err_inv,
        }

    return {
        "mode": mode,
        "aborted": aborted,
        "t": t_arr,
        "pitch": pitch_arr,
        "roll": roll_arr,
        "yaw_rel": yaw_rel_arr,
        "yaw_glob": yaw_glob_arr,
        "pert": pert_arr,
        "freqs": freqs,
        "amps_input_unit": amps_input_unit,
        "phases": phases,
        "tgt_gains": tgt_gains,
        "tgt_phases": tgt_phases,
        "freq_results": freq_results,
    }


def render_baseline_4panel(sim_data, out_filename):
    mode = sim_data["mode"]
    t = sim_data["t"]
    pitch = sim_data["pitch"]
    roll = sim_data["roll"]
    yaw_rel = sim_data["yaw_rel"]
    yaw_glob = sim_data["yaw_glob"]
    freqs = sim_data["freqs"]
    amps = sim_data["amps_input_unit"]
    phases = sim_data["phases"]
    tgt_gains = sim_data["tgt_gains"]
    tgt_phases = sim_data["tgt_phases"]
    freq_results = sim_data["freq_results"]

    # Determine whether raw or ISO-inverted sign matches Happee's coordinate convention better:
    mean_abs_err_raw = np.mean([abs(freq_results[f"{f:.4f}"]["phase_err_raw"]) for f in freqs])
    mean_abs_err_inv = np.mean([abs(freq_results[f"{f:.4f}"]["phase_err_inv"]) for f in freqs])
    use_iso_sign_flip = (mode == "LatEC" and mean_abs_err_inv < mean_abs_err_raw)
    sign_mult = -1.0 if use_iso_sign_flip else 1.0

    # Construct analytic Happee reference trace in time domain
    delay, ramp = 1.5, 1.5
    ref_trace = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti < delay:
            continue
        t_rel = ti - delay
        w = 1.0
        if t_rel < ramp:
            tau = t_rel / ramp
            w = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
        y_deg = 0.0
        for k in range(len(freqs)):
            psi = 2.0 * np.pi * freqs[k] * t_rel + phases[k] + np.radians(tgt_phases[k])
            y_deg += amps[k] * tgt_gains[k] * np.sin(psi)
        ref_trace[i] = w * y_deg

    main_angle = sign_mult * roll if mode == "LatEC" else yaw_glob
    main_vel = np.gradient(main_angle, t)
    ref_vel = np.gradient(ref_trace, t)

    mask_ss = t >= 3.0
    mean_ang_ss = float(np.mean(main_angle[mask_ss]))
    rms_vel_ss = float(np.sqrt(np.mean(main_vel[mask_ss]**2)))
    ref_rms_v = float(np.sqrt(np.mean(ref_vel[mask_ss]**2)))
    corr_vel_ss = float(np.corrcoef(main_vel[mask_ss], ref_vel[mask_ss])[0, 1]) if np.std(main_vel[mask_ss]) > 1e-6 else 0.0

    sim_gains = [freq_results[f"{f:.4f}"]["sim_gain"] for f in freqs]
    sim_phases = [
        freq_results[f"{f:.4f}"]["sim_phase_inv" if use_iso_sign_flip else "sim_phase_raw"]
        for f in freqs
    ]

    color_main = "#2563eb" if mode == "LatEC" else "#7c3aed"
    axis_name = "Head Roll" if mode == "LatEC" else "Global Head Yaw"
    gain_unit = "deg/m" if mode == "LatEC" else "deg/deg"

    fig = plt.figure(figsize=(15, 14), dpi=150)
    gs = fig.add_gridspec(4, 1, height_ratios=[1.1, 1.1, 0.85, 1.15], hspace=0.32)

    # Subplot 1: Angle vs Time
    pert_vel = np.gradient(sim_data["pert"], t)
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, ref_trace, 'k--', lw=1.6, label=f'Target Happee (2023) Reference {axis_name} [Max = {np.max(np.abs(ref_trace[mask_ss])):.2f}°]')
    ax1.plot(t, main_angle, color=color_main, lw=2.0, label=f'Zero-Shot Pitch-Trained Model {axis_name} [Mean = {mean_ang_ss:+.2f}°, Max = {np.max(np.abs(main_angle[mask_ss])):.2f}°]')
    if mode == "YawMA":
        ax1.plot(t, sim_data["pert"], color='#64748b', ls=':', lw=1.2, alpha=0.8, label=f'Applied Trunk Yaw Input gndyaw(t) [Max = {np.max(np.abs(sim_data["pert"][mask_ss])):.2f}°]')
    ax1.axvline(1.5, color='#64748b', ls=':', lw=1.3, label='Perturbation Ramp Start (t = 1.5s)')
    ax1.axvline(3.0, color='#dc2626', ls='--', lw=1.4, label='Steady-State Window (t >= 3.0s)')
    ax1.set_xlim(0, t[-1])
    ax1.set_ylabel(f'{axis_name} [deg]', fontweight='bold', fontsize=11)
    ax1.set_title(f'1. {axis_name} Angle vs. Time — Zero-Shot Baseline ({mode}, Duration: {t[-1]:.2f}s)', fontweight='bold', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=8.8)

    # Subplot 2: Angular Velocity vs Time
    ax2 = fig.add_subplot(gs[1])
    if mode == "LatEC":
        ax2.axhspan(-10, 10, color='#f1f5f9', alpha=0.9, label=r'Happee Fig. 5 Roll Rate Envelope ($\pm 10^\circ$/s; Seat $\dot{Y}$ Peak = ' + f'{np.max(np.abs(pert_vel[mask_ss])):.3f} m/s vs 0.020 m/s in Fig. 5)')
    else:
        ax2.axhspan(-15, 15, color='#f1f5f9', alpha=0.9, label=r'Target Yaw Rate Envelope ($\pm 15^\circ$/s; Trunk $\dot{\psi}_{\mathrm{T1}}$ Peak = ' + f'{np.max(np.abs(pert_vel[mask_ss])):.1f}°/s)')
    ax2.axhline(0, color='#94a3b8', lw=0.8)
    ax2.plot(t, ref_vel, 'k--', lw=1.5, alpha=0.85, label=f'Target Happee Reference Velocity [RMS = {ref_rms_v:.2f}°/s, Peak = {np.max(np.abs(ref_vel[mask_ss])):.1f}°/s]')
    ax2.plot(t, main_vel, color=color_main, lw=1.8, label=f'Zero-Shot Model Velocity [RMS = {rms_vel_ss:.2f}°/s, Peak = {np.max(np.abs(main_vel[mask_ss])):.1f}°/s, corr r = {corr_vel_ss:+.3f}]')
    ax2.axvline(1.5, color='#64748b', ls=':', lw=1.3)
    ax2.axvline(3.0, color='#dc2626', ls='--', lw=1.4)
    ax2.set_xlim(0, t[-1])
    ax2.set_ylabel(f'{axis_name} Vel [°/s]', fontweight='bold', fontsize=11)
    ax2.set_title(f'2. {axis_name} Angular Velocity vs. Time ({mode})', fontweight='bold', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=8.5)

    # Subplot 3: Cross-Axis Stability
    ax3 = fig.add_subplot(gs[2])
    ax3.axhline(0, color='#94a3b8', lw=0.8)
    if mode == "LatEC":
        ax3.plot(t, pitch - pitch[0], color='#059669', lw=1.4, label=f'Pitch Deviation (p1+p2 - p0) [Max = {np.max(np.abs(pitch - pitch[0])):.2f}°]')
        ax3.plot(t, yaw_rel, color='#d97706', lw=1.4, label=f'Coupled Neck Yaw (y1+y2) [Max = {np.max(np.abs(yaw_rel)):.2f}°]')
    else:
        ax3.plot(t, pitch - pitch[0], color='#059669', lw=1.4, label=f'Pitch Deviation (p1+p2 - p0) [Max = {np.max(np.abs(pitch - pitch[0])):.2f}°]')
        ax3.plot(t, roll, color='#dc2626', lw=1.4, label=f'Coupled Head Roll (r1+r2) [Max = {np.max(np.abs(roll)):.2f}°]')
        ax3.plot(t, yaw_rel, color='#64748b', ls='--', lw=1.2, label=f'Relative Neck-on-Trunk Yaw (y1+y2) [Max = {np.max(np.abs(yaw_rel)):.2f}°]')
    ax3.axvline(1.5, color='#64748b', ls=':', lw=1.3)
    ax3.axvline(3.0, color='#dc2626', ls='--', lw=1.4)
    ax3.set_xlim(0, t[-1])
    ax3.set_ylabel('Cross-Axes [deg]', fontweight='bold', fontsize=11)
    ax3.set_title(f'3. 6-DOF Cross-Axis Coupling & Stability During {mode}', fontweight='bold', fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right', fontsize=8.8)

    # Subplot 4: Bode Plot
    ax4 = fig.add_subplot(gs[3])
    ax4_r = ax4.twinx()

    ax4.plot(freqs, tgt_gains, 'ko--', lw=1.8, label=f'Target Exp Gain (Happee 2023) [{gain_unit}]')
    ax4.plot(freqs, sim_gains, 's-', color=color_main, lw=2.2, label=f'Zero-Shot Model Gain [{gain_unit}]')
    ax4_r.plot(freqs, tgt_phases, '^--', color='#92400e', lw=1.8, label='Target Exp Phase (Happee 2023) [deg]')
    ax4_r.plot(freqs, sim_phases, 'D-', color='#b91c1c', lw=2.0, label='Zero-Shot Model Phase [deg]')

    for f, tg, mg, tp, mp in zip(freqs, tgt_gains, sim_gains, tgt_phases, sim_phases):
        db = 20.0 * np.log10(max(mg, 1e-6) / tg)
        pe = mp - tp
        ax4.annotate(f'{db:+.1f} dB', (f, mg), textcoords='offset points', xytext=(0, 9),
                     ha='center', fontsize=8.2, fontweight='bold', color=color_main)
        ax4_r.annotate(f'{pe:+.1f}°', (f, mp), textcoords='offset points', xytext=(0, -14),
                       ha='center', fontsize=8.2, fontweight='bold', color='#b91c1c')

    ax4.set_xscale('log')
    ax4.xaxis.set_minor_locator(ticker.NullLocator())
    ax4.xaxis.set_minor_formatter(ticker.NullFormatter())
    ax4.minorticks_off()
    ax4.set_xticks(freqs)
    ax4.set_xticklabels([f'{f:.2f} Hz' for f in freqs], fontweight='bold', fontsize=9.0)

    if mode == "LatEC":
        ax4.set_ylim(0, max(650.0, float(np.max(sim_gains)) * 1.18))
        ax4_r.set_ylim(-420, -150)
    else:
        ax4.set_ylim(0, max(2.2, float(np.max(sim_gains)) * 1.18))
        ax4_r.set_ylim(-180, 60)

    ax4.set_ylabel(f'Bode Gain [{gain_unit}]', color=color_main, fontweight='bold', fontsize=11)
    ax4_r.set_ylabel('Bode Phase [deg]', color='#b91c1c', fontweight='bold', fontsize=11)
    ax4.set_xlabel('Multisine Excitation Frequency [Hz] (Log Scale)', fontweight='bold', fontsize=11)
    ax4.set_title(f'4. Zero-Shot {len(freqs)}-Frequency Bode Response ({mode})', fontweight='bold', fontsize=12)
    ax4.grid(True, alpha=0.3)

    l1, lb1 = ax4.get_legend_handles_labels()
    l2, lb2 = ax4_r.get_legend_handles_labels()
    ax4.legend(l1 + l2, lb1 + lb2, loc='upper left', fontsize=8.5)

    fig.suptitle(f'Zero-Shot {mode} Baseline Evaluation — Pitch-Trained Champion Model (Run 4)', fontweight='bold', fontsize=13, y=0.98)
    out_art = os.path.join(ART_DIR, out_filename)
    fig.savefig(out_art, bbox_inches='tight')
    shutil.copyfile(out_art, os.path.join(_ROOT, out_filename))
    plt.close(fig)
    print(f"[OK] Saved 4-panel baseline plot: {out_art} (use_iso_sign_flip={use_iso_sign_flip})", flush=True)


if __name__ == "__main__":
    df_lat, df_yaw = load_aligned_data()
    generate_plausibility_plots(df_lat, df_yaw)

    sim_lat = run_axis_perturbation_sim("LatEC", df_lat, P15_RUN4, eval_cycles=2.0)
    render_baseline_4panel(sim_lat, "zero_shot_latec_roll_baseline_4panel.png")

    sim_yaw = run_axis_perturbation_sim("YawMA", df_yaw, P15_RUN4, eval_cycles=3.0)
    render_baseline_4panel(sim_yaw, "zero_shot_yawma_yaw_baseline_4panel.png")

    # Print summary tables
    print("\n================ ZERO-SHOT LatEC (ROLL) SUMMARY ================")
    for f in sim_lat["freqs"]:
        r = sim_lat["freq_results"][f"{f:.4f}"]
        print(f"  f={f:6.4f} Hz | Gain: {r['sim_gain']:7.2f} vs {r['target_gain']:7.2f} ({r['gain_db_diff']:+6.2f} dB) | Phase(raw): {r['sim_phase_raw']:7.2f}° (err={r['phase_err_raw']:+6.1f}°) | Phase(ISO): {r['sim_phase_inv']:7.2f}° (err={r['phase_err_inv']:+6.1f}°)")

    print("\n================ ZERO-SHOT YawMA (YAW) SUMMARY ================")
    for f in sim_yaw["freqs"]:
        r = sim_yaw["freq_results"][f"{f:.4f}"]
        print(f"  f={f:6.4f} Hz | Gain: {r['sim_gain']:7.3f} vs {r['target_gain']:7.3f} ({r['gain_db_diff']:+6.2f} dB) | Phase(raw): {r['sim_phase_raw']:7.2f}° (err={r['phase_err_raw']:+6.1f}°)")

    os.makedirs(os.path.join(_ROOT, "logs/roll_yaw_baseline"), exist_ok=True)
    with open(os.path.join(_ROOT, "logs/roll_yaw_baseline/zero_shot_summary.json"), "w") as f:
        json.dump({
            "LatEC": sim_lat["freq_results"],
            "YawMA": sim_yaw["freq_results"],
        }, f, indent=2)
