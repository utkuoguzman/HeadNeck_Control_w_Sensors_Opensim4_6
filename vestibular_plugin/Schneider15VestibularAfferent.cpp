#include "Schneider15VestibularAfferent.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace SimTK;

// ============================================================================
// CONSTRUCTOR
// ============================================================================
Schneider15VestibularAfferent::Schneider15VestibularAfferent() 
{
    constructProperties();
}

void Schneider15VestibularAfferent::constructProperties()
{
    // Default canal transfer function parameters
    constructProperty_canal_k(2.83);
    constructProperty_canal_T1(0.0175);
    constructProperty_canal_T2(0.0027);
    constructProperty_canal_Tc(5.7);

    // Otolith defaults (Regular Afferent from Fernandez & Goldberg 1976)
    constructProperty_otolith_KOTO(25.6);
    constructProperty_otolith_kA(1.12);
    constructProperty_otolith_tauA(69.0);
    constructProperty_otolith_tauM(0.016);
    constructProperty_otolith_kv(0.188); // alpha
    constructProperty_otolith_tauV(40.0);

    constructProperty_oustaloup_order(3);

    // Default thresholds for sensory deadband (Leans illusion)
    constructProperty_canal_threshold(2.0); // 2.0 deg/s
    constructProperty_otolith_threshold(0.005); // 0.005 Gs

    // Default anatomical orientation (Right Ear, based on Aw et al 1996)
    // Aw 1996 coordinates (x=Anterior, y=Left, z=Up) mapped to OpenSim (X=Anterior, Y=Up, Z=Right)
    constructProperty_location_in_skull(SimTK::Vec3(-0.01, -0.01, 0.05)); // Right inner ear approx location

    // Canal normals (These will also be used as orthogonal basis for Utricle/Saccule projection)
    constructProperty_normal_anterior(SimTK::Vec3(-0.04, 0.65, 0.76).normalize());
    constructProperty_normal_posterior(SimTK::Vec3(-0.64, 0.05, -0.77).normalize());
    constructProperty_normal_horizontal(SimTK::Vec3(0.18, 0.94, 0.27).normalize());
}

// ============================================================================
// STATE VARIABLES & ALLOCATION
// ============================================================================
void Schneider15VestibularAfferent::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    const std::string axes[3] = {"X", "Y", "Z"};
    int N = get_oustaloup_order();

    for (int i = 0; i < 3; ++i) {
        // Canal states
        addStateVariable("canal_x1_" + axes[i], SimTK::Stage::Dynamics);
        addStateVariable("canal_x2_" + axes[i], SimTK::Stage::Dynamics);

        // Otolith states
        addStateVariable("oto_x1_" + axes[i], SimTK::Stage::Dynamics); // Lead-lag
        addStateVariable("oto_y_" + axes[i], SimTK::Stage::Dynamics);  // Low-pass output

        // Otolith Oustaloup fractional derivative states
        for (int k = 1; k <= N; ++k) {
            addStateVariable("oto_w" + std::to_string(k) + "_" + axes[i], SimTK::Stage::Dynamics);
        }
    }
}

void Schneider15VestibularAfferent::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    // Initialize all states to 0 (steady state for zero movement)
    const std::string axes[3] = {"X", "Y", "Z"};
    int N = get_oustaloup_order();

    for (int i = 0; i < 3; ++i) {
        setStateVariableValue(s, "canal_x1_" + axes[i], 0.0);
        setStateVariableValue(s, "canal_x2_" + axes[i], 0.0);
        setStateVariableValue(s, "oto_x1_" + axes[i], 0.0);
        setStateVariableValue(s, "oto_y_" + axes[i], 0.0);

        for (int k = 1; k <= N; ++k) {
            setStateVariableValue(s, "oto_w" + std::to_string(k) + "_" + axes[i], 0.0);
        }
    }
}

// ============================================================================
// OUSTALOUP APPROXIMATION HELPER
// ============================================================================
void Schneider15VestibularAfferent::computeOustaloupFilter(double alpha, double tauV, int N, 
    std::vector<double>& zeros, std::vector<double>& poles, double& gain) const
{
    double wL = 0.01;
    double wH = 100.0;
    
    zeros.resize(N);
    poles.resize(N);

    double r = std::pow(wH / wL, 1.0 / N);
    
    zeros[0] = wL * std::sqrt(r);
    poles[0] = zeros[0] * std::pow(r, alpha);
    
    for (int k = 1; k < N; ++k) {
        zeros[k] = poles[k-1] * std::pow(r, 1.0 - alpha);
        poles[k] = zeros[k] * std::pow(r, alpha);
    }

    gain = std::pow(tauV * wH, alpha);
}

// ============================================================================
// DYNAMICS
// ============================================================================
void Schneider15VestibularAfferent::computeStateVariableDerivatives(const SimTK::State& s) const
{
    if (!getSocket<PhysicalFrame>("head_frame").isConnected()) {
        return; // Do nothing if not connected
    }

    const PhysicalFrame& head = getConnectee<PhysicalFrame>("head_frame");
    const Ground& ground = getModel().getGround();

    // Get Head kinematics in global frame
    Vec3 w = head.getAngularVelocityInGround(s); // rad/s
    Vec3 alpha = head.getAngularAccelerationInGround(s); // rad/s^2
    Vec3 r_G = head.getTransformInGround(s).R() * get_location_in_skull(); // offset in ground frame
    
    // Acceleration at the sensor location: a_sensor = a_origin + alpha x r + w x (w x r)
    Vec3 linAccGlobal = head.getLinearAccelerationInGround(s) + (alpha % r_G) + (w % (w % r_G)); // m/s^2
    Vec3 angVelGlobal = w;

    // Biologically, otoliths also sense gravity.
    // In OpenSim, linear acceleration does not include gravity. We must add it in the upward direction 
    // to simulate the "gravity-in" force felt by the otoliths.
    Vec3 gravity = getModel().getGravity(); 
    Vec3 sensedAccGlobal = linAccGlobal - gravity; 

    // Convert to degrees/s and Gs for the transfer functions
    Vec3 angVelDegGlobal = angVelGlobal * (180.0 / SimTK::Pi);
    Vec3 sensedAccGsGlobal = sensedAccGlobal / 9.80665; 

    // Express kinematics in the local head frame
    Vec3 u_ang = ground.expressVectorInAnotherFrame(s, angVelDegGlobal, head);
    Vec3 u_acc = ground.expressVectorInAnotherFrame(s, sensedAccGsGlobal, head);

    const std::string axes[3] = {"X", "Y", "Z"};
    
    // --- Canal Parameters ---
    double k = get_canal_k();
    double a = 1.0 / get_canal_T1();
    double b = 1.0 / get_canal_Tc();
    double c = 1.0 / get_canal_T2();
    double alpha1 = b + c;
    double alpha2 = b * c;
    double beta0 = k;
    double beta1 = k * a;
    
    // --- Otolith Parameters ---
    double KOTO = get_otolith_KOTO();
    double kA = get_otolith_kA();
    double tauA = get_otolith_tauA();
    double tauM = get_otolith_tauM();
    double kv = get_otolith_kv();
    double tauV = get_otolith_tauV();
    int N = get_oustaloup_order();

    std::vector<double> z(N), p(N);
    double K_oust;
    computeOustaloupFilter(kv, tauV, N, z, p, K_oust);

    // Get anatomical normal vectors
    Vec3 n_anterior = get_normal_anterior();
    Vec3 n_posterior = get_normal_posterior();
    Vec3 n_horizontal = get_normal_horizontal();

    // ------------------------------------------------------------------------
    // Semicircular Canals (Project angular velocity)
    // ------------------------------------------------------------------------
    double u_canal_proj[3];
    u_canal_proj[0] = dot(u_ang, n_anterior);   // Anterior Canal
    u_canal_proj[1] = dot(u_ang, n_posterior);  // Posterior Canal
    u_canal_proj[2] = dot(u_ang, n_horizontal); // Horizontal Canal

    double u_acc_proj[3];
    u_acc_proj[0] = dot(u_acc, n_anterior);   // Utricle roughly aligned with Anterior
    u_acc_proj[1] = dot(u_acc, n_posterior);  // Utricle roughly aligned with Posterior
    u_acc_proj[2] = dot(u_acc, n_horizontal); // Saccule roughly aligned with Horizontal (Vertical normal)

    // ------------------------------------------------------------------------
    // Apply Threshold (Deadband) for Leans Illusion
    // ------------------------------------------------------------------------
    auto apply_deadband = [](double value, double threshold) -> double {
        if (value > threshold) return value - threshold;
        if (value < -threshold) return value + threshold;
        return 0.0;
    };

    double thr_canal = get_canal_threshold();
    double thr_oto = get_otolith_threshold();

    for (int i = 0; i < 3; ++i) {
        u_canal_proj[i] = apply_deadband(u_canal_proj[i], thr_canal);
        u_acc_proj[i] = apply_deadband(u_acc_proj[i], thr_oto);
    }

    for (int i = 0; i < 3; ++i) {
        // -----------------------------------------------------------
        // Semicircular Canal ODEs
        // -----------------------------------------------------------
        double u_canal = u_canal_proj[i];
        double x1_c = getStateVariableValue(s, "canal_x1_" + axes[i]);
        double x2_c = getStateVariableValue(s, "canal_x2_" + axes[i]);
        
        double x1_c_dot = -alpha1 * x1_c + x2_c + (beta1 - alpha1 * beta0) * u_canal;
        double x2_c_dot = -alpha2 * x1_c + (0.0 - alpha2 * beta0) * u_canal;
        
        setStateVariableDerivativeValue(s, "canal_x1_" + axes[i], x1_c_dot);
        setStateVariableDerivativeValue(s, "canal_x2_" + axes[i], x2_c_dot);

        // -----------------------------------------------------------
        // Otolith ODEs
        // -----------------------------------------------------------
        double u_oto = u_acc_proj[i];
        
        // 1. Lead-Lag Filter
        double x1_oto = getStateVariableValue(s, "oto_x1_" + axes[i]);
        double x1_oto_dot = (u_oto - x1_oto) / tauA;
        setStateVariableDerivativeValue(s, "oto_x1_" + axes[i], x1_oto_dot);
        
        double y1 = KOTO * (kA * u_oto + (1.0 - kA) * x1_oto);

        // 2. Fractional Derivative (Oustaloup Cascade)
        double v = y1;
        for (int k_idx = 1; k_idx <= N; ++k_idx) {
            double w = getStateVariableValue(s, "oto_w" + std::to_string(k_idx) + "_" + axes[i]);
            double w_dot = -p[k_idx-1] * w + v;
            setStateVariableDerivativeValue(s, "oto_w" + std::to_string(k_idx) + "_" + axes[i], w_dot);
            v = v + (z[k_idx-1] - p[k_idx-1]) * w;
        }
        double y_frac = K_oust * v;

        // 3. Final Low-Pass Filter
        double y_oto = getStateVariableValue(s, "oto_y_" + axes[i]);
        double y_oto_dot = (y1 + kv * y_frac - y_oto) / tauM;
        setStateVariableDerivativeValue(s, "oto_y_" + axes[i], y_oto_dot);
    }
}

// ============================================================================
// OUTPUT GENERATION
// ============================================================================
SimTK::Vec3 Schneider15VestibularAfferent::getCanalFiringRate(const SimTK::State& s) const
{
    if (!getSocket<PhysicalFrame>("head_frame").isConnected()) return Vec3(0);

    const PhysicalFrame& head = getConnectee<PhysicalFrame>("head_frame");
    const Ground& ground = getModel().getGround();
    
    Vec3 angVelGlobal = head.getAngularVelocityInGround(s);
    Vec3 u_ang = ground.expressVectorInAnotherFrame(s, angVelGlobal * (180.0 / SimTK::Pi), head);

    Vec3 n_anterior = get_normal_anterior();
    Vec3 n_posterior = get_normal_posterior();
    Vec3 n_horizontal = get_normal_horizontal();
    
    double u_canal_proj[3];
    u_canal_proj[0] = dot(u_ang, n_anterior);
    u_canal_proj[1] = dot(u_ang, n_posterior);
    u_canal_proj[2] = dot(u_ang, n_horizontal);

    auto apply_deadband = [](double value, double threshold) -> double {
        if (value > threshold) return value - threshold;
        if (value < -threshold) return value + threshold;
        return 0.0;
    };
    double thr_canal = get_canal_threshold();
    
    for (int i = 0; i < 3; ++i) {
        u_canal_proj[i] = apply_deadband(u_canal_proj[i], thr_canal);
    }

    double k = get_canal_k();
    Vec3 rate(0);
    const std::string axes[3] = {"X", "Y", "Z"};

    for (int i = 0; i < 3; ++i) {
        double x1 = getStateVariableValue(s, "canal_x1_" + axes[i]);
        rate[i] = k * u_canal_proj[i] + x1;
    }
    return rate;
}

SimTK::Vec3 Schneider15VestibularAfferent::getOtolithFiringRate(const SimTK::State& s) const
{
    Vec3 rate(0);
    const std::string axes[3] = {"X", "Y", "Z"};

    for (int i = 0; i < 3; ++i) {
        rate[i] = getStateVariableValue(s, "oto_y_" + axes[i]);
    }
    return rate;
}

