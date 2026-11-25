// INCLUDES
#include "Millard12EqMuscleWithAfferents.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std;
using namespace OpenSim;

Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents() {
    constructProperties();
    upd_spindle().setOwnerMuscleName("no_spindle_name");
    upd_GTO().setOwnerMuscleName("no_GTO_name");
}

Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents(
    const std::string &name, double maxIsometricForce, double optimalFiberLength,
    double tendonSlackLength, double pennationAngle)
    : Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength, pennationAngle) {
    constructProperties();
    upd_spindle().setOwnerMuscleName(getName());
    upd_GTO().setOwnerMuscleName(getName());
}

void Millard12EqMuscleWithAfferents::ensureStatePointers() const {
    if (!sv_lpf_vel) {
        sv_lpf_vel = traverseToStateVariable("LPF_velocity");
        sv_lpf_acc = traverseToStateVariable("LPF_acceleration");
    }
    if (get_enable_SRS() && !sv_srs_anchor) {
        sv_srs_anchor = traverseToStateVariable("srs_anchor");
        sv_srs_thix   = traverseToStateVariable("srs_thix");
        sv_activation = traverseToStateVariable("activation");
    }
}

double Millard12EqMuscleWithAfferents::getLPFvelocity(const SimTK::State& s) const {
    if (!isAfferentActive()) return 0.0;
    ensureStatePointers();
    return sv_lpf_vel ? sv_lpf_vel->getValue(s) : getStateVariableValue(s, "LPF_velocity");
}

void Millard12EqMuscleWithAfferents::setLPFvelocity(SimTK::State& s, double Velocity) const {
    if (!isAfferentActive()) return;
    ensureStatePointers();
    if (sv_lpf_vel) sv_lpf_vel->setValue(s, Velocity);
    else setStateVariableValue(s, "LPF_velocity", Velocity);
}

double Millard12EqMuscleWithAfferents::getLPFacceleration(const SimTK::State& s) const {
    if (!isAfferentActive()) return 0.0;
    ensureStatePointers();
    return sv_lpf_acc ? sv_lpf_acc->getValue(s) : getStateVariableValue(s, "LPF_acceleration");
}

void Millard12EqMuscleWithAfferents::setLPFacceleration(SimTK::State& s, double Acceleration) const {
    if (!isAfferentActive()) return;
    ensureStatePointers();
    if (sv_lpf_acc) sv_lpf_acc->setValue(s, Acceleration);
    else setStateVariableValue(s, "LPF_acceleration", Acceleration);
}

double Millard12EqMuscleWithAfferents::getSRSAnchor(const SimTK::State& s) const {
    if (!get_enable_SRS() || !isAfferentActive()) return 1.0;
    ensureStatePointers();
    return sv_srs_anchor ? sv_srs_anchor->getValue(s) : getStateVariableValue(s, "srs_anchor");
}

void Millard12EqMuscleWithAfferents::setSRSAnchor(SimTK::State& s, double anchorLen) const {
    if (!get_enable_SRS() || !isAfferentActive()) return;
    ensureStatePointers();
    if (sv_srs_anchor) sv_srs_anchor->setValue(s, anchorLen);
    else setStateVariableValue(s, "srs_anchor", anchorLen);
}

double Millard12EqMuscleWithAfferents::getSRSThix(const SimTK::State& s) const {
    if (!get_enable_SRS() || !isAfferentActive()) return 1.0;
    ensureStatePointers();
    return sv_srs_thix ? sv_srs_thix->getValue(s) : getStateVariableValue(s, "srs_thix");
}

void Millard12EqMuscleWithAfferents::setSRSThix(SimTK::State& s, double thixVal) const {
    if (!get_enable_SRS() || !isAfferentActive()) return;
    ensureStatePointers();
    if (sv_srs_thix) sv_srs_thix->setValue(s, thixVal);
    else setStateVariableValue(s, "srs_thix", thixVal);
}

void Millard12EqMuscleWithAfferents::setLPFtau(double aLPFtau) {
    set_lpf_tau(aLPFtau);
}

void Millard12EqMuscleWithAfferents::constructProperties() {
    setAuthors("Utku Oguzman");
    constructProperty_lpf_tau(0.01);
    constructProperty_enable_eye_afferents(false);
    constructProperty_enable_SRS(false);
    constructProperty_k_srs(23.4);
    constructProperty_delta_srs(0.018);
    constructProperty_tau_srs_reset(0.400);
    constructProperty_tau_srs_short(0.025);
    constructProperty_srs_thix_min(0.35);
    constructProperty_tau_srs_thix_rec(1.5);
    constructProperty_delta_srs_thix_break(0.05);
    constructProperty_spindle(Mileusnic06Spindle());
    constructProperty_GTO(Lin02GolgiTendonOrgan());
}

void Millard12EqMuscleWithAfferents::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    if (!isAfferentActive()) return;

    addStateVariable("LPF_velocity");
    addStateVariable("LPF_acceleration");
    if (get_enable_SRS()) {
        // Invalidates Stage::Velocity so calcFiberVelocityInfo and calcMuscleDynamicsInfo
        // always see up-to-date cross-bridge anchor and thixotropy states.
        addStateVariable("srs_anchor", SimTK::Stage::Velocity);
        addStateVariable("srs_thix", SimTK::Stage::Velocity);
    }
}

void Millard12EqMuscleWithAfferents::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    sv_lpf_vel = nullptr;
    sv_lpf_acc = nullptr;
    sv_srs_anchor = nullptr;
    sv_srs_thix = nullptr;
    sv_activation = nullptr;
    active_checked = false;
    if (!isAfferentActive()) return;

    setLPFvelocity(s, 0.0);
    setLPFacceleration(s, 0.0);
    if (get_enable_SRS()) {
        setSRSAnchor(s, 1.0);
        setSRSThix(s, 1.0);
    }
}

void Millard12EqMuscleWithAfferents::extendSetPropertiesFromState(const SimTK::State& s) {
    Super::extendSetPropertiesFromState(s);
}

void Millard12EqMuscleWithAfferents::extendConnectToModel(Model& aModel) {
    upd_spindle().setOwnerMuscleName(getName());
    upd_GTO().setOwnerMuscleName(getName());
    Super::extendConnectToModel(aModel);
}

double Millard12EqMuscleWithAfferents::calcSRSTensionAlongTendon(const SimTK::State& s) const {
    if (!get_enable_SRS() || !isAfferentActive()) return 0.0;
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    const double l_norm = mli.normFiberLength;
    const double l_anc  = getSRSAnchor(s);
    const double dl     = l_norm - l_anc;
    if (std::abs(dl) < 1e-12) return 0.0;
    const double s_min  = std::clamp(get_srs_thix_min(), 0.05, 1.0);
    const double s_thix = std::clamp(getSRSThix(s), s_min, 1.0);
    const double delta  = std::max(0.002, get_delta_srs());
    const double d_fsrs = std::max(-0.95, s_thix * get_k_srs() * delta * std::tanh(dl / delta));
    const double act_raw = getIgnoreActivationDynamics(s)
        ? getExcitation(s)
        : (sv_activation ? sv_activation->getValue(s) : getStateVariableValue(s, "activation"));
    const double act    = std::clamp(act_raw, getMinimumActivation(), 1.0);
    const double fl     = std::max(0.0, mli.fiberActiveForceLengthMultiplier);
    const double cos_p  = std::max(0.0, mli.cosPennationAngle);
    return (act * fl * getMaxIsometricForce()) * d_fsrs * cos_p;
}

double Millard12EqMuscleWithAfferents::getTotalTendonForce(const SimTK::State& s) const {
    // SRS is equilibrated inside the fiber in series with the tendon, so getTendonForce(s)
    // is the exact equilibrated total tendon tension.
    return std::max(0.0, getTendonForce(s));
}

void Millard12EqMuscleWithAfferents::calcFiberVelocityInfo(
    const SimTK::State& s,
    FiberVelocityInfo& fvi) const {
    if (!get_enable_SRS() || !isAfferentActive() || getIgnoreTendonCompliance(s)) {
        Super::calcFiberVelocityInfo(s, fvi);
        return;
    }

    ensureStatePointers();
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    const double lopt   = getOptimalFiberLength();
    const double vmax   = getMaxContractionVelocity();
    const double lts    = getTendonSlackLength();
    const double lce    = mli.fiberLength;
    const double lceN   = mli.normFiberLength;
    const double cosPhi = std::max(1e-6, mli.cosPennationAngle);
    const double sinPhi = mli.sinPennationAngle;
    const double fl     = std::max(0.0, mli.fiberActiveForceLengthMultiplier);
    const double fpe    = mli.fiberPassiveForceLengthMultiplier;
    const double ft     = getTendonForceLengthCurve().calcValue(mli.normTendonLength);
    // Read activation state directly (never call getActivation(s), which queries
    // MuscleDynamicsInfo at Stage::Dynamics and would recurse back into calcFiberVelocityInfo!)
    const double act_raw = getIgnoreActivationDynamics(s)
        ? getExcitation(s)
        : (sv_activation ? sv_activation->getValue(s) : getStateVariableValue(s, "activation"));
    const double act    = std::clamp(act_raw, getMinimumActivation(), 1.0);
    const double beta   = getUseFiberDamping() ? getFiberDamping() : 0.0;

    // Monotonic, distortion-free cross-bridge SRS force (in series with tendon):
    //   f_T / cos(phi) = a * f_L * (f_V(v_N) + d_fsrs_norm) + f_PE + beta * v_N
    const double l_anc  = getSRSAnchor(s);
    const double s_min  = std::clamp(get_srs_thix_min(), 0.05, 1.0);
    const double s_thix = std::clamp(getSRSThix(s), s_min, 1.0);
    const double dl     = lceN - l_anc;
    const double delta  = std::max(0.002, get_delta_srs());
    const double th     = std::tanh(dl / delta);
    const double d_fsrs_norm = std::max(-0.95, s_thix * get_k_srs() * delta * th);

    const double afl = act * fl;
    const double rhs = (ft / cosPhi) - fpe - afl * d_fsrs_norm;
    const ForceVelocityCurve& fvCurve = getForceVelocityCurve();

    // Safeguarded Newton-Raphson on strictly monotonic g(v) = afl * f_V(v) + beta * v - rhs = 0
    double dlceN = 0.0;
    if (afl > 1e-10 || beta > 1e-10) {
        double v = 0.0;
        double lo = -10.0, hi = 10.0;
        for (int iter = 0; iter < 20; ++iter) {
            const double fv  = fvCurve.calcValue(v);
            const double dfv = fvCurve.calcDerivative(v, 1);
            const double res = afl * fv + beta * v - rhs;
            if (std::abs(res) < 1e-11) break;
            if (res < 0.0) lo = std::max(lo, v);
            else           hi = std::min(hi, v);
            const double dres = afl * dfv + beta;
            double v_next = v;
            if (dres > 1e-10) {
                double dv = -res / dres;
                dv = std::clamp(dv, -0.75, 0.75);
                v_next = v + dv;
            }
            if (!(v_next > lo && v_next < hi)) {
                v_next = 0.5 * (lo + hi);
            }
            if (std::abs(v_next - v) < 1e-12) {
                v = v_next;
                break;
            }
            v = v_next;
        }
        dlceN = v;
    }

    // Clamp shortening velocity if fiber has reached minimum allowable length
    if (lce <= getMinimumFiberLength() && dlceN < 0.0) {
        dlceN = 0.0;
    }

    const double fv_val  = fvCurve.calcValue(dlceN);
    const double dlce    = dlceN * vmax * lopt;
    const double tanPhi  = sinPhi / cosPhi;
    const MuscleFixedWidthPennationModel& penMdl = getPennationModel();
    const double dphi    = penMdl.calcPennationAngularVelocity(tanPhi, lce, dlce);
    const double dlceAT  = penMdl.calcFiberVelocityAlongTendon(lce, dlce, sinPhi, cosPhi, dphi);
    const double v_mt    = getLengtheningSpeed(s);
    const double dlt     = penMdl.calcTendonVelocity(cosPhi, sinPhi, dphi, lce, dlce, v_mt);
    const double dltN    = dlt / lts;

    fvi.fiberVelocity                = dlce;
    fvi.fiberVelocityAlongTendon     = dlceAT;
    fvi.normFiberVelocity            = dlceN;
    fvi.pennationAngularVelocity     = dphi;
    fvi.tendonVelocity               = dlt;
    fvi.normTendonVelocity           = dltN;
    fvi.fiberForceVelocityMultiplier = fv_val;

    fvi.userDefinedVelocityExtras.resize(3);
    fvi.userDefinedVelocityExtras[0] = d_fsrs_norm;
    fvi.userDefinedVelocityExtras[1] = s_thix;
    fvi.userDefinedVelocityExtras[2] = 1.0 - th * th; // sech^2(dl / delta) for tangent stiffness
}

void Millard12EqMuscleWithAfferents::calcMuscleDynamicsInfo(
    const SimTK::State& s,
    MuscleDynamicsInfo& mdi) const {
    if (!get_enable_SRS() || !isAfferentActive() || getIgnoreTendonCompliance(s)) {
        Super::calcMuscleDynamicsInfo(s, mdi);
        return;
    }

    ensureStatePointers();
    const MuscleLengthInfo& mli  = getMuscleLengthInfo(s);
    const FiberVelocityInfo& fvi = getFiberVelocityInfo(s);

    const double fiso   = getMaxIsometricForce();
    const double lopt   = getOptimalFiberLength();
    const double lts    = getTendonSlackLength();
    const double lce    = std::max(1e-6, mli.fiberLength);
    const double cosPhi = std::max(1e-6, mli.cosPennationAngle);
    const double sinPhi = mli.sinPennationAngle;
    const double fl     = std::max(0.0, mli.fiberActiveForceLengthMultiplier);
    const double fpe    = mli.fiberPassiveForceLengthMultiplier;
    const double ft     = getTendonForceLengthCurve().calcValue(mli.normTendonLength);
    const double beta   = getUseFiberDamping() ? getFiberDamping() : 0.0;
    const double act_raw = getIgnoreActivationDynamics(s)
        ? getExcitation(s)
        : (sv_activation ? sv_activation->getValue(s) : getStateVariableValue(s, "activation"));
    const double act    = std::clamp(act_raw, getMinimumActivation(), 1.0);

    double d_fsrs_norm = 0.0;
    double s_thix = 1.0;
    double sech2 = 1.0;
    if (fvi.userDefinedVelocityExtras.size() >= 3) {
        d_fsrs_norm = fvi.userDefinedVelocityExtras[0];
        s_thix      = fvi.userDefinedVelocityExtras[1];
        sech2       = fvi.userDefinedVelocityExtras[2];
    } else {
        const double dl    = mli.normFiberLength - getSRSAnchor(s);
        const double delta = std::max(0.002, get_delta_srs());
        const double th    = std::tanh(dl / delta);
        const double s_min = std::clamp(get_srs_thix_min(), 0.05, 1.0);
        s_thix      = std::clamp(getSRSThix(s), s_min, 1.0);
        d_fsrs_norm = std::max(-0.95, s_thix * get_k_srs() * delta * th);
        sech2       = 1.0 - th * th;
    }

    const double fv        = fvi.fiberForceVelocityMultiplier;
    const double f_act_N   = fiso * act * fl * (fv + d_fsrs_norm);
    const double f_pe_el   = fiso * fpe;
    const double f_pe_damp = fiso * beta * fvi.normFiberVelocity;
    const double f_pass_N  = f_pe_el + f_pe_damp;
    const double f_fib_N   = f_act_N + f_pass_N;

    mdi.activation            = act;
    mdi.activeFiberForce      = f_act_N;
    mdi.passiveFiberForce     = f_pass_N;
    mdi.fiberForce            = f_fib_N;
    mdi.fiberForceAlongTendon = f_fib_N * cosPhi;
    mdi.normFiberForce        = f_fib_N / fiso;
    mdi.normTendonForce       = ft;
    mdi.tendonForce           = fiso * ft;

    const double df_fl     = getActiveForceLengthCurve().calcDerivative(mli.normFiberLength, 1);
    const double df_pe     = getFiberForceLengthCurve().calcDerivative(mli.normFiberLength, 1);
    const double df_t      = getTendonForceLengthCurve().calcDerivative(mli.normTendonLength, 1);
    const double k_srs_tan = (fiso / lopt) * act * fl * s_thix * get_k_srs() * sech2;

    mdi.fiberStiffness = (fiso / lopt) * (act * df_fl * (fv + d_fsrs_norm) + df_pe) + k_srs_tan;
    const double dfAT_dlce = mdi.fiberStiffness * cosPhi + (f_fib_N * sinPhi * sinPhi) / (lce * cosPhi);
    mdi.fiberStiffnessAlongTendon = dfAT_dlce * cosPhi;
    mdi.tendonStiffness           = (fiso / lts) * df_t;

    mdi.fiberActivePower  = -mdi.activeFiberForce * fvi.fiberVelocity;
    mdi.fiberPassivePower = -mdi.passiveFiberForce * fvi.fiberVelocity;
    mdi.tendonPower       = -mdi.tendonForce * fvi.tendonVelocity;

    mdi.userDefinedDynamicsExtras.resize(2);
    mdi.userDefinedDynamicsExtras[0] = f_pe_el;
    mdi.userDefinedDynamicsExtras[1] = f_pe_damp;
}

void Millard12EqMuscleWithAfferents::computeInitialFiberEquilibrium(SimTK::State& s) const {
    Super::computeInitialFiberEquilibrium(s);
    if (!isAfferentActive()) return;

    // Anchor SRS to equilibrated fiber length BEFORE querying getFiberVelocity(s),
    // so calcFiberVelocityInfo evaluates at dl = 0 (exact zero-velocity equilibrium).
    if (get_enable_SRS()) {
        setSRSAnchor(s, getNormalizedFiberLength(s));
        setSRSThix(s, 1.0);
    }

    double v0 = getFiberVelocity(s);
    setLPFvelocity(s, (std::abs(v0) < 1e-4) ? 0.0 : v0);
    setLPFacceleration(s, 0.0);

    get_spindle().computeInitialSpindleEquilibrium(s);
    get_GTO().initFromMuscle(s);
}

void Millard12EqMuscleWithAfferents::computeStateVariableDerivatives(const SimTK::State& s) const {
    Super::computeStateVariableDerivatives(s);
    if (!isAfferentActive()) return;
    ensureStatePointers();

    double lpf_v = sv_lpf_vel->getValue(s);
    double lpf_a = sv_lpf_acc->getValue(s);
    double tau = cached_lpf_tau;
    double curr_accel = (getFiberVelocity(s) - lpf_v) / tau;
    sv_lpf_vel->setDerivative(s, curr_accel);
    sv_lpf_acc->setDerivative(s, (curr_accel - lpf_a) / tau);

    if (get_enable_SRS() && sv_srs_anchor && sv_srs_thix) {
        const double l_norm = getNormalizedFiberLength(s);
        const double l_anc  = sv_srs_anchor->getValue(s);
        const double dl     = l_norm - l_anc;
        const double delta  = std::max(0.002, get_delta_srs());
        const double r      = dl / delta;

        // C-infinity smooth transition: slow symmetric turnover tau_turn (0.40 s) inside
        // small postural/Bode oscillations (|dl| < 0.75*delta); smooth quadratic slip above
        // yield stretch; fast detachment tau_short (0.025 s) on large active shortening.
        const double tau_turn  = std::max(0.01, get_tau_srs_reset());
        const double tau_short = std::max(0.005, get_tau_srs_short());
        const double w_short   = 0.5 * (1.0 - std::tanh((dl + 0.75 * delta) / (0.25 * delta)));
        const double inv_tau   = (1.0 - w_short) / tau_turn + w_short / tau_short;
        const double rate      = std::min(60.0, (1.0 + r * r) * inv_tau);
        sv_srs_anchor->setDerivative(s, rate * dl);

        // Thixotropy (movement-history) state s_thix in [s_min, 1.0]:
        // Full 1.0 at quiet posture (rapid sled protection), decays smoothly with cumulative
        // fiber excursion |v_M/L_0| / delta_break during continuous oscillation or turns,
        // and recovers slowly at rest (tau_rec ~ 1.5 s).
        const double s_thix  = sv_srs_thix->getValue(s);
        const double s_min   = std::clamp(get_srs_thix_min(), 0.05, 1.0);
        const double tau_rec = std::max(0.05, get_tau_srs_thix_rec());
        const double d_break = std::max(0.005, get_delta_srs_thix_break());
        const double v_l0    = std::abs(getFiberVelocity(s)) / std::max(1e-4, getOptimalFiberLength());
        const double k_break = std::min(40.0, v_l0 / d_break);
        sv_srs_thix->setDerivative(s, (1.0 - s_thix) / tau_rec - k_break * std::max(0.0, s_thix - s_min));
    }
}

double Millard12EqMuscleWithAfferents::approxFiberAcceleration(const SimTK::State& s) const {
    return (getFiberVelocity(s) - getLPFvelocity(s)) / getLPFtau();
}
