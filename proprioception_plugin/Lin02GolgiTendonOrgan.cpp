/*
* Lin & Crago (2002) Golgi Tendon Organ (Ib) model with exact derivative-free
* partial-fraction realization, tendon-force input (including optional SRS),
* strength-normalized Gf scale, eye-muscle gating, and Stage::Velocity state output.
*/

#include <iostream>
#include <algorithm>
#include <cmath>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include "Millard12EqMuscleWithAfferents.h"

using namespace OpenSim;

Lin02GolgiTendonOrgan::Lin02GolgiTendonOrgan() {
    constructProperties();
    Gg = 60.0;
    Gf = 4.0;
    thr = 0.0;
}

void Lin02GolgiTendonOrgan::constructProperties() {
    setAuthors("Sergio Verduzco");
    constructProperty_lpf_tau(0.01);
    constructProperty_enable_GTO(false);
    constructProperty_gf_fraction(0.08);
    constructProperty_single_afferent_scale(true);
}

void Lin02GolgiTendonOrgan::setLPFtau(double aLPFtau) {
    set_lpf_tau(aLPFtau);
}

void Lin02GolgiTendonOrgan::setOwnerMuscleName(std::string OwnerMuscleName) {
    ownerMuscleName = std::move(OwnerMuscleName);
}

bool Lin02GolgiTendonOrgan::isGTOActive() const {
    if (!get_enable_GTO()) return false;
    if (const auto* aff_mus = dynamic_cast<const Millard12EqMuscleWithAfferents*>(musclePtr)) {
        return aff_mus->isAfferentActive();
    }
    const bool is_eye = (ownerMuscleName.rfind("r_", 0) == 0 || ownerMuscleName.rfind("l_", 0) == 0);
    return !is_eye;
}

void Lin02GolgiTendonOrgan::ensureStatePointers() const {
    if (!sv_gto_w1) {
        sv_gto_w1  = traverseToStateVariable("gto_w1");
        sv_gto_w2  = traverseToStateVariable("gto_w2");
        sv_gto_out = traverseToStateVariable("gto_out_state");
    }
}

double Lin02GolgiTendonOrgan::getGTOout(const SimTK::State& s) const {
    if (!isGTOActive()) return 0.0;
    ensureStatePointers();
    return sv_gto_out ? sv_gto_out->getValue(s) : getStateVariableValue(s, "gto_out_state");
}

double Lin02GolgiTendonOrgan::computeNonlinearDrive(const SimTK::State& s) const {
    if (!musclePtr) return 0.0;
    const double f_tot = std::max(0.0, musclePtr->getTendonForce(s));
    const double gf_frac = get_gf_fraction();
    const double gf_eff = (gf_frac > 0.0)
        ? std::max(1.5, gf_frac * musclePtr->getMaxIsometricForce())
        : std::max(1e-3, Gf);
    return Gg * std::log(1.0 + f_tot / gf_eff);
}

void Lin02GolgiTendonOrgan::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    if (!isGTOActive()) return;

    addStateVariable("gto_w1", SimTK::Stage::Velocity);
    addStateVariable("gto_w2", SimTK::Stage::Velocity);
    addStateVariable("gto_out_state", SimTK::Stage::Velocity);
}

void Lin02GolgiTendonOrgan::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    sv_gto_w1 = nullptr;
    sv_gto_w2 = nullptr;
    sv_gto_out = nullptr;
    if (!isGTOActive()) return;
    ensureStatePointers();
    if (sv_gto_w1) sv_gto_w1->setValue(s, 0.0); else setStateVariableValue(s, "gto_w1", 0.0);
    if (sv_gto_w2) sv_gto_w2->setValue(s, 0.0); else setStateVariableValue(s, "gto_w2", 0.0);
    if (sv_gto_out) sv_gto_out->setValue(s, 0.0); else setStateVariableValue(s, "gto_out_state", 0.0);
}

void Lin02GolgiTendonOrgan::extendSetPropertiesFromState(const SimTK::State& s) {
    Super::extendSetPropertiesFromState(s);
}

void Lin02GolgiTendonOrgan::extendConnectToModel(Model& aModel) {
    Super::extendConnectToModel(aModel);
    if (aModel.getMuscles().contains(ownerMuscleName)) {
        musclePtr = &(aModel.getMuscles().get(ownerMuscleName));
    } else {
        musclePtr = nullptr;
    }
}

void Lin02GolgiTendonOrgan::initFromMuscle(SimTK::State& s) const {
    if (!isGTOActive() || !musclePtr) return;
    ensureStatePointers();
    if (s.getSystemStage() < SimTK::Stage::Velocity && hasModel()) {
        getModel().getMultibodySystem().realize(s, SimTK::Stage::Velocity);
    }
    const double u0 = computeNonlinearDrive(s);
    const double scale = get_single_afferent_scale() ? 1.0 : 40.0;
    const double y0 = scale * u0; // Exact DC gain 1.7 - (17/15)/2 - (2/75)/0.2 = 1.0
    const double y_diff = y0 - thr;
    const double y_smooth0 = 0.5 * (y0 + thr + std::sqrt(y_diff * y_diff + 1e-4));

    if (sv_gto_w1) sv_gto_w1->setValue(s, u0 / 2.0); else setStateVariableValue(s, "gto_w1", u0 / 2.0);
    if (sv_gto_w2) sv_gto_w2->setValue(s, u0 / 0.2); else setStateVariableValue(s, "gto_w2", u0 / 0.2);
    if (sv_gto_out) sv_gto_out->setValue(s, y_smooth0); else setStateVariableValue(s, "gto_out_state", y_smooth0);
}

void Lin02GolgiTendonOrgan::computeStateVariableDerivatives(const SimTK::State& s) const {
    if (!isGTOActive()) return;
    ensureStatePointers();

    const double u_nl = computeNonlinearDrive(s);
    const double w1 = sv_gto_w1 ? sv_gto_w1->getValue(s) : getStateVariableValue(s, "gto_w1");
    const double w2 = sv_gto_w2 ? sv_gto_w2->getValue(s) : getStateVariableValue(s, "gto_w2");
    const double y_out = sv_gto_out ? sv_gto_out->getValue(s) : getStateVariableValue(s, "gto_out_state");

    const double dw1 = -2.0 * w1 + u_nl;
    const double dw2 = -0.2 * w2 + u_nl;

    const double scale = get_single_afferent_scale() ? 1.0 : 40.0;
    const double y_raw = scale * (1.7 * u_nl - (17.0 / 15.0) * w1 - (2.0 / 75.0) * w2);
    const double y_diff = y_raw - thr;
    const double y_rect = 0.5 * (y_raw + thr + std::sqrt(y_diff * y_diff + 1e-4));
    const double dy_out = 500.0 * (y_rect - y_out);

    if (sv_gto_w1) sv_gto_w1->setDerivative(s, dw1); else setStateVariableDerivativeValue(s, "gto_w1", dw1);
    if (sv_gto_w2) sv_gto_w2->setDerivative(s, dw2); else setStateVariableDerivativeValue(s, "gto_w2", dw2);
    if (sv_gto_out) sv_gto_out->setDerivative(s, dy_out); else setStateVariableDerivativeValue(s, "gto_out_state", dy_out);
}
