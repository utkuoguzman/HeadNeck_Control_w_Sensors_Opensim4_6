/* -------------------------------------------------------------------------- *
 *                     OpenSim:  DynamicLigamentForce.cpp                     *
 * -------------------------------------------------------------------------- *
 * Ported to OpenSim 4.6 from Calvin Kuo et al. (2019),                       *
 * "Passive cervical spine ligaments provide stability during head impacts",  *
 * J. R. Soc. Interface 16: 20190086.                                         *
 * -------------------------------------------------------------------------- */

#include "DynamicLigamentForce.h"
#include <algorithm>
#include <cmath>

using namespace OpenSim;

namespace {
const SimTK::Vec3 kDefaultLigamentColor(0.9, 0.9, 0.1);
} // namespace

DynamicLigamentForce::DynamicLigamentForce() {
    constructProperties();
}

void DynamicLigamentForce::constructProperties() {
    setAuthors("Calvin Kuo (OpenSim 4.6 port)");
    constructProperty_GeometryPath(GeometryPath());
    constructProperty_resting_length(0.0);
    constructProperty_cross_sectional_area(0.0);
    constructProperty_toe_strain(1.0);
    constructProperty_toe_modulus(0.0);
    constructProperty_linear_modulus(0.0);
    constructProperty_rate_scale(0.0);
    constructProperty_rate_intercept(0.0);
}

void DynamicLigamentForce::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    upd_GeometryPath().setDefaultColor(kDefaultLigamentColor);
}

void DynamicLigamentForce::extendConnectToModel(Model& aModel) {
    upd_GeometryPath().setDefaultColor(kDefaultLigamentColor);
    Super::extendConnectToModel(aModel);
}

void DynamicLigamentForce::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
}

void DynamicLigamentForce::extendPostScale(const SimTK::State& s, const ScaleSet& aScaleSet) {
    Super::extendPostScale(s, aScaleSet);
    const GeometryPath& path = getGeometryPath();
    if (path.getPreScaleLength(s) > 0.0) {
        const double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        upd_resting_length() *= scaleFactor;
        upd_cross_sectional_area() *= scaleFactor;
    }
}

double DynamicLigamentForce::getLength(const SimTK::State& s) const {
    return getGeometryPath().getLength(s);
}

void DynamicLigamentForce::evalLigamentState(const SimTK::State& s,
                                             double& length,
                                             double& rate,
                                             double& strain,
                                             double& strainRate,
                                             double& force) const {
    const GeometryPath& path = getGeometryPath();
    const double restingLength = get_resting_length();
    length = path.getLength(s);
    rate = path.getLengtheningSpeed(s);

    if (restingLength <= 0.0) {
        strain = 0.0;
        strainRate = 0.0;
        force = 0.0;
        return;
    }

    strain = length / restingLength;
    strainRate = rate / restingLength;
    force = 0.0;

    if (strain <= 1.0) {
        return;
    }

    const double crossSectionalArea = get_cross_sectional_area();
    const double toeStrain = get_toe_strain();
    const double toeModulus = get_toe_modulus();
    const double linearModulus = get_linear_modulus();
    const double rateScale = get_rate_scale();
    const double rateIntercept = get_rate_intercept();

    double rateFactor = 1.0;
    if (rate > rateIntercept && rateIntercept > 0.0 && rateScale != 0.0) {
        rateFactor = std::max(0.0, 1.0 + std::log(rate / rateIntercept) * rateScale);
    }

    if (strain <= toeStrain) {
        force = (strain - 1.0) * toeModulus * crossSectionalArea * rateFactor * 1000000.0;
    } else {
        force = ((toeStrain - 1.0) * toeModulus + (strain - toeStrain) * linearModulus)
            * crossSectionalArea * rateFactor * 1000000.0;
    }
}

double DynamicLigamentForce::getTension(const SimTK::State& s) const {
    if (!appliesForce(s)) return 0.0;
    double length = 0.0, rate = 0.0, strain = 0.0, strainRate = 0.0, force = 0.0;
    evalLigamentState(s, length, rate, strain, strainRate, force);
    return force;
}

double DynamicLigamentForce::getStrain(const SimTK::State& s) const {
    const double restingLength = get_resting_length();
    if (restingLength <= 0.0) return 0.0;
    return getGeometryPath().getLength(s) / restingLength;
}

double DynamicLigamentForce::getStrainRate(const SimTK::State& s) const {
    const double restingLength = get_resting_length();
    if (restingLength <= 0.0) return 0.0;
    return getGeometryPath().getLengtheningSpeed(s) / restingLength;
}

double DynamicLigamentForce::getCurrentLength(const SimTK::State& s) const {
    return getGeometryPath().getLength(s);
}

double DynamicLigamentForce::getLengtheningRate(const SimTK::State& s) const {
    return getGeometryPath().getLengtheningSpeed(s);
}

double DynamicLigamentForce::computeMomentArm(const SimTK::State& s, const Coordinate& aCoord) const {
    return getGeometryPath().computeMomentArm(s, aCoord);
}

void DynamicLigamentForce::computeForce(const SimTK::State& s,
                                        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                                        SimTK::Vector& generalizedForces) const {
    if (!appliesForce(s)) return;

    double length = 0.0, rate = 0.0, strain = 0.0, strainRate = 0.0, force = 0.0;
    evalLigamentState(s, length, rate, strain, strainRate, force);
    if (force <= 1e-12) return;

    OpenSim::Array<PointForceDirection*> PFDs;
    getGeometryPath().getPointForceDirections(s, &PFDs);
    for (int i = 0; i < PFDs.getSize(); ++i) {
        applyForceToPoint(s, PFDs[i]->frame(), PFDs[i]->point(),
                          force * PFDs[i]->direction(), bodyForces);
        delete PFDs[i];
    }
}

double DynamicLigamentForce::computePotentialEnergy(const SimTK::State& s) const {
    if (!appliesForce(s)) return 0.0;
    const double restingLength = get_resting_length();
    if (restingLength <= 0.0) return 0.0;
    const double strain = getGeometryPath().getLength(s) / restingLength;
    if (strain <= 1.0) return 0.0;

    const double c = restingLength * get_cross_sectional_area() * 1000000.0;
    const double toeStrain = get_toe_strain();
    const double toeModulus = get_toe_modulus();
    const double linearModulus = get_linear_modulus();

    if (strain <= toeStrain) {
        const double dl = strain - 1.0;
        return 0.5 * c * toeModulus * dl * dl;
    }
    const double dt = toeStrain - 1.0;
    const double dl = strain - toeStrain;
    return c * (0.5 * toeModulus * dt * dt + toeModulus * dt * dl + 0.5 * linearModulus * dl * dl);
}
