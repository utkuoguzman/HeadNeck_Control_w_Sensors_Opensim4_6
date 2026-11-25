#ifndef OPENSIM_DYNAMIC_LIGAMENT_FORCE_H_
#define OPENSIM_DYNAMIC_LIGAMENT_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  DynamicLigamentForce.h                      *
 * -------------------------------------------------------------------------- *
 * Ported to OpenSim 4.6 from Calvin Kuo et al. (2019),                       *
 * "Passive cervical spine ligaments provide stability during head impacts",  *
 * J. R. Soc. Interface 16: 20190086.                                         *
 * Original OpenSim 3.3 DynamicLigamentPlugin Author: Calvin Kuo (2018).      *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PointForceDirection.h>
#include <SimTKcommon/internal/State.h>

namespace OpenSim {

/**
 * Rate-dependent cervical ligament force (Kuo et al., 2019) ported to OpenSim 4.6.
 * Implements a bilinear force-stretch curve (toe region + linear elastic region,
 * slack for L <= L_0) scaled multiplicatively by a logarithmic lengthening-rate factor
 * above `rate_intercept`:
 *   lambda = L / L_0
 *   rateFactor = (rate > rate_intercept) ? 1 + rate_scale * ln(rate / rate_intercept) : 1
 *   F = rateFactor * A * 1e6 * bilinear_stress(lambda)
 */
class DynamicLigamentForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(DynamicLigamentForce, Force);

public:
    OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath,
        "Ligament geometry path");

    OpenSim_DECLARE_PROPERTY(resting_length, double,
        "Resting length of the ligament (m)");

    OpenSim_DECLARE_PROPERTY(cross_sectional_area, double,
        "Cross Sectional Area allows for scaling the modulus (m^2)");

    OpenSim_DECLARE_PROPERTY(toe_strain, double,
        "Toe Strain defines the stretch ratio transition from toe to linear regions (L/L0). Must be >= 1");

    OpenSim_DECLARE_PROPERTY(toe_modulus, double,
        "Toe Modulus defines the modulus of the toe region (MPa). Must be >= 0");

    OpenSim_DECLARE_PROPERTY(linear_modulus, double,
        "Linear Modulus defines the modulus of the linear region (MPa). Must be >= 0");

    OpenSim_DECLARE_PROPERTY(rate_scale, double,
        "Rate dependent log scaling of the force-length curve");

    OpenSim_DECLARE_PROPERTY(rate_intercept, double,
        "Rate intercept (m/s) for log scaling at which rate scaling takes effect");

    OpenSim_DECLARE_OUTPUT(tension, double, getTension, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(strain, double, getStrain, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(strain_rate, double, getStrainRate, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(length, double, getCurrentLength, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(lengthening_rate, double, getLengtheningRate, SimTK::Stage::Velocity);

    DynamicLigamentForce();

    void setRestingLength(double aRestingLength) { set_resting_length(aRestingLength); }
    void setCrossSectionalArea(double aCrossSectionalArea) { set_cross_sectional_area(aCrossSectionalArea); }
    void setToeStrain(double aToeStrain) { set_toe_strain(aToeStrain); }
    void setToeModulus(double aToeModulus) { set_toe_modulus(aToeModulus); }
    void setLinearModulus(double aLinearModulus) { set_linear_modulus(aLinearModulus); }
    void setRateScale(double aRateScale) { set_rate_scale(aRateScale); }
    void setRateIntercept(double aRateIntercept) { set_rate_intercept(aRateIntercept); }

    const GeometryPath& getGeometryPath() const { return get_GeometryPath(); }
    GeometryPath& updGeometryPath() { return upd_GeometryPath(); }
    double getRestingLength() const { return get_resting_length(); }
    double getCrossSectionalArea() const { return get_cross_sectional_area(); }
    double getToeStrain() const { return get_toe_strain(); }
    double getToeModulus() const { return get_toe_modulus(); }
    double getLinearModulus() const { return get_linear_modulus(); }
    double getRateScale() const { return get_rate_scale(); }
    double getRateIntercept() const { return get_rate_intercept(); }
    double getLength(const SimTK::State& s) const;

    double getTension(const SimTK::State& s) const;
    double getStrain(const SimTK::State& s) const;
    double getStrainRate(const SimTK::State& s) const;
    double getCurrentLength(const SimTK::State& s) const;
    double getLengtheningRate(const SimTK::State& s) const;

    double computeMomentArm(const SimTK::State& s, const Coordinate& aCoord) const;

    void computeForce(const SimTK::State& s,
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                      SimTK::Vector& generalizedForces) const override;

    double computePotentialEnergy(const SimTK::State& s) const override;

    void extendPostScale(const SimTK::State& s, const ScaleSet& aScaleSet) override;

protected:
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& aModel) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    OpenSim::Array<std::string> getRecordLabels() const override {
        OpenSim::Array<std::string> labels("");
        labels.append(getName() + ".force");
        labels.append(getName() + ".strain");
        labels.append(getName() + ".length");
        labels.append(getName() + ".strainRate");
        labels.append(getName() + ".lengtheningRate");
        return labels;
    }

    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override {
        OpenSim::Array<double> values(1);
        values.append(getTension(state));
        values.append(getStrain(state));
        values.append(getCurrentLength(state));
        values.append(getStrainRate(state));
        values.append(getLengtheningRate(state));
        return values;
    }

private:
    void constructProperties();
    void evalLigamentState(const SimTK::State& s,
                           double& length,
                           double& rate,
                           double& strain,
                           double& strainRate,
                           double& force) const;
};

} // end of namespace OpenSim

#endif // OPENSIM_DYNAMIC_LIGAMENT_FORCE_H_
