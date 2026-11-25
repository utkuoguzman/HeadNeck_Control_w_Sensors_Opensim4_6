// INCLUDE
#include <OpenSim/OpenSim.h>
#include "Mileusnic06Spindle.h"
#include "Lin02GolgiTendonOrgan.h"

namespace OpenSim {
class Millard12EqMuscleWithAfferents : public Millard2012EquilibriumMuscle {
OpenSim_DECLARE_CONCRETE_OBJECT(Millard12EqMuscleWithAfferents, Millard2012EquilibriumMuscle);

friend class Mileusnic06Spindle;
friend class Lin02GolgiTendonOrgan;

public:
    OpenSim_DECLARE_PROPERTY(lpf_tau, double, "time constant for all the low-pass filters");
    OpenSim_DECLARE_PROPERTY(enable_eye_afferents, bool, "Whether to enable LPF/spindle ODE states on extraocular eye muscles (r_*, l_*) (default: false during neck study)");
    OpenSim_DECLARE_PROPERTY(enable_SRS, bool, "Whether to enable tendon-equilibrated cross-bridge short-range muscle stiffness (default: false)");
    OpenSim_DECLARE_PROPERTY(k_srs, double, "Normalized active short-range stiffness gain k_srs (Cui et al. 2008 default: 23.4)");
    OpenSim_DECLARE_PROPERTY(delta_srs, double, "Cross-bridge yield strain threshold in L_0 units (Rack & Westbury 1974 / Cui et al. 2008 default: 0.018)");
    OpenSim_DECLARE_PROPERTY(tau_srs_reset, double, "Slow cross-bridge turnover time constant in s (Kirsch et al. 1994 default: 0.400)");
    OpenSim_DECLARE_PROPERTY(tau_srs_short, double, "Fast cross-bridge detachment time constant during shortening in s (default: 0.025)");
    OpenSim_DECLARE_PROPERTY(srs_thix_min, double, "Steady continuous-movement thixotropy floor s_min (Kirsch et al. 1994 default: 0.35)");
    OpenSim_DECLARE_PROPERTY(tau_srs_thix_rec, double, "Thixotropy recovery time constant at rest in s (Campbell & Lakie 1998 default: 1.5)");
    OpenSim_DECLARE_PROPERTY(delta_srs_thix_break, double, "Cumulative normalized fiber excursion scale for thixotropic breakdown (De Groote et al. 2018 default: 0.05)");

    bool isAfferentActive() const {
        if (!active_checked) {
            const std::string& mname = getName();
            bool is_eye = (mname.rfind("r_", 0) == 0 || mname.rfind("l_", 0) == 0);
            is_active_cached = !is_eye || get_enable_eye_afferents();
            cached_lpf_tau = get_lpf_tau();
            active_checked = true;
        }
        return is_active_cached;
    }

    OpenSim_DECLARE_PROPERTY(spindle, Mileusnic06Spindle, "Spindle model");
    OpenSim_DECLARE_PROPERTY(GTO, Lin02GolgiTendonOrgan, "GTO model");

    Storage *afferents;

    Millard12EqMuscleWithAfferents();
    Millard12EqMuscleWithAfferents(const std::string &name, double maxIsometricForce,
                                   double optimalFiberLength, double tendonSlackLength,
                                   double pennationAngle);

    double getLPFvelocity(const SimTK::State& s) const;
    void   setLPFvelocity(SimTK::State& s, double Velocity) const;
    double getLPFacceleration(const SimTK::State& s) const;
    void   setLPFacceleration(SimTK::State& s, double Acceleration) const;
    double getSRSAnchor(const SimTK::State& s) const;
    void   setSRSAnchor(SimTK::State& s, double anchorLen) const;
    double getSRSThix(const SimTK::State& s) const;
    void   setSRSThix(SimTK::State& s, double thixVal) const;

    double getLPFtau() const { return get_lpf_tau(); }
    void setLPFtau(double aLPFtau);

    double getIaAfferent(const SimTK::State& s) const { return get_spindle().getIaOutput(s); }
    double getIIAfferent(const SimTK::State& s) const { return get_spindle().getIIOutput(s); }
    double getIbAfferent(const SimTK::State& s) const { return get_GTO().getGTOout(s); }

    double calcSRSTensionAlongTendon(const SimTK::State& s) const;
    double getTotalTendonForce(const SimTK::State& s) const;

    OpenSim_DECLARE_OUTPUT(primary_Ia, double, getIaAfferent, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(secondary_II, double, getIIAfferent, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(tendon_Ib, double, getIbAfferent, SimTK::Stage::Velocity);

    int numControls() const override { return 3; }

    const Mileusnic06Spindle* getSpindle() const { return &get_spindle(); }
    const Lin02GolgiTendonOrgan* getGTO() const { return &get_GTO(); }

protected:
    void calcFiberVelocityInfo(const SimTK::State& s,
                               FiberVelocityInfo& fvi) const override;
    void calcMuscleDynamicsInfo(const SimTK::State& s,
                                MuscleDynamicsInfo& mdi) const override;

    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& s) override;
    void extendConnectToModel(Model& aModel) override;

    void computeInitialFiberEquilibrium(SimTK::State& s) const override;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;
    double approxFiberAcceleration(const SimTK::State& s) const;

private:
    void constructProperties();
    mutable const StateVariable* sv_lpf_vel = nullptr;
    mutable const StateVariable* sv_lpf_acc = nullptr;
    mutable const StateVariable* sv_srs_anchor = nullptr;
    mutable const StateVariable* sv_srs_thix = nullptr;
    mutable const StateVariable* sv_activation = nullptr;
    mutable bool active_checked = false;
    mutable bool is_active_cached = false;
    mutable double cached_lpf_tau = 0.01;
    void ensureStatePointers() const;
};
}
