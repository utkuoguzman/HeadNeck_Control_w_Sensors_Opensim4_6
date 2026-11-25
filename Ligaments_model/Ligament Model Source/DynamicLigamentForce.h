#ifndef __DynamicLigamentForce_h__
#define __DynamicLigamentForce_h__
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  DynamicLigamentForce.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// INCLUDE
#include <string>
#include "osimPluginDLL.h"
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>

namespace OpenSim {

	class GeometryPath;

//=================================================================================
//=================================================================================
/**
 * A class implementing a Dynamic Ligament.
 * Implements a dynamic ligament. The ligament force-length is a piece-wise
 * linear relationship with a characteristic toe region and linear-elastic region.
 * Failure is not represented in this model. Ligaments are affected by dynamic
 * loading through a scaling of the force-length curve based on the log of the
 * loading rate.
 *
 * Parameters:
 * 1) Resting Length - Resting length for the ligament (m). Below this length,
 *      no forces will be applied. Above this length, the ligament will exhibit
 *      a bilinear force-length response as defined by later parameters
 * 2) Cross Sectional Area - Cross sectional area for the ligament (m^2). Used to
 *      convert the ligament modulus in MPa to the forces in N
 * 3) Toe strain - The strain below which we are in the toe region for the ligament (%).
 *      Ligaments exhibit bilinear force-length response with a low stiffness toe region
 *      and a high stiffness linear region. Must be >1
 * 4) Toe modulus - The modulus of the toe region (MPa). Must be >=0
 * 5) Linear modulus - The modulus of the linear region (MPa). Must be >=0
 *
 * @author Calvin Kuo
 * @version 1.0
 */
class OSIMPLUGIN_API DynamicLigamentForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(DynamicLigamentForce, Force);

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Geometry path for the ligament, allows for more flexible ligament insertion and routing */
	OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath,
		"Ligament geometry path");

	/** Resting length for ligament **/
	OpenSim_DECLARE_PROPERTY(resting_length, double,
		"Resting length of the ligament (m)");

	/** Cross Sectional Area allows for scaling the modulus **/
	OpenSim_DECLARE_PROPERTY(cross_sectional_area, double,
		"Cross Sectional Area allows for scaling the modulus (m^2)");

	/** Toe Strain defines the transition from toe to linear **/
	OpenSim_DECLARE_PROPERTY(toe_strain, double,
		"Toe Strain defines the length transition from toe to linear regions (%). Must be >=1");

	/** Toe Modulus defines the modulus of the toe region **/
	OpenSim_DECLARE_PROPERTY(toe_modulus, double,
		"Toe Modulus defines the modulus of the toe region (MPa). Must be >=0");

	/** Linear Modulus defines the modulus of the linear region **/
	OpenSim_DECLARE_PROPERTY(linear_modulus, double,
		"Linear Modulus defines the modulus of the linear region (MPa). Must be >=0");

	/** Rate depenent log scaling of the force length curve **/
	OpenSim_DECLARE_PROPERTY(rate_scale, double,
		"Rate dependent log scaling of the force-length curve")

	/** Rate intercept for log scaling **/
	OpenSim_DECLARE_PROPERTY(rate_intercept, double,
		"Rate intercept for log scaling. This is the rate at which rate scaling takes effect.")

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION - Use default (compiler-generated) destructor, copy constructor, and copy
	// assignment operator
	DynamicLigamentForce();

	//SET
	virtual bool setRestingLength(double aRestingLength);
	virtual bool setCrossSectionalArea(double aCrossSectionalArea);
	virtual bool setToeStrain(double aToeStrain);
	virtual bool setToeModulus(double aToeModulus);
	virtual bool setLinearModulus(double aLinearModulus);
	virtual bool setRateScale(double aRateScale);
	virtual bool setRateIntercept(double aRateIntercept);

	//GET
	const GeometryPath& getGeometryPath() const
	{ return get_GeometryPath(); }
	GeometryPath& updGeometryPath()
	{ return upd_GeometryPath(); }
	virtual bool hasGeomeryPath() const { return true; };
	virtual double getRestingLength() const
	{ return get_resting_length(); }
	virtual double getCrossSectionalArea() const
	{ return get_cross_sectional_area(); }
	virtual const double getToeStrain() const
	{ return get_toe_strain(); }
	virtual const double getToeModulus() const
	{ return get_toe_modulus(); }
	virtual const double getLinearModulus() const
	{ return get_linear_modulus(); }
	virtual double getRateScale() const
	{ return get_rate_scale(); }
	virtual double getRateIntercept() const
	{ return get_rate_intercept(); }
	virtual double getLength(const SimTK::State& s) const;

	// computed variables
	const double& getTension(const SimTK::State& s) const;
	const double& getStrain(const SimTK::State& s) const;
	const double& getStrainRate(const SimTK::State& s) const;
	const double& getCurrentLength(const SimTK::State& s) const;
	const double& getLengtheningRate(const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	
	// Moment Arm
	virtual double computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const;
	
	// Compute the dynamic ligament force
	virtual void computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

	//--------------------------------------------------------------------------
	// SCALE
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);

	//--------------------------------------------------------------------------
	// Display
	//--------------------------------------------------------------------------
	virtual const VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s) const;

protected:
	// Implement ModelComponent Interface
	void connectToModel(Model& aModel) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	void realizeDynamics(const SimTK::State& state) const OVERRIDE_11;

	// Force Reporting
	OpenSim::Array<std::string> getRecordLabels() const {
		OpenSim::Array<std::string> labels("");
		labels.append(getName() + ".force");
		labels.append(getName() + ".strain");
		labels.append(getName() + ".length");
		labels.append(getName() + ".strainRate");
		labels.append(getName() + ".lengtheningRate");
		return labels;
	}
	OpenSim::Array<double> getRecordValues(const SimTK::State& state) const {
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

//=============================================================================
};	// END of class DynamicLigamentForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __DynamicLigamentForce_h__


