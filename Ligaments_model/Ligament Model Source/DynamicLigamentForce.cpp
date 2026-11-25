/* -------------------------------------------------------------------------- *
 *                     OpenSim:  DynamicLigamentForce.cpp                     *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/PointForceDirection.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>

#include "DynamicLigamentForce.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

static const Vec3 DefaultLigamentColor(.9, .9, .1); // mostly yellow

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
DynamicLigamentForce::DynamicLigamentForce()
{
	constructProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void DynamicLigamentForce::constructProperties()
{
	// Default parameters
	setAuthors("Calvin Kuo");
	constructProperty_GeometryPath(GeometryPath());
	constructProperty_resting_length(0.0);
	constructProperty_cross_sectional_area(0.0);
	constructProperty_toe_strain(1.0);
	constructProperty_toe_modulus(0.0);
	constructProperty_linear_modulus(0.0);
	constructProperty_rate_scale(0.0);
	constructProperty_rate_intercept(0.0);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this DynamicLigamentForce.
 */
void DynamicLigamentForce::connectToModel(Model& aModel)
{
	GeometryPath& path = upd_GeometryPath();
	const double& restingLength = get_resting_length();
	const double& rateScale = get_rate_scale();

	// Set ligament color
	path.setDefaultColor(DefaultLigamentColor);

	// Specify underlying ModelComponents
	includeAsSubComponent(&path);

	// Propagate up
	Super::connectToModel(aModel);

	// _model will be NULL when objects are being registered
	if (_model == NULL)
		return;

	// Assert various parameters are >0.0
	assert(restingLength > 0.0);
	assert(rateScale > 0.0);

	path.setOwner(this);
}

void DynamicLigamentForce::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);
	// Cache the computed tension, strain, and strain rate of the ligament
	addCacheVariable<double>("tension", 0.0, Stage::Velocity);
	addCacheVariable<double>("strain", 0.0, Stage::Velocity);
	addCacheVariable<double>("length", 0.0, Stage::Velocity);
	addCacheVariable<double>("strain_rate", 0.0, Stage::Velocity);
	addCacheVariable<double>("lengthening_rate", 0.0, Stage::Velocity);
}

void DynamicLigamentForce::realizeDynamics(const SimTK::State& state) const
{
	Super::realizeDynamics(state); // Mandatory first line
}

//=============================================================================
// GET
//=============================================================================
//_____________________________________________________________________________

// Get length of ligament
double DynamicLigamentForce::getLength(const SimTK::State& s) const
{
	return getGeometryPath().getLength(s);
}

// Get tension cached variable
const double& DynamicLigamentForce::getTension(const SimTK::State& s) const
{
	return getCacheVariable<double>(s, "tension");
}

// Get strain cached variable
const double& DynamicLigamentForce::getStrain(const SimTK::State& s) const
{
	return getCacheVariable<double>(s, "strain");
}

// Get strain rate cached variable
const double& DynamicLigamentForce::getStrainRate(const SimTK::State& s) const
{
	return getCacheVariable<double>(s, "strain_rate");
}

// Get length cached variable
const double& DynamicLigamentForce::getCurrentLength(const SimTK::State& s) const
{
	return getCacheVariable<double>(s, "length");
}

// Get lengthening rate cached vairable
const double& DynamicLigamentForce::getLengtheningRate(const SimTK::State& s) const
{
	return getCacheVariable<double>(s, "lengthening_rate");
}
//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________

// Set resting length
bool DynamicLigamentForce::setRestingLength(double aRestingLength)
{
	set_resting_length(aRestingLength);
	return true;
}

// Set Cross Sectional Area
bool DynamicLigamentForce::setCrossSectionalArea(double aCrossSectionalArea)
{
	set_cross_sectional_area(aCrossSectionalArea);
	return true;
}

// Set Toe Strain
bool DynamicLigamentForce::setToeStrain(double aToeStrain)
{
	set_toe_strain(aToeStrain);
	return true;
}

// Set Toe Modulus
bool DynamicLigamentForce::setToeModulus(double aToeModulus)
{
	set_toe_modulus(aToeModulus);
	return true;
}

// Set Linear Modulus
bool DynamicLigamentForce::setLinearModulus(double aLinearModulus)
{
	set_linear_modulus(aLinearModulus);
	return true;
}

// Set Rate Scaling
bool DynamicLigamentForce::setRateScale(double aRateScale)
{
	set_rate_scale(aRateScale);
	return true;
}

// Set Rate Intercept
bool DynamicLigamentForce::setRateIntercept(double aRateIntercept)
{
	set_rate_intercept(aRateIntercept);
	return true;
}

//=============================================================================
// SCALE
//=============================================================================
// Perform computation that need to happen before ligament is scaled
void DynamicLigamentForce::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	updGeometryPath().preScale(s, aScaleSet);
}

// Scale the ligament
void DynamicLigamentForce::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	updGeometryPath().scale(s, aScaleSet);
}

// Perform computations that need to happen after the ligament is scaled
void DynamicLigamentForce::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	GeometryPath& path = updGeometryPath();

	path.postScale(s, aScaleSet);
	if (path.getPreScaleLength(s) > 0.0)
	{
		double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
		
		// Scale Resting Length by the same amount as change in total
		// ligament length
		upd_resting_length() *= scaleFactor;
		upd_cross_sectional_area() *= scaleFactor;
		path.setPreScaleLength(s, 0.0);
	}
}

//=============================================================================
// COMPUTATION
//=============================================================================

// Compute Moment Arm for the ligament
double DynamicLigamentForce::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
	return getGeometryPath().computeMomentArm(s, aCoord);
}

// Compute Force Provided by dynamic ligament
void DynamicLigamentForce::computeForce(const SimTK::State& s,
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	const GeometryPath& path = getGeometryPath();
	const double& restingLength = get_resting_length();
	const double& crossSectionalArea = get_cross_sectional_area();
	const double& toeStrain = get_toe_strain();
	const double& toeModulus = get_toe_modulus();
	const double& linearModulus = get_linear_modulus();
	const double& rate = path.getLengtheningSpeed(s);
	const double& rateScale = get_rate_scale();
	const double& rateIntercept = get_rate_intercept();

	double force = 0;

	// Ligament is slack, provides no force
	double length = path.getLength(s);
	double strain = length / restingLength;
	double strain_rate = rate / restingLength;

	if (strain <= 1.0) {
		setCacheVariable<double>(s, "tension", force);
		setCacheVariable<double>(s, "strain", strain);
		setCacheVariable<double>(s, "length", length);
		setCacheVariable<double>(s, "lengthening_rate", rate);
		setCacheVariable<double>(s, "strain_rate", strain_rate);
		return;
	}

	// Rate scaling should not be negative. Also, since we are using the log(rate),
	// scaling is only computed when rate is >0.0
	double rateFactor = 1.0;
	if (rate > rateIntercept) {
		rateFactor = 1.0 + log(rate/rateIntercept) * rateScale;
	}

	// Evaluate normalized ligament force-length curve
	// Scale by failure force and log elongation rate
	if (strain <= toeStrain)
	{
		// In toe region
		force = (strain-1.0) * toeModulus * crossSectionalArea * rateFactor * 1000000;
	}
	else
	{
		// In linear region
		force = ((toeStrain - 1.0) * toeModulus + (strain - toeStrain) * linearModulus) * crossSectionalArea * rateFactor * 1000000;
	}
	setCacheVariable<double>(s, "tension", force);
	setCacheVariable<double>(s, "strain", strain);
	setCacheVariable<double>(s, "length", length);
	setCacheVariable<double>(s, "strain_rate", strain_rate);
	setCacheVariable<double>(s, "lengthening_rate", rate);

	OpenSim::Array<PointForceDirection*> PFDs;
	path.getPointForceDirections(s, &PFDs);

	for (int i = 0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->body(), PFDs[i]->point(),
			force * PFDs[i]->direction(), bodyForces);
	}
	for (int i = 0; i < PFDs.getSize(); i++) {
		delete PFDs[i];
	}

	return;
}

//=============================================================================
// DISPLAYER
//=============================================================================
const VisibleObject* DynamicLigamentForce::getDisplayer() const
{
	return getGeometryPath().getDisplayer();
}

void DynamicLigamentForce::updateDisplayer(const SimTK::State& s) const
{
	getGeometryPath().updateDisplayer(s);
}