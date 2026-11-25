# -*- coding: utf-8 -*-
"""
Created on Tue Jan 23 17:46:30 2018

@author: Calvin

Script for scaling OpenSim models with the new ligament.
For some reason, the scale tool does not work.

Scale Masses - Scales body masses evenly
Scale Strength - Scales muscle max isometric forces evenly

"""
import ctypes
import numpy as np
import os
import opensim as osim

"""
For opensim
"""
def test_check_env_var():
    if 'OPENSIM_HOME' not in os.environ:
        raise Exception("To run tests, must set environment "
                "variable OPENSIM_HOME "
                "to an OpenSim installation.")

"""
Main - Change things in here to do different types of scaling etc.
"""
def main():
    
    massScale = 1
    strengthScale = 1
    sizeScale = 1
    stiffScale = 1
    
    # Load DynamicLigament Plugin
    print("Loading Plugin")
    ctypes.CDLL("C:/Users/Calvin/Documents/Stanford/CamLab/Head Neck Modeling/Ligament Model/build/Release/osimDynamicLigamentPlugin.dll")
    
    #####################
    # Load Full Model - NOTE CHANGE THIS PATH TO YOUR OPENSIM FILE.
    #####################
    print("Creating Model")
    #ligModel = osim.Model("C:/Users/Calvin/Documents/Stanford/CamLab/Head Neck Modeling/Cervical Spine Full/fulldof_groundtruth_new_Millard.osim")
    ligModel = osim.Model("C:/Users/Calvin/Documents/Stanford/CamLab/Head Neck Modeling/Cervical Spine Full/fulldof_HYOID_wrap.osim")

    # Initialize the model, if values need to be set in the model's state
    # pass along the variable ligState returned by initSystem
    print("Initializing Model")
    ligState = ligModel.initSystem()
    
    # Building model copy
    print("Building Copy")
    copyModel = osim.Model(ligModel)
    
    # Scale mass
    print("Scaling Mass")
    copyModel = scaleMass(ligModel, copyModel, massScale)
    
    # Scale Strength
    print("Scaling Strength")
    copyModel = scaleStrength(ligModel, copyModel, strengthScale)
    
    # Scale Size
    print("Scaling Size")
    copyModel = scaleSize(ligModel, copyModel, sizeScale)
    
    # Scale Size
    print("Scaling Stiffness")
    copyModel = scaleStiffness(ligModel, copyModel, stiffScale)
    
    # Get full path name of original.old model
    fullPathName = ligModel.getInputFileName()

    # Change the name of the modified model
    newName = fullPathName.replace('.osim', '_edited.osim')

    # Save copy
    print("Saving Copy")
    copyModel.printToXML(newName)
    
    return 0

"""
Function to scale mass of bodies
- Scales mass by input value
"""
def scaleMass(origModel, newModel, massScale):
    bodySet = newModel.getBodySet()
    numBodies = bodySet.getSize()
    
    # Scale body masses except ground
    for i in (range(numBodies-1)):
        bd = bodySet.get(i+1)
        bd.scaleMass(massScale)
        
    return newModel

"""
Function to scale max isometric strength of muscles
- Scales strength by input value
"""
def scaleStrength(origModel, newModel, strengthScale):

    # Apply scale factor to MaxIsometricForce
    for i in range(newModel.getMuscles().getSize()):
        msc = newModel.getMuscles().get(i)
        msc.setMaxIsometricForce(msc.getMaxIsometricForce()*strengthScale)

    return newModel

"""
Function to scale size
- Scales size uniformly by input value.
- Note, in the future, may do individual x,y,z scaling, but that would make geometry path stuff a pain
"""
def scaleSize(origModel, newModel, sizeScale):
    
    # State
    st = newModel.initSystem()
    
    # Scale Factors Vector
    scaleV3 = osim.Vec3(sizeScale)
    
    # Scale Bodies
    bodySet = newModel.getBodySet()
    numBodies = bodySet.getSize()
    
    # Scale set for everyone else
    sclSet = osim.ScaleSet()
    for i in range(numBodies):
        bd = bodySet.get(i)
        
        # Make ScaleSet for other scaling things...
        bdScale = osim.Scale()
        bdScale.setSegmentName(bd.getName())
        bdScale.setScaleFactors(scaleV3)
        bdScale.setApply(True)
        sclSet.adoptAndAppend(bdScale)
    
    # Prescale and scale forces
    frcSet = newModel.getForceSet()
    numForces = frcSet.getSize()
    
    for i in range(numForces):
        frc = frcSet.get(i)
        
        # Scale Muscles
        if ( frc.getConcreteClassName() == 'Millard2012EquilibriumMuscle' ):
            msc = osim.Millard2012EquilibriumMuscle_safeDownCast(frc)
            msc.preScale(st, sclSet)
            msc.scale(st, sclSet)
            
        # Scale Ligaments
        else:
            if ( frc.getConcreteClassName() == 'DynamicLigamentForce' ):
                geometryPath = osim.GeometryPath.safeDownCast( frc.updPropertyByName("GeometryPath").getValueAsObject() )
                geometryPath.preScale(st, sclSet)
                geometryPath.scale(st, sclSet)
                
            # Scale other forces
            else:
                if ( frc.getConcreteClassName() == 'PathSpring' ):
                    pspring = osim.PathSpring.safeDownCast(frc)
                    pspring.preScale(st, sclSet)
                    pspring.scale(st, sclSet)
    
    # Scale body Sizes except ground
    for i in range(numBodies-1):
        bd = bodySet.get(i+1)
        bd.scale(scaleV3, False) # DO NOT UPDATE MASS SCALE - THAT IS DONE ELSEWHERE
    
    # Scale Joints
    jointSet = newModel.getJointSet()
    numJoints = jointSet.getSize()
    
    # Scale joint locations
    for i in range(numJoints):
        jnt = jointSet.get(i)
        jnt.scale(sclSet)
        
    st = newModel.initSystem()
    
    # Postscale forces
    for i in range(numForces):
        frc = frcSet.get(i)
        
        # Scale Muscles
        if ( frc.getConcreteClassName() == 'Millard2012EquilibriumMuscle' ):
            msc = osim.Millard2012EquilibriumMuscle_safeDownCast(frc)
            msc.postScale(st, sclSet)
            
        # Scale Ligaments
        else:
            if ( frc.getConcreteClassName() == 'DynamicLigamentForce' ):
                geometryPath = osim.GeometryPath.safeDownCast( frc.updPropertyByName("GeometryPath").getValueAsObject() )
                geometryPath.postScale(st, sclSet)
                if (geometryPath.getPreScaleLength(st) > 0):
                    scaleFactor = geometryPath.getLength(st) / geometryPath.getPreScaleLength(st);
                    rlengthProp = frc.updPropertyByName("resting_length")
                    osim.PropertyHelper_setValueDouble( osim.PropertyHelper_getValueDouble( rlengthProp ) * scaleFactor, rlengthProp )
                    csaProp = frc.updPropertyByName("cross_sectional_area")
                    osim.PropertyHelper_setValueDouble( osim.PropertyHelper_getValueDouble( csaProp ) * scaleFactor, csaProp )
    
            # Scale other forces
            else:
                if ( frc.getConcreteClassName() == 'PathSpring' ):
                    pspring = osim.PathSpring.safeDownCast(frc)
                    pspring.postScale(st, sclSet)
    
    return newModel

"""
Function to scale ligament stiffnesses
- Scales ligament young's moduli by input value
"""
def scaleStiffness(origModel, newModel, stiffScale):
    
    # State
    st = newModel.initSystem()
    
    # Ligaments to scale
    frcSet = newModel.getForceSet()
    numForces = frcSet.getSize()
    
    for i in range(numForces):
        frc = frcSet.get(i)

        # Scale Ligaments
        if ( frc.getConcreteClassName() == 'DynamicLigamentForce' ):
            lmodProp = frc.updPropertyByName("linear_modulus")
            osim.PropertyHelper_setValueDouble( osim.PropertyHelper_getValueDouble( lmodProp ) * stiffScale, lmodProp )
            tmodProp = frc.updPropertyByName("toe_modulus")
            osim.PropertyHelper_setValueDouble( osim.PropertyHelper_getValueDouble( tmodProp ) * stiffScale, tmodProp )

    return newModel

if __name__ == "__main__":
    print("Starting")
    test_check_env_var()
    main()