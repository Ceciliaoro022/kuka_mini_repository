#!/usr/bin/env python3

"""
Simple test script to check if Pinocchio can be initialized properly.
Run this script directly to test if Pinocchio works on your system.
"""

import sys
import os
import numpy as np

try:
    print("Importing pinocchio...")
    import pinocchio as pin
    print(f"Pinocchio imported successfully. Version: {pin.__version__}")
except ImportError as e:
    print(f"Failed to import pinocchio: {e}")
    sys.exit(1)

# Try to create a simple model
try:
    print("\nCreating a simple 2R robot model...")
    model = pin.Model()
    joint1_id = model.addJoint(0, pin.JointModelRY(), pin.SE3.Identity(), "joint1")
    joint2_id = model.addJoint(joint1_id, pin.JointModelRY(), pin.SE3(np.eye(3), np.array([1.0, 0, 0])), "joint2")
    
    # Add some inertia
    model.appendBodyToJoint(joint1_id, pin.Inertia.Random(), pin.SE3.Identity())
    model.appendBodyToJoint(joint2_id, pin.Inertia.Random(), pin.SE3.Identity())
    
    # Create data object
    data = pin.Data(model)
    
    print(f"Model created successfully with {model.nq} position DoFs and {model.nv} velocity DoFs")
    print(f"Joint names: {[model.names[i] for i in range(1, model.njoints)]}")
    
    # Try to compute gravity
    q = np.zeros(model.nq)
    print("\nComputing gravity at zero configuration...")
    pin.computeGeneralizedGravity(model, data, q)
    gravity_torques = data.g.copy()
    print(f"Gravity torques: {gravity_torques}")
    
    print("\nPinocchio test successful!")
except Exception as e:
    print(f"\nError creating or using Pinocchio model: {e}")
    sys.exit(1)

# If URDF path is provided, try loading it
if len(sys.argv) > 1:
    urdf_path = sys.argv[1]
    try:
        print(f"\nTrying to load URDF from {urdf_path}...")
        if os.path.exists(urdf_path):
            urdf_model = pin.buildModelFromUrdf(urdf_path)
            urdf_data = pin.Data(urdf_model)
            print(f"URDF model loaded successfully with {urdf_model.nq} position DoFs and {urdf_model.nv} velocity DoFs")
            print(f"Joint names: {[urdf_model.names[i] for i in range(1, urdf_model.njoints)]}")
            
            # Try to compute gravity
            q = np.zeros(urdf_model.nq)
            print("\nComputing gravity at zero configuration for URDF model...")
            pin.computeGeneralizedGravity(urdf_model, urdf_data, q)
            gravity_torques = urdf_data.g.copy()
            print(f"Gravity torques: {gravity_torques}")
        else:
            print(f"URDF file not found: {urdf_path}")
    except Exception as e:
        print(f"Error loading or using URDF model: {e}")