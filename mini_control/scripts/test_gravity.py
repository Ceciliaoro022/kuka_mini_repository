#!/usr/bin/env python3

import numpy as np
import pinocchio as pin

# Print Pinocchio version
print(f"Using Pinocchio version: {pin.__version__}")

# Create a simple 2R robot model programmatically
model = pin.Model()
joint1_id = model.addJoint(0, pin.JointModelRY(), pin.SE3.Identity(), "joint1")
joint2_id = model.addJoint(joint1_id, pin.JointModelRY(), pin.SE3(np.eye(3), np.array([0, 0, 2.0])), "joint2")

# Add inertia for more realistic dynamics
mass1 = 1.0  # kg
mass2 = 0.5  # kg

# Create simple inertia parameters (based on a box)
length1 = 2.0  # meters
length2 = 1.0  # meters
width = 0.1  # meters

# Create inertia objects directly without using FromMass
# For a rectangular box with width×width×length dimensions
I1xx = I1yy = mass1 * (length1*length1 + width*width) / 12
I1zz = mass1 * (width*width + width*width) / 12
inertia1 = pin.Inertia(mass1, np.array([0, 0, length1/2]), np.diag([I1xx, I1yy, I1zz]))

I2xx = I2yy = mass2 * (length2*length2 + width*width) / 12
I2zz = mass2 * (width*width + width*width) / 12
inertia2 = pin.Inertia(mass2, np.array([0, 0, length2/2]), np.diag([I2xx, I2yy, I2zz]))

model.appendBodyToJoint(joint1_id, inertia1, pin.SE3.Identity())
model.appendBodyToJoint(joint2_id, inertia2, pin.SE3.Identity())

# Create data object
data = pin.Data(model)

# Compute gravity at different configurations
q = np.zeros(model.nq)
pin.computeGeneralizedGravity(model, data, q)
print(f"Gravity at [0, 0]: {data.g}")

q = np.array([np.pi/4, np.pi/4])
pin.computeGeneralizedGravity(model, data, q)
print(f"Gravity at [pi/4, pi/4]: {data.g}")

q = np.array([np.pi/2, 0])
pin.computeGeneralizedGravity(model, data, q)
print(f"Gravity at [pi/2, 0]: {data.g}")