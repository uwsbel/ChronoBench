import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, irrlicht, vehicle

# Initialize the system
sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath('path/to/data')  # Replace with your data path

# Visualization setup
vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear System')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  # Position camera

# Fixed truss
truss_body = chrono.ChBody()
truss_body.SetPos(chrono.ChVector3d(0, 0, 0))
truss_body.SetMass(100)
truss_body.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  # Inertia for a point mass
truss_body.SetFixed(True)
sys.Add(truss_body)

# Material for truss
truss_material = chrono.ChContactMaterialNSC()
truss_material.SetFriction(0.5)
truss_material.SetDampingF(0.1)
truss_body.SetMaterial(truss_material)

# Visual setup for truss
truss_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(2, 2, 0.2))
truss_shape.SetColor(chrono.ChColor(1, 1, 0))  # Yellow
truss_body.AddVisualShape(truss_shape)

# Rotating bar
rotating_bar = chrono.ChBody()
rotating_bar.SetPos(chrono.ChVector3d(5, 0, 0))
rotating_bar.SetMass(10)
rotating_bar.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  # Point mass
rotating_bar.EnableCollision(True)
sys.Add(rotating_bar)

# Material for rotating bar
bar_material = chrono.ChContactMaterialNSC()
bar_material.SetFriction(0.3)
rotating_bar.SetMaterial(bar_material)

# Visual setup for rotating bar
bar_shape = chrono.ChVisualShapeCylinder(0.5, 1.0)
bar_shape.SetColor(chrono.ChColor(1, 0, 0))  # Red
rotating_bar.AddVisualShape(bar_shape)

# Upper gear
upper_gear = chrono.ChBodyEasyCircle(0.5, 1000, True, True, bar_material)
upper_gear.SetPos(chrono.ChVector3d(0, 5, 0))
sys.Add(upper_gear)

# Lower gear
lower_gear = chrono.ChBodyEasyCircle(0.5, 1000, True, True, bar_material)
lower_gear.SetPos(chrono.ChVector3d(0, -5, 0))
sys.Add(lower_gear)

# Motor setup
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rotating_bar, upper_gear, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # 90 degree offset
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # Constant speed
sys.Add(motor)

# Spherical joint between upper and lower gear
spher_joint = chrono.ChLinkLockSpherical()
spher_joint.Initialize(upper_gear, lower_gear, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # 90 degree offset
sys.Add(spher_joint)

# Visualization materials for gears
gear_color = chrono.ChColor(0.8, 0.8, 0.8)  # Light gray
gear_material = chrono.ChContactMaterialNSC()
gear_material.SetFriction(0.2)
upper_gear.SetMaterial(gear_material)
lower_gear.SetMaterial(gear_material)

# Main simulation loop
while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Close the visualization window when the loop exits
vis.DestroyWindow()