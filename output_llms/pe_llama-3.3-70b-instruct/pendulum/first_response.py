import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Initialize the PyChrono environment
chrono.SetChronoDataPath('/path/to/data')

# Create the simulation system
sys = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
sys.Add(ground)

# Create the pendulum body
pendulum = chrono.ChBodyEasySphere(sys, 1, 1000)
pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(pendulum)

# Define the pendulum's mass and inertia
pendulum.SetMass(10)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))

# Create a revolute joint between the ground and the pendulum
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, -5, 0), chrono.Q_from_AngX(0)))
sys.Add(rev_joint)

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()

# Simulation loop
time_step = 0.01
while vis.Run():
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    # Log the pendulum's position and velocity
    if sys.GetChTime() % 1 < time_step:
        print(f"Time: {sys.GetChTime()}, Position: {pendulum.GetPos()}, Velocity: {pendulum.GetPos_dt()}")