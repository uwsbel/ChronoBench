import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Set up the simulation
system = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBodyEasyBox(system,  # Parent system
                              10, 10, 1,  # Dimensions
                              1000,  # Density
                              True,  # Visualization
                              True)  # Collision
ground.SetBodyFixed(True)
system.Add(ground)

# Create the pendulum body
pendulum = chrono.ChBodyEasyCylinder(system,  # Parent system
                                     0.5,  # Radius
                                     2.0,  # Height
                                     1000,  # Density
                                     True,  # Visualization
                                     True)  # Collision
pendulum.SetPos(chrono.ChVectorD(0, 2, 0))
pendulum.SetMass(10)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(pendulum)

# Create a revolute joint between the ground and the pendulum
joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 1, 0)))
system.Add(joint)

# Set up the visualization
application = chronoirr.ChIrrApp(system, 'PyChrono Simple Pendulum', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(2, 2, 2), chronoirr.vector3df(0, 0, 0))

# Simulation loop
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    # Log the pendulum's position and velocity
    if system.GetChTime() % 0.1 < 0.01:
        print(f"Time: {system.GetChTime():.2f} s")
        print(f"Pendulum Position: {pendulum.GetPos().x:.2f}, {pendulum.GetPos().y:.2f}, {pendulum.GetPos().z:.2f}")
        print(f"Pendulum Velocity: {pendulum.GetPos_dt().x:.2f}, {pendulum.GetPos_dt().y:.2f}, {pendulum.GetPos_dt().z:.2f}")