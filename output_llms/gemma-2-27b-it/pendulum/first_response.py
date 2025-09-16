import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(1, 1, 0.2, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
mysystem.Add(ground)

# Create the pendulum bob
bob = chrono.ChBodyEasySphere(0.2, 1000, True, True)
bob.SetPos(chrono.ChVectorD(0, 1, 0))
mysystem.Add(bob)

# Create a revolute joint between the ground and the bob
joint = chrono.ChLinkRevolute()
joint.Initialize(ground, bob, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
mysystem.Add(joint)

# ---------------------------------------------------------------------
#
#  Create the Irrlicht visualization system
#

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Simple Pendulum")
vis.Initialize()
vis.AddTypicalSky()
vis.AddTypicalCamera(chrono.ChVectorD(2, 2, 2))
vis.SetCameraPosition(chrono.ChVectorD(2, 2, 2))

# ---------------------------------------------------------------------
#
#  Simulation loop
#

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    mysystem.DoStepDynamics(0.01)

    # Print the pendulum's position and velocity every 10 steps
    if mysystem.GetChTime() % 0.1 == 0:
        print("Time:", mysystem.GetChTime())
        print("Position:", bob.GetPos())
        print("Velocity:", bob.GetPos_dt())