# Import necessary modules
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine

# Create the simulation system
mysystem = chrono.ChSystemNSC()

# Create the Irrlicht visualization
application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(800, 600)
application.SetWindowTitle("Crank-Slider Mechanism")
application.AddTypicalLights()
application.AddLogo(0.5, 0.05)
application.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.5))
application.AddTypicalSky()
application.AddTypicalGround(chrono.ChVectorD(0, 0, 0))

# Create the floor (truss)
floor = chrono.ChBodyEasyBox(mysystem, 4, 0.1, 4, 1000, True, True, chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
mysystem.Add(floor)

# Create the crankshaft
crankshaft = chrono.ChBodyEasyCylinder(mysystem, 0.1, 0.5, 1000, True, True, chrono.ChVectorD(0, 0, 0))
crankshaft.SetPos(chrono.ChVectorD(0, 0.5, 0))
mysystem.Add(crankshaft)

# Create the connecting rod
connecting_rod = chrono.ChBodyEasyBox(mysystem, 0.1, 0.5, 0.1, 1000, True, True, chrono.ChVectorD(0, 0, 0))
connecting_rod.SetPos(chrono.ChVectorD(0, 1, 0))
mysystem.Add(connecting_rod)

# Create the piston
piston = chrono.ChBodyEasyBox(mysystem, 0.2, 0.2, 0.2, 1000, True, True, chrono.ChVectorD(0, 0, 0))
piston.SetPos(chrono.ChVectorD(0, 1.5, 0))
mysystem.Add(piston)

# Create the joints
crankshaft_pivot = chrono.ChLinkRevolute()
crankshaft_pivot.Initialize(crankshaft, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
mysystem.Add(crankshaft_pivot)

connecting_rod_joint = chrono.ChLinkRevolute()
connecting_rod_joint.Initialize(connecting_rod, crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 1, 0)))
mysystem.Add(connecting_rod_joint)

piston_joint = chrono.ChLinkRevolute()
piston_joint.Initialize(piston, connecting_rod, chrono.ChFrameD(chrono.ChVectorD(0, 1.5, 0)))
mysystem.Add(piston_joint)

# Create the motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
motor.SetSpeed(chrono.ChVectorD(0, 0, 10))  # 10 rad/s
mysystem.Add(motor)

# Set the simulation step and collision margin
mysystem.SetStep(0.01)
mysystem.SetCollisionMargin(0.001)

# Simulate and visualize
while application.GetDevice().run():
    mysystem.DoStepDynamics(0.01)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    application.DoStep()