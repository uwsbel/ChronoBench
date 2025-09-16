import pychrono as chrono
import pychrono.irrlicht as irr

# Create a Chrono system
system = chrono.ChSystemNSC()

# Create a ground body
ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create a rover body
rover = chrono.ChBodyEasyBox(1, 1, 2, 100)
rover.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(rover)

# Add a motor control driver for the rover
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rover, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(motor)

# Set up the visualization
application = irr.ChIrrApp(system, 'Curiosity Rover Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 3, -6))
application.AddTypicalLights()
application.SetTimestep(0.01)

# Run the simulation
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

# Clean up
application.GetSystem().Clear()