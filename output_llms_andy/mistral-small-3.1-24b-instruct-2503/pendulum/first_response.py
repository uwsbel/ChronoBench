import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Set up the simulation system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 10, 0.5, 1000)
ground.SetPos(chrono.ChVectorD(0, -0.25, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create the pendulum body
pendulum = chrono.ChBodyEasyCylinder(0.1, 0.5, 10, True, True, True, chrono.ChMaterialSurfaceNSC())
pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
pendulum.SetMass(1)  # Set the mass of the pendulum
pendulum.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))  # Set the inertia of the pendulum
system.Add(pendulum)

# Create a revolute joint to connect the pendulum to the ground
revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(revolute)

# Set up the visualization system
application = chronoirr.ChIrrApp()
application.AddSystem(system)
application.SetWindowTitle("Simple Pendulum Simulation")
application.SetWindowSize(800, 600)
application.SetTryFullscreen(False)
application.SetShowInfo(true)
application.SetPaused(False)
application.SetTimestep(0.01)
application.SetTryFullscreen(False)

# Add a camera to the visualization system
camera = chronoirr.ChIrrToolsCamera(application.GetDevice())
camera.SetTarget(chrono.ChVectorD(0, 0, 0))
camera.SetPos(chrono.ChVectorD(2, 2, 2))
application.AddCamera(camera)

# Run the simulation
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    # Log the pendulum's position and velocity
    pos = pendulum.GetPos()
    vel = pendulum.GetPos_dt()
    print(f"Position: {pos}, Velocity: {vel}")

# Close the application
application.GetDevice().closeDevice()