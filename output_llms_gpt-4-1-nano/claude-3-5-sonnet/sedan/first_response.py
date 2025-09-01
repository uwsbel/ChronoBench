import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono system
chrono.SetChronoDataPath("../data/")

# Create a physical system
system = chrono.ChSystemNSC()

# Set the default gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground
ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create a falling sphere
sphere = chrono.ChBodyEasySphere(1, 1000, True, True)
sphere.SetPos(chrono.ChVectorD(0, 10, 0))
sphere.SetPos_dt(chrono.ChVectorD(5, 0, 0))  # Initial velocity
system.Add(sphere)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Basic PyChrono Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 14, -20))
vis.AddTypicalLights()

# Simulation loop
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

    # Break the loop after 10 seconds of simulation
    if system.GetChTime() > 10:
        break

print("Simulation complete")