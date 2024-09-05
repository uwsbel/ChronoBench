import pychrono as chrono
import pychrono.irrlicht as vis

# Initialize Chrono environment
chrono.SetChronoDataPath("path/to/chrono/data")  # Replace with your Chrono data path
system = chrono.ChSystemNSC()

# Create a fixed truss
truss = chrono.ChBodyEasyBox(1.0, 0.5, 0.2, 1000)  # Dimensions and mass
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
system.Add(truss)

# Create a rotating bar
bar = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000)  # Radius, height, mass
bar.SetPos(chrono.ChVectorD(0.5, 0, 0))
bar.SetRot(chrono.Q_from_Euler(chrono.rad2deg(0), chrono.rad2deg(0), chrono.rad2deg(0)))
system.Add(bar)

# Create two gears
gear1 = chrono.ChBodyEasyCylinder(0.1, 0.05, 100)  # Radius, height, mass
gear1.SetPos(chrono.ChVectorD(0.2, 0, 0))
gear1.SetRot(chrono.Q_from_Euler(chrono.rad2deg(0), chrono.rad2deg(0), chrono.rad2deg(0)))
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.1, 0.05, 100)  # Radius, height, mass
gear2.SetPos(chrono.ChVectorD(0.7, 0, 0))
gear2.SetRot(chrono.Q_from_Euler(chrono.rad2deg(0), chrono.rad2deg(0), chrono.rad2deg(0)))
system.Add(gear2)

# Create gear motor
motor = chrono.ChMotorRotation(gear1.GetJoint(0), chrono.ChVectorD(0, 0, 1), 100)  # Motor attached to gear1
motor.SetMotorSpeed(chrono.rad2deg(10))  # Set constant rotation speed

# Visualize the setup
vis.ChVisualizationSystem()
vis.Add(system)
vis.SetCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.SetBackground(chrono.ChColor(0.8, 0.8, 0.8))
vis.SetLight(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.SetMaterial(chrono.ChColor(0.5, 0.5, 0.5), chrono.ChColor(0.2, 0.2, 0.2))

# Run the simulation
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.DoStepDynamics(0.01)

# Keep the visualization window open
vis.Run()