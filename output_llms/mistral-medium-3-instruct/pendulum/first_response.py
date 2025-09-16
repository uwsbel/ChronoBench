import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Set the path to Chrono data directory

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add physical systems and objects

# Create the ground body
ground = chrono.ChBodyEasyBox(4, 0.2, 4, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create the pendulum bob
pendulum = chrono.ChBodyEasySphere(0.2, 1, True, True)  # radius, density, visualization, collision
pendulum.SetPos(chrono.ChVectorD(0, 1.5, 0))
system.Add(pendulum)

# 3. Set up the revolute joint
revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0)))
system.Add(revolute)

# 4. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Simple Pendulum")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.5, 2))
vis.AddTypicalLights()
vis.AssetBindAll()
vis.AssetUpdateAll()

# 5. Simulation parameters
time_step = 0.01
sim_time = 10.0
output_step = 0.1  # Log every 0.1 seconds

# 6. Simulation loop
current_time = 0
output_time = 0
while current_time < sim_time:
    # Advance simulation
    system.DoStepDynamics(time_step)
    current_time += time_step

    # Visualization update
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Periodic logging
    if current_time >= output_time:
        # Get pendulum position and velocity
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()

        # Calculate angle (simple approximation)
        angle = math.atan2(pos.x, 1.5 - pos.y)  # Approximate angle from vertical

        print(f"Time: {current_time:.2f}s | Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | "
              f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f}) | Angle: {math.degrees(angle):.1f}°")

        output_time += output_step

    # Check for user exit
    if vis.Run() == 0:
        break