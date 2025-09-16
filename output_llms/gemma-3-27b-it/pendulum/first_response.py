import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ---
#  Create the simulation system and add items
# ---

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create the ground body
body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVectorD(0, -1, 0))
body_ground.SetShapeBoxes(0.5, 0.1, 0.5)
body_ground.AddAsset(chrono.ChColor(0.5, 0.5, 0.5))
system.Add(body_ground)

# Create the pendulum body
body_pendulum = chrono.ChBodyEasy()
body_pendulum.SetMass(1.0)
body_pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
body_pendulum.SetShapeSphere(0.1)
body_pendulum.AddAsset(chrono.ChColor(0.2, 0.8, 0.2))

# Calculate inertia tensor
inertia = chrono.ChMatrix33()
inertia.SetDiagonal(chrono.ChVector3D(0.01, 0.01, 0.01))  # Approximate inertia for a sphere
body_pendulum.SetInertia(inertia)

system.Add(body_pendulum)

# Create a revolute joint to connect the pendulum to the ground
joint = chrono.ChLinkRevolute()
joint.Initialize(body_ground, body_pendulum, chrono.ChCoordsys(chrono.ChVectorD(0, -1, 0)))
system.Add(joint)

# ---
#  Create an Irrlicht application to visualize the system
# ---

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -2))
vis.AddTypicalLights()

# ---
#  Simulation loop
# ---

time_step = 0.005
time_end = 10.0
current_time = 0.0

while vis.Run() and current_time < time_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    # Log pendulum position and velocity
    if current_time % 0.5 < time_step:
        pos = body_pendulum.GetPos()
        vel = body_pendulum.GetPos_WVEL()
        print(f"Time: {current_time:.2f}, Pendulum Position: {pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}, Velocity: {vel.x:.4f}, {vel.y:.4f}, {vel.z:.4f}")

    current_time += time_step