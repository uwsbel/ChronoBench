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
body_ground.AddAsset(chrono.ChBoxShape())
system.Add(body_ground)

# Create the pendulum body
body_pendulum = chrono.ChBodyEasy()
body_pendulum.SetMass(1.0)
body_pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
body_pendulum.SetShapeCylinder(0.1, 1.0)
body_pendulum.AddAsset(chrono.ChCylinderShape())

# Calculate inertia tensor (for a cylinder around its center)
inertia = chrono.ChMatrix33()
inertia.SetDiagonal([1/12 * body_pendulum.GetMass() * (3 * 0.1**2 + 1.0**2),
                     1/12 * body_pendulum.GetMass() * (3 * 0.1**2 + 1.0**2),
                     1/2 * body_pendulum.GetMass() * 0.1**2])
body_pendulum.SetInertia(inertia)

system.Add(body_pendulum)

# Create a revolute joint to connect the pendulum to the ground
joint = chrono.ChLinkRevolute()
joint.Initialize(body_ground, body_pendulum, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
system.Add(joint)

# ---
#  Create an Irrlicht application to visualize the system
# ---

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 2, -2))
vis.AddTypicalLights()

# ---
#  Simulation loop
# ---

time_step = 0.01
time_end = 10.0
current_time = 0.0

while current_time <= time_end:
    system.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Log pendulum position and velocity
    pos = body_pendulum.GetPos()
    vel = body_pendulum.GetVel()
    print(f"Time: {current_time:.2f}, Pendulum Position: {pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}, Velocity: {vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f}")

    current_time += time_step

vis.GetIrrlichtApplication().GetDevice().CloseDevice()