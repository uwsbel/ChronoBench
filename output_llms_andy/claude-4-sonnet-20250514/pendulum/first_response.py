import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the PyChrono environment
chrono.SetChronoDataPath('/path/to/chrono/data/')  # Adjust this path as needed

# Create the physical system
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create ground body (fixed)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)

# Add a visual shape to the ground (a box)
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(2, 0.1, 0.5)
ground.AddVisualShape(ground_shape, chrono.ChFrameD())

# Pendulum parameters
pendulum_length = 2.0  # Length of pendulum arm
pendulum_mass = 1.0    # Mass of pendulum bob
pendulum_radius = 0.1  # Radius of pendulum bob

# Create pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(pendulum_mass)

# Set initial position (hanging down with slight angle for initial motion)
initial_angle = math.pi / 6  # 30 degrees
pendulum_pos = chrono.ChVectorD(
    pendulum_length * math.sin(initial_angle),
    -pendulum_length * math.cos(initial_angle),
    0
)
pendulum.SetPos(pendulum_pos)

# Calculate inertia for a point mass at distance L from pivot
inertia = pendulum_mass * pendulum_length * pendulum_length
pendulum.SetInertiaXX(chrono.ChVectorD(inertia, inertia, inertia))

# Add visual shape to pendulum (sphere for the bob)
pendulum_shape = chrono.ChSphereShape()
pendulum_shape.GetSphereGeometry().rad = pendulum_radius
pendulum.AddVisualShape(pendulum_shape, chrono.ChFrameD())

# Add collision shape (optional, for more realistic simulation)
pendulum.GetCollisionModel().ClearModel()
pendulum.GetCollisionModel().AddSphere(chrono.ChMaterialSurface.DefaultMaterial(chrono.ChContactMethod_NSC), pendulum_radius)
pendulum.GetCollisionModel().BuildModel()
pendulum.SetCollide(True)

system.Add(pendulum)

# Create revolute joint between ground and pendulum
# The joint will be at the origin (pivot point)
revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(
    ground,
    pendulum,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
)
system.Add(revolute_joint)

# Create a visual representation of the pendulum arm (line/rod)
arm_shape = chrono.ChCylinderShape()
arm_shape.GetCylinderGeometry().rad = 0.02
arm_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
arm_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -pendulum_length, 0)

# Create a separate body for the arm visualization
arm_body = chrono.ChBody()
arm_body.SetBodyFixed(True)
arm_body.AddVisualShape(arm_shape, chrono.ChFrameD())
system.Add(arm_body)

# Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(4, 2, 6))
vis.AddTypicalLights()

# Simulation parameters
time_step = 0.01
simulation_time = 0.0
log_interval = 1.0  # Log every 1 second
next_log_time = 0.0

print("Starting pendulum simulation...")
print("Time\tPosition (x, y, z)\t\tVelocity (x, y, z)")
print("-" * 70)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    # Update the arm visualization to follow the pendulum
    pendulum_pos = pendulum.GetPos()
    arm_rotation = chrono.ChQuaternionD()
    arm_rotation.Q_from_AngAxis(-math.atan2(pendulum_pos.x, -pendulum_pos.y), chrono.ChVectorD(0, 0, 1))
    arm_body.SetPos(chrono.ChVectorD(pendulum_pos.x/2, pendulum_pos.y/2, 0))
    arm_body.SetRot(arm_rotation)
    
    # Log pendulum state periodically
    if simulation_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"{simulation_time:.2f}\t({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})\t\t({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        next_log_time += log_interval
    
    # Advance the simulation
    system.DoStepDynamics(time_step)
    simulation_time += time_step
    
    vis.EndScene()

print("Simulation completed.")