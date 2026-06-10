import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# =============================================================================
# Initialize the PyChrono environment
# =============================================================================
print("Initializing PyChrono Pendulum Simulation...")

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# =============================================================================
# Create the Ground Body
# =============================================================================
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetName("Ground")

# Add visual shape for ground
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(2, 0.1, 2))
ground.AddVisualShape(ground_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

# Add a small sphere at the pivot point for visualization
pivot_shape = chrono.ChSphereShape()
pivot_shape.GetSphereGeometry().rad = 0.05
ground.AddVisualShape(pivot_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

system.Add(ground)

# =============================================================================
# Create the Pendulum Body
# =============================================================================
pendulum_length = 1.5       # meters
pendulum_mass   = 1.0       # kg
pendulum_radius = 0.05      # visual radius of the rod

pendulum = chrono.ChBody()
pendulum.SetName("Pendulum")
pendulum.SetMass(pendulum_mass)

# Place the pendulum's center of mass at the midpoint of the rod
# Pivot is at (0,0,0), so COM is at (pendulum_length/2, 0, 0) initially
# We start at a 45-degree angle for interesting motion
initial_angle = math.radians(45)
com_x = (pendulum_length / 2) * math.sin(initial_angle)
com_y = -(pendulum_length / 2) * math.cos(initial_angle)
pendulum.SetPos(chrono.ChVectorD(com_x, com_y, 0))

# Set inertia tensor for a thin rod: I = (1/12) * m * L^2 about center
I_rod = (1.0 / 12.0) * pendulum_mass * pendulum_length**2
pendulum.SetInertiaXX(chrono.ChVectorD(I_rod, I_rod * 0.01, I_rod))

# Set initial rotation to match the starting angle
initial_rot = chrono.ChQuaternionD()
initial_rot.Q_from_AngZ(initial_angle)
pendulum.SetRot(initial_rot)

# Add visual shape: cylinder representing the pendulum rod
cyl_shape = chrono.ChCylinderShape()
cyl_shape.GetCylinderGeometry().rad = pendulum_radius
cyl_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0,  pendulum_length / 2, 0)
cyl_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -pendulum_length / 2, 0)
pendulum.AddVisualShape(cyl_shape)

# Add a sphere at the bob (end of rod)
bob_shape = chrono.ChSphereShape()
bob_shape.GetSphereGeometry().rad = 0.1
# Bob is at the bottom of the rod in local frame
pendulum.AddVisualShape(
    bob_shape,
    chrono.ChFrameD(chrono.ChVectorD(0, -pendulum_length / 2, 0))
)

system.Add(pendulum)

# =============================================================================
# Connect Ground and Pendulum with a Revolute Joint
# =============================================================================
# The revolute joint is placed at the pivot point (origin)
revolute_joint = chrono.ChLinkLockRevolute()

# Frame for the joint: at the pivot point, rotation axis along Z
joint_frame = chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 0),   # position: at origin (pivot)
    chrono.Q_from_AngX(0)         # no extra rotation needed (Z-axis is default)
)

revolute_joint.Initialize(pendulum, ground, joint_frame)
revolute_joint.SetName("RevoluteJoint")
system.Add(revolute_joint)

# =============================================================================
# Set Up Irrlicht Visualization
# =============================================================================
application = chronoirr.ChIrrApp(
    system,
    "Simple Pendulum Simulation",
    chronoirr.dimension2du(1024, 768)
)

application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 4))
application.AddTypicalLights()

# Draw the coordinate system and contact forces
application.AssetBindAll()
application.AssetUpdateAll()

# =============================================================================
# Simulation Loop Parameters
# =============================================================================
time_step    = 0.001   # seconds
log_interval = 0.1     # log every 0.1 seconds
next_log_time = 0.0

print("\n{'='*60}")
print("Starting Pendulum Simulation")
print(f"  Pendulum Length : {pendulum_length} m")
print(f"  Pendulum Mass   : {pendulum_mass} kg")
print(f"  Initial Angle   : {math.degrees(initial_angle):.1f} degrees")
print(f"  Time Step       : {time_step} s")
print("="*60)
print(f"{'Time (s)':>10} | {'Pos X (m)':>10} | {'Pos Y (m)':>10} | "
      f"{'Vel X (m/s)':>12} | {'Vel Y (m/s)':>12} | {'Speed (m/s)':>12}")
print("-" * 75)

application.SetTimestep(time_step)
application.SetTryRealtime(True)

# =============================================================================
# Run the Simulation
# =============================================================================
while application.GetDevice().run():
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    current_time = system.GetChTime()

    # Periodically log pendulum state
    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        speed = vel.Length()

        # Compute bob position (end of pendulum)
        bob_local = chrono.ChVectorD(0, -pendulum_length / 2, 0)
        bob_world = pendulum.TransformPointLocalToParent(bob_local)

        print(f"{current_time:>10.3f} | "
              f"{bob_world.x:>10.4f} | "
              f"{bob_world.y:>10.4f} | "
              f"{vel.x:>12.4f} | "
              f"{vel.y:>12.4f} | "
              f"{speed:>12.4f}")

        next_log_time += log_interval

    # Stop simulation after 30 seconds
    if current_time >= 30.0:
        print("\nSimulation completed (30 seconds elapsed).")
        break

print("\nSimulation finished.")