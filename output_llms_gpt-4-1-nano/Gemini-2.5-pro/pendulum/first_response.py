import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
# General PyChrono setup
# ---------------------------------------------------------------------

# Create a Chrono physical system
my_system = chrono.ChSystemNSC() # Use Non-Smooth Contact (NSC) solver

# Set gravity
my_system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

# Solver settings (optional, defaults are often fine for simple cases)
# my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
# my_system.SetSolverMaxIterations(100)

# Collision settings (not strictly necessary for this example as no collisions are expected)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)

# ---------------------------------------------------------------------
# Pendulum Parameters
# ---------------------------------------------------------------------
pendulum_length = 2.0  # meters
pendulum_mass = 1.0    # kg
bob_radius = 0.1       # meters (for visualization and inertia)
initial_angle_deg = 45 # degrees from vertical

# Pivot point position
pivot_point_pos = chrono.ChVectorD(0, pendulum_length + 0.5, 0) # Suspend from this point

# ---------------------------------------------------------------------
# Create the ground body
# ---------------------------------------------------------------------
ground_body = chrono.ChBody()
ground_body.SetBodyFixed(True) # Ground is static
my_system.Add(ground_body)

# Optional: Add a visual shape to the ground for context
ground_box = chrono.ChBoxShape(4, 0.2, 4)
ground_box.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground_body.AddVisualShape(ground_box, chrono.ChFrameD(chrono.ChVectorD(0, -0.1, 0))) # Position it slightly below origin

# ---------------------------------------------------------------------
# Create the pendulum bob
# ---------------------------------------------------------------------
pendulum_bob = chrono.ChBody()
pendulum_bob.SetMass(pendulum_mass)

# Calculate initial position of the bob
initial_angle_rad = math.radians(initial_angle_deg)
bob_initial_x = pivot_point_pos.x + pendulum_length * math.sin(initial_angle_rad)
bob_initial_y = pivot_point_pos.y - pendulum_length * math.cos(initial_angle_rad)
bob_initial_z = pivot_point_pos.z
pendulum_bob.SetPos(chrono.ChVectorD(bob_initial_x, bob_initial_y, bob_initial_z))

# Set inertia (for a solid sphere: I = 2/5 * m * r^2)
inertia_val = (2.0/5.0) * pendulum_mass * bob_radius**2
pendulum_bob.SetInertiaXX(chrono.ChVectorD(inertia_val, inertia_val, inertia_val))

# Add visual shape for the bob
bob_sphere = chrono.ChSphereShape(bob_radius)
bob_sphere.SetColor(chrono.ChColor(0.8, 0.2, 0.2)) # Red color
pendulum_bob.AddVisualShape(bob_sphere)

my_system.Add(pendulum_bob)

# ---------------------------------------------------------------------
# Create the revolute joint
# ---------------------------------------------------------------------
# The joint connects the pendulum_bob (slave) to the ground_body (master).
# The joint is located at pivot_point_pos.
# The axis of rotation will be the Z-axis of the joint's coordinate system.
# We want the pendulum to swing in the X-Y plane, so the rotation axis should be along global Z.
# A ChFrameD defines a ChCoordsys. The default ChCoordsys has its Z axis aligned with global Z if QUNIT is used.

joint_frame = chrono.ChFrameD(pivot_point_pos, chrono.QUNIT) # QUNIT means identity quaternion (no rotation relative to global frame)

revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(pendulum_bob,  # Slave body
                          ground_body,   # Master body
                          joint_frame)   # Joint frame specified in absolute coordinates
my_system.Add(revolute_joint)

# ---------------------------------------------------------------------
# Create the Irrlicht visualization system
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum Simulation - PyChrono')
vis.Initialize()

# Add a camera
vis.AddCamera(chrono.ChVectorD(1.5, pendulum_length, 3.5),  # Camera position
              chrono.ChVectorD(0, pendulum_length * 0.7, 0)) # Look at point

# Add some lights
vis.AddSkyBox()
vis.AddTypicalLights()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
time_step = 0.01
simulation_time = 10.0  # seconds
log_frequency = 0.1   # Log data every 0.1 seconds
next_log_time = 0.0

print("Starting simulation...")
print(f"{'Time (s)':<10} | {'Pos X (m)':<12} | {'Pos Y (m)':<12} | {'Pos Z (m)':<12} | {'Vel X (m/s)':<12} | {'Vel Y (m/s)':<12} | {'Vel Z (m/s)':<12}")
print("-" * 90)

while vis.Run():
    current_time = my_system.GetChTime()

    vis.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.15)) # Background color
    vis.Render()
    # Optional: Draw contact points or other debug info
    # chronoirr.draw_ChPhysicsItem_contacts(vis, pendulum_bob)
    # chronoirr.draw_GASC_axis(vis.GetSceneManager().getVideoDriver(), 1.0, chronoirr.ALIGN_WORLD,chrono.ChVectorD(0,0,0),chrono.ChMatrix33D(1))
    vis.EndScene()

    my_system.DoStepDynamics(time_step)

    # Log data
    if current_time >= next_log_time:
        pos = pendulum_bob.GetPos()
        vel = pendulum_bob.GetPos_dt() # Linear velocity of CoG
        # ang_vel = pendulum_bob.GetWvel_loc() # Angular velocity in local frame

        print(f"{current_time:<10.2f} | "
              f"{pos.x:<12.3f} | {pos.y:<12.3f} | {pos.z:<12.3f} | "
              f"{vel.x:<12.3f} | {vel.y:<12.3f} | {vel.z:<12.3f}")
        next_log_time += log_frequency

    if current_time > simulation_time:
        vis.GetDevice().closeDevice() # Close window after simulation time

print("Simulation finished.")