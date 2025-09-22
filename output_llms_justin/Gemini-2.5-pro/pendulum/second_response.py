import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
# Original Script Error Identification and Correction Summary:
# 1. Pendulum Cylinder Visualization Orientation:
#    - Original: chrono.QuatFromAngleY(chrono.CH_PI_2) for the pendulum's visual cylinder.
#    - Correction: For a cylinder whose height is along its local Y-axis, and a pendulum
#                  body whose length is along its X-axis, the correct rotation to align
#                  them is chrono.QuatFromAngleZ(chrono.CH_PI_2). This aligns the
#                  cylinder's length with the pendulum body's X-axis. This correction
#                  is incorporated into the modified pendulum visualization.
# 2. Variable Name Reuse:
#    - Original: `cyl_1` was used for both ground and pendulum visual shapes.
#    - Correction: Using distinct variable names (e.g., `ground_cyl_vis`, `pend_cyl_vis`)
#                  for better readability. This is a best-practice change.
# Other aspects of the original script were generally consistent for its intended purpose.
# ---------------------------------------------------------------------

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set gravitational acceleration (Modified for Moon)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  # Moon gravity: g_moon = 1.62 m/s^2

# --- Ground Body ---
# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground to mark the pivot's attachment point.
# This part is kept from the original script for reference.
ground_cyl_vis = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(ground_cyl_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, 1))) # Positioned at (0,0,1) global

# --- Pendulum Body ---
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum

# Mass and Inertia (Modified as per instructions)
pend_1.SetMass(2.0)  # Mass set to 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Inertia tensor [kg·m^2]

# Pendulum Visualization (Modified: dimensions, orientation)
# The pendulum rod is visualized as a cylinder.
# Its length (pivot to CoM) L_pend is half the cylinder's height if CoM is at geometric center.
pend_cyl_radius = 0.1  # Radius of 0.1 m
pend_cyl_height = 1.5 # Height of 1.5 m
L_pend = pend_cyl_height / 2.0 # Effective pendulum length (pivot to CoM) = 0.75 m

pend_cyl_vis = chrono.ChVisualShapeCylinder(pend_cyl_radius, pend_cyl_height)
pend_cyl_vis.SetColor(chrono.ChColor(0.6, 0, 0))  # Set color to red

# The ChVisualShapeCylinder has its longitudinal axis along its local Y-axis.
# Assume the pendulum body's length is defined along its X-axis.
# To align the cylinder's length (local Y) with the body's X-axis,
# rotate the cylinder's frame by +90 degrees around the body's Z-axis.
# The cylinder visual shape is centered at the pendulum's CoM (chrono.VNULL in local frame).
pend_1.AddVisualShape(pend_cyl_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(chrono.CH_PI_2)))

# Initial position and orientation of the pendulum's CoM
# The pivot point is fixed in global coordinates.
pivot_point_abs = chrono.ChVector3d(0, 0, 1)

# Set the pendulum's initial CoM position.
# Example: Initially horizontal, with its arm along the global X-axis.
pend_1.SetPos(pivot_point_abs + chrono.ChVector3d(L_pend, 0, 0))
# Set initial orientation: pendulum body's axes aligned with global axes.
pend_1.SetRot(chrono.QUNIT)

# Initial Angular Velocity (NEW requirement)
# Set an initial angular velocity for the pendulum body in the parent (global) frame.
# This will give it a 3D motion. Example: (0 rad/s around X, 2.0 rad/s around Y, 0.5 rad/s around Z)
pend_1.SetAngVelParent(chrono.ChVector3d(0, 2.0, 0.5))

# --- Joint ---
# Spherical Joint (Modified: replaced Revolute joint)
sph_joint = chrono.ChLinkLockSpherical()

# Initialize the joint. It connects `ground` and `pend_1`.
# The ChFramed(pivot_point_abs) specifies the joint location in absolute coordinates.
# Markers on both bodies will be created coincident with this absolute frame.
sph_joint.Initialize(ground, pend_1, chrono.ChFramed(pivot_point_abs))
sys.AddLink(sph_joint)

# Joint Visualization (NEW requirement)
# Add a sphere shape to visualize the joint itself. Radius 2.0.
joint_vis_sphere = chrono.ChVisualShapeSphere(2.0) # Radius 2.0 as per instruction
joint_vis_sphere.SetColor(chrono.ChColor(0.2, 0.2, 0.8))  # Bluish color for the joint sphere
sph_joint.AddVisualShape(joint_vis_sphere)


# --- Irrlicht Visualization System ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum: Spherical Joint, Moon Gravity, Initial Velocity') # Updated title
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
# Adjust camera position and target for a better view of the pendulum.
# Camera position relative to pivot, looking at the pivot.
camera_pos_relative = chrono.ChVector3d(L_pend * 3, L_pend * 2.5, L_pend * 2)
vis.AddCamera(pivot_point_abs + camera_pos_relative, pivot_point_abs)
vis.AddTypicalLights()

# --- Simulation Loop ---
time_step = 1e-3  # Time step for the simulation (0.001 seconds)
log_info = True  # Flag to control console logging (print info once)

while vis.Run():
    vis.BeginScene()  # Begin rendering the scene
    vis.Render()      # Render the scene
    vis.EndScene()    # End rendering

    sys.DoStepDynamics(time_step)  # Advance the simulation by one time step

    # Log information after approximately 1 second of simulation time
    if log_info and sys.GetChTime() >= 1.0:
        pos_1 = pend_1.GetPos()  # Get the CoM position of the pendulum
        lin_vel_1 = pend_1.GetPosDt()  # Get the CoM linear velocity
        ang_vel_1_local = pend_1.GetAngVelLocal() # Get angular velocity in pendulum's local frame

        print(f"--- Simulation info at t = {sys.GetChTime():.3f} s ---")
        print(f"Pendulum CoM Position:      x={pos_1.x:.3f}, y={pos_1.y:.3f}, z={pos_1.z:.3f} m")
        print(f"Pendulum CoM Linear Vel.:   x={lin_vel_1.x:.3f}, y={lin_vel_1.y:.3f}, z={lin_vel_1.z:.3f} m/s")
        print(f"Pendulum Ang. Vel. (local): x={ang_vel_1_local.x:.3f}, y={ang_vel_1_local.y:.3f}, z={ang_vel_1_local.z:.3f} rad/s")
        
        log_info = False  # Disable further logging to keep the console output clean