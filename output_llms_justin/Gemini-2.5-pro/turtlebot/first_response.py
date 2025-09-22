import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# -----------------------------------------------------------------------------
# Set Chrono data paths
# -----------------------------------------------------------------------------
# It's important to set the data path for Chrono and Chrono::Vehicle
# This path should point to the 'data/' directory of your Chrono installation
try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
    veh.SetDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'vehicle', ''))
except KeyError:
    print("Error: CHRONO_DATA_DIR environment variable not set.")
    print("Please set CHRONO_DATA_DIR to the 'data' directory of your Chrono installation.")
    exit(1)


# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
timestep = 0.01  # Time step for the simulation
simulation_duration = 20.0  # Total simulation time in seconds

# Initial Turtlebot position and orientation
initial_pos = chrono.ChVector3d(0, 0.2, 0)  # x, y, z coordinates
# Q_from_AngZ(angle_rad) for rotation around Z-axis
# QUNIT means no rotation (aligned with global axes)
initial_rot = chrono.ChQuaterniond(1, 0, 0, 0) # No initial rotation, facing +X

# Motor control parameters
time_to_turn_left_start = 3.0
time_to_turn_left_end = 6.0
time_to_turn_right_start = 9.0
time_to_turn_right_end = 12.0

# Speeds for movement (rad/s for wheels)
# Turtlebot wheel radius is approx 0.033m
# Linear speed = wheel_ang_vel * radius
# Differential drive:
# Forward: left_speed = right_speed
# Left turn: left_speed < right_speed
# Right turn: left_speed > right_speed
straight_speed_rad_s = 5.0  # rad/s, approx 0.165 m/s linear speed
turn_inner_speed_rad_s = 2.0
turn_outer_speed_rad_s = 5.0

# -----------------------------------------------------------------------------
# 1. Initialize PyChrono environment and core components
# -----------------------------------------------------------------------------
print("Initializing Chrono system...")
# Create a Chrono system (NSC: Non-Smooth Contact)
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Set solver settings (optional, but good for stability/performance)
system.SetSolverType(chrono.ChSolver.Type.BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(100)
system.SetMaxPenetrationRecoverySpeed(1.0) # Helps with interpenetrations

# -----------------------------------------------------------------------------
# 2. Add the required physical systems and objects
# -----------------------------------------------------------------------------

# --- Create the ground ---
print("Creating ground...")
ground_material = chrono.ChContactMaterialNSC() # For NSC systems
ground_material.SetFriction(0.9)
ground_material.SetRestitution(0.01)

ground = chrono.ChBodyEasyBox(40, 2, 40, 1000, True, True, ground_material) # width, height, depth, density, visualize, collide
ground.SetPos(chrono.ChVector3d(0, -1, 0)) # Position its top surface at y=0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
system.Add(ground)


# --- Initialize the Turtlebot ---
print("Initializing Turtlebot...")
# Create the Turtlebot robot
# The constructor takes the ChSystem and an initial pose (ChFrame)
# Note: TurtleBot constructor might expect system and initial ChFrame directly
# Let's create the robot and then initialize it.
robot = veh.TurtleBot(system)

# Initialize the Turtlebot at a specific position and orientation
# The ChFramed object combines position (ChVector3d) and orientation (ChQuaterniond)
initial_frame = chrono.ChFramed(initial_pos, initial_rot)
robot.Initialize(initial_frame)

# Set a dummy driver (needed by Chrono::Vehicle framework, even if we control motors directly)
# For Turtlebot, we often control wheel speeds directly.
# A simple driver is still needed for the framework.
driver = veh.ChDriver(robot.GetVehicle()) # Pass the underlying wheeled vehicle
robot.SetDriver(driver) # This allows robot.Update() to work as expected.


# -----------------------------------------------------------------------------
# 3. Set up real-time visualization using Irrlicht
# -----------------------------------------------------------------------------
print("Initializing Irrlicht visualization...")
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Turtlebot Simulation')
vis.Initialize()

# Add Irrlicht components
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddTypicalLights()

# Set camera position and target
# Arguments: camera_pos (ChVector3d), target_pos (ChVector3d)
vis.AddCamera(chrono.ChVector3d(3, 2.5, 4), chrono.ChVector3d(0, 0.5, 0)) # Camera looking at the origin area

# To have the Irrlicht GUI do lazy updates (only when needed)
vis.SetSymbolscale(0.1) # Scale of contact normal symbols, etc.
vis.EnableContactDrawing(irr.ContactsDrawMode_CONTACT_FORCES) # Draw contact forces

# -----------------------------------------------------------------------------
# 4. Implement the simulation loop
# -----------------------------------------------------------------------------
print(f"Starting simulation loop for {simulation_duration} seconds...")
current_time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    # Optionally draw a small Chrono logo in the corner
    irr.draw_chrono_logo(vis, chrono.ChVector2d(10,10), chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')

    # --- Motor Control Logic ---
    # The TurtleBot class has a SetMotorSpeed(left_speed_rad_s, right_speed_rad_s) method
    left_wheel_speed = 0.0
    right_wheel_speed = 0.0

    if time_to_turn_left_start <= current_time < time_to_turn_left_end:
        # Turn Left: Slower left wheel, faster right wheel
        left_wheel_speed = turn_inner_speed_rad_s
        right_wheel_speed = turn_outer_speed_rad_s
        if current_time < time_to_turn_left_start + timestep: # Print once at start of turn
             print(f"Time: {current_time:.2f}s - Turning Left")
    elif time_to_turn_right_start <= current_time < time_to_turn_right_end:
        # Turn Right: Faster left wheel, slower right wheel
        left_wheel_speed = turn_outer_speed_rad_s
        right_wheel_speed = turn_inner_speed_rad_s
        if current_time < time_to_turn_right_start + timestep: # Print once at start of turn
             print(f"Time: {current_time:.2f}s - Turning Right")
    else:
        # Go Straight (or stop if straight_speed_rad_s is 0)
        left_wheel_speed = straight_speed_rad_s
        right_wheel_speed = straight_speed_rad_s
        if current_time == 0.0 or \
           (current_time > time_to_turn_left_end and current_time < time_to_turn_left_end + timestep) or \
           (current_time > time_to_turn_right_end and current_time < time_to_turn_right_end + timestep):
            if straight_speed_rad_s != 0:
                print(f"Time: {current_time:.2f}s - Moving Straight")
            else:
                print(f"Time: {current_time:.2f}s - Stopped")


    robot.SetMotorSpeed(left_wheel_speed, right_wheel_speed)

    # --- Update and Advance Simulation ---
    # Update the robot (this will process driver inputs, sensor data, etc.)
    # For direct motor control, this might mostly update internal states.
    robot.Update(current_time, driver) # Pass current time and driver

    # Advance the simulation by one timestep
    system.DoStepDynamics(timestep)

    # Update current time
    current_time += timestep

    vis.EndScene()

    # Exit condition for the loop
    if current_time >= simulation_duration:
        print(f"Simulation ended at time: {current_time:.2f}s")
        break

print("Simulation complete.")

# Proper cleanup is good, though Python's GC will handle it
del vis
del system