#!/usr/bin/env python3
"""
PyChrono Kraz Vehicle Simulation
================================
This script creates a complete simulation featuring a Kraz vehicle
driving on a rigid terrain with real-time Irrlicht visualization.
"""

import numpy as np
import math

# ============================================================================
# 1. INITIALIZE PYCHRONO ENVIRONMENT AND CORE COMPONENTS
# ============================================================================

# Import PyChrono core modules
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Set the path for vehicle data files
veh.SetDataPath(veh.GetDefaultDataPath())

# Create the physical system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# Set simulation parameters
system.SetSolverType(chrono.ChSolver.Type_SOR)
system.SetMaxItersSolverSpeed(50)
system.SetMaxItersSolverStab(50)
system.SetTolForce(1e-4)
system.Set timestep(0.002)  # 2ms timestep for stability

# Set contact force model
contact_method = veh.ChContactMethod_SMC
friction_model = chrono.ChContactFrictionModel_MicroSlip

print("=" * 60)
print("PyChrono Kraz Vehicle Simulation Initialized")
print("=" * 60)

# ============================================================================
# 2. ADD PHYSICAL SYSTEMS AND OBJECTS
# ============================================================================

# -------------------------------------------------------------------------
# 2.1 Create Rigid Terrain with Defined Friction and Restitution
# -------------------------------------------------------------------------

# Terrain parameters
terrain_length = 200.0  # meters
terrain_width = 50.0    # meters
terrain_thickness = 1.0  # meters

# Create ground material (SMC compatible)
ground_material = chrono.ChMaterialSurfaceSMC()
ground_material.SetFriction(0.8)       # Coefficient of friction
ground_material.SetRestitution(0.1)   # Coefficient of restitution
ground_material.SetAdhesion(0.0)       # No adhesion

# Create the terrain body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -terrain_thickness/2, 0))
ground.SetMaterialSurface(chrono.ChMaterialSurfaceSMC(ground_material))

# Create terrain collision shape (box)
ground_shape = chrono.ChCollisionShapeBox(terrain_length/2, terrain_thickness/2, terrain_width/2)
ground.AddCollisionShape(ground_shape, chrono.ChFrameD())
ground.SetCollide(True)

system.AddBody(ground)

# Add visual material for terrain
ground_visual = chrono.ChVisualShapeBox(terrain_length, terrain_thickness, terrain_width)
ground_visual.SetTexture(veh.GetDataPath() + "vehicle/textures/tile4.jpg")
ground.AddVisualShape(ground_visual, chrono.ChFrameD())

print(f"Terrain created: {terrain_length}m x {terrain_width}m")
print(f"  - Friction coefficient: 0.8")
print(f"  - Restitution coefficient: 0.1")

# -------------------------------------------------------------------------
# 2.2 Create Kraz Vehicle
# -------------------------------------------------------------------------

# Vehicle assembly location
vehicle_start_position = chrono.ChVectorD(-50, 1.5, 0)
vehicle_start_orientation = chrono.Q_ROTATE_Y_TO_Z

# Create the Kraz vehicle
print("\nInitializing Kraz vehicle...")
kraz = veh.KRAZ(system, veh.CollisionType_LINE, contact_method)

# Vehicle initialization parameters
kraz.SetInitPosition(veh.ChCoordsysD(vehicle_start_position, vehicle_start_orientation))
kraz.SetLightsVisualization(True)
kraz.SetChassisCollision(False)

# Initialize the vehicle
kraz.Initialize()

# Get vehicle subsystems for access
chassis = kraz.GetChassis()
chassis_body = chassis.GetBody()

print("Kraz vehicle initialized successfully!")
print(f"  - Vehicle mass: {chassis_body.GetMass():.2f} kg")
print(f"  - Number of wheels: {kraz.GetNumberAxles() * 2}")
print(f"  - Initial position: ({vehicle_start_position.x}, {vehicle_start_position.y}, {vehicle_start_position.z})")

# -------------------------------------------------------------------------
# 2.3 Create Wheel-Terrain Contact Materials
# -------------------------------------------------------------------------

# Create wheel material with specified properties
wheel_material = chrono.ChMaterialSurfaceSMC()
wheel_material.SetFriction(0.9)        # Higher friction for wheels
wheel_material.SetRestitution(0.05)    # Low restitution for rubber
wheel_material.SetYoungModulus(1.0e7)  # Stiffness
wheel_material.SetPoissonRatio(0.3)    # Poisson ratio

# Apply material to wheels
for axle in kraz.GetAxles():
    for wheel in axle.GetWheels():
        wheel.GetWheelBody().SetMaterialSurface(chrono.ChMaterialSurfaceSMC(wheel_material))

print("Wheel materials configured")

# ============================================================================
# 3. SET NECESSARY DEFAULT PARAMETERS
# ============================================================================

# -------------------------------------------------------------------------
# 3.1 Vehicle Initial Conditions
# -------------------------------------------------------------------------

# Initial vehicle velocity
initial_speed = 15.0  # m/s (approximately 54 km/h)

# Apply initial velocity to all wheels
for axle in kraz.GetAxles():
    for wheel in axle.GetWheels():
        wheel.GetWheelBody().SetPos_dt(chrono.ChVectorD(initial_speed, 0, 0))

# Set initial chassis velocity
chassis_body.SetPos_dt(chrono.ChVectorD(initial_speed, 0, 0))

print(f"\nInitial conditions set:")
print(f"  - Initial speed: {initial_speed} m/s ({initial_speed * 3.6:.1f} km/h)")

# -------------------------------------------------------------------------
# 3.2 Driver System Initialization
# -------------------------------------------------------------------------

# Create driver system for vehicle control
driver = veh.ChDriver(kraz.GetVehicle())

# Set driver parameters
driver.SetSteeringDelta(0.05)      # Maximum steering increment per timestep
driver.SetThrottleDelta(0.1)        # Maximum throttle increment per timestep
driver.SetBrakingDelta(0.2)         # Maximum braking increment per timestep

# Initialize driver at starting position
driver.Initialize()

print("Driver system initialized")

# -------------------------------------------------------------------------
# 3.3 Steering Controller Parameters
# -------------------------------------------------------------------------

# Create steering controller
steering_controller = veh.ChSteeringController()

# PID controller parameters for steering
steering_controller.SetGains(
    proportional_gain=0.5,    # Kp
    integral_gain=0.01,        # Ki
    derivative_gain=0.1       # Kd
)

# Set lookahead distance for steering
steering_controller.SetLookAheadDistance(5.0)  # meters
steering_controller.SetMaxSteeringAngle(0.5)   # radians

print("Steering controller configured")

# -------------------------------------------------------------------------
# 3.4 Speed Controller Parameters
# -------------------------------------------------------------------------

# Create speed controller (cruise control)
speed_controller = veh.ChSpeedController()

# Desired cruising speed
desired_speed = 20.0  # m/s (72 km/h)

speed_controller.SetGains(
    proportional_gain=1.0,
    integral_gain=0.05,
    derivative_gain=0.2
)
speed_controller.SetDesiredSpeed(desired_speed)

print(f"Speed controller configured (desired speed: {desired_speed} m/s)")
print(f"\nControl parameters:")
print(f"  - Steering delta: {driver.GetSteeringDelta():.3f} rad/s")
print(f"  - Throttle delta: {driver.GetThrottleDelta():.3f} 1/s")
print(f"  - Braking delta: {driver.GetBrakingDelta():.3f} 1/s")

# ============================================================================
# 4. CREATE REAL-TIME VISUALIZATION WITH IRRLICHT
# ============================================================================

# -------------------------------------------------------------------------
# 4.1 Initialize Irrlicht Application
# -------------------------------------------------------------------------

print("\nInitializing Irrlicht visualization...")

# Create the visualization application
vis = irr.CChIrrApp(
    system,
    "Kraz Vehicle Simulation",
    irr.dimension2du(1600, 900),  # Window size
    irr.E_WINDOW_ORIENTATION_LANDSCAPE
)

# Setup window title and other parameters
vis.AddTypicalLogo(veh.GetDataPath() + "logo_pychrono_alpha.png")
vis.AddTypicalSky()
vis.AddTypicalLights(
    irr.dimension2df(0.5, 0.5),  # Sun Size
    irr.dimension2df(0.5, 0.5)   # Ambient Size
)
vis.AddTypicalCamera(
    irr.vector3df(10, 8, -15),     # Camera position
    irr.vector3df(0, 2, 0)        # Camera target
)

# -------------------------------------------------------------------------
# 4.2 Camera Settings
# -------------------------------------------------------------------------

# Set camera properties
camera = vis.GetDevice().getSceneManager().getActiveCamera()
camera.setFOV(1.2)  # Field of view in radians

# Set camera near and far planes
camera.setNearClip(0.1)
camera.setFarClip(1000.0)

# Set camera movement speed
camera.setInputReceiverEnabled(True)

print("Irrlicht visualization initialized")
print(f"  - Window size: 1600 x 900")
print(f"  - Camera FOV: 1.2 rad")
print(f"  - Clip planes: 0.1 - 1000.0 m")

# -------------------------------------------------------------------------
# 4.3 Add Custom Visualization Elements
# -------------------------------------------------------------------------

# Add grid to terrain visualization
grid = irr.CChIrrWizard().add_Grid(
    vis.GetDevice(),
    10.0,              # Width
    2.0,               # Altitude offset
    50,                # X subdivisions
    50,                # Z subdivisions
    irr.video.SColor(100, 80, 80, 80),  # Color
    True               # Visible
)

# Add coordinate axes at origin
axes = irr.CChIrrWizard().add_LineAxes(
    vis.GetDevice(),
    10.0,              # Axis length
    irr.vector3df(0, 0, 0),  # Position
    5.0,               # Radius
    2.0                # Thickness
)

# -------------------------------------------------------------------------
# 4.4 Lighting Setup
# -------------------------------------------------------------------------

# Main directional light (sun)
sun_light = vis.GetDevice().getSceneManager().addLightSceneNode()
sun_light.setPosition(irr.vector3df(100, 100, 50))
sun_light.setLightType(irr.E_LIGHT_TYPE.ELT_DIRECTIONAL)
sun_light.setAmbient(irr.video.SColorf(0.3, 0.3, 0.3, 1.0))
sun_light.setDiffuseColor(irr.video.SColorf(1.0, 0.95, 0.8, 1.0))
sun_light.setSpecularColor(irr.video.SColorf(0.5, 0.5, 0.5, 1.0))

# Secondary fill light
fill_light = vis.GetDevice().getSceneManager().addLightSceneNode()
fill_light.setPosition(irr.vector3df(-50, 30, -30))
fill_light.setLightType(irr.E_LIGHT_TYPE.ELT_DIRECTIONAL)
fill_light.setAmbient(irr.video.SColorf(0.15, 0.15, 0.2, 1.0))
fill_light.setDiffuseColor(irr.video.SColorf(0.6, 0.7, 1.0, 1.0))

print("Lighting configured (sun + fill light)")

# -------------------------------------------------------------------------
# 4.5 Add Vehicle to Visualization
# -------------------------------------------------------------------------

# Add vehicle to visualization
kraz.GetVehicle().GetSystem().Add(chassis_body)
kraz.AddVisualizationAssets(vis.GetSceneManager())

print("Vehicle visualization added")

# -------------------------------------------------------------------------
# 4.6 Configure Shadow Settings
# -------------------------------------------------------------------------

# Enable shadows
shadow_enabled = vis.GetDevice().getSceneManager().addShadowLight(
    irr.vector3df(100, 100, 50),
    irr.video.SColorf(0.5, 0.5, 0.5, 1.0),
    200.0,    # Range
    20.0,     # Near
    100.0,    # Far
    1024      # Shadow map size
)

print("Shadow lighting enabled")

# ============================================================================
# 5. IMPLEMENT SIMULATION LOOP
# ============================================================================

print("\n" + "=" * 60)
print("STARTING SIMULATION")
print("=" * 60)

# Simulation parameters
simulation_step = 0.002           # 2ms timestep
real_time_factor = 1.0            # Run at real-time speed
simulation_duration = 30.0        # Total simulation time in seconds
output_interval = 1.0             # Output interval in seconds

# Initialize timing variables
current_time = 0.0
last_output_time = 0.0
frame_count = 0

# Data recording
time_history = []
speed_history = []
steering_history = []
throttle_history = []
brake_history = []

# Set up the application
vis.SetTimestep(simulation_step)
vis.Set罐管理模式(irr.E_UI_SCROLL_BAR.ESBM_VERTICAL)
vis.SetupProxy(vis.GetRenderDevice())

# Start the simulation loop
while (vis.GetDevice().run() and current_time < simulation_duration):
    
    # Get current simulation time
    step_start_time = vis.GetDevice().getTimer().getRealTime()
    
    # ---------------------------------------------------------
    # 5.1 Update Driver Input
    # ---------------------------------------------------------
    
    # Simple autonomous driving: maintain speed with slight steering
    # In real scenarios, this would come from path planning
    
    # Calculate steering input (slight sinusoidal path)
    steering_input = 0.1 * math.sin(current_time * 0.2)
    
    # Calculate throttle based on desired speed
    current_speed = chassis_body.GetPos_dt().Length()
    speed_error = desired_speed - current_speed
    throttle_input = min(1.0, max(0.0, speed_error * 0.1))
    
    # Apply brakes if speed exceeds desired by too much
    if current_speed > desired_speed * 1.1:
        brake_input = min(1.0, (current_speed - desired_speed) * 0.1)
    else:
        brake_input = 0.0
    
    # Set driver inputs
    driver.SetSteering(steering_input)
    driver.SetThrottle(throttle_input)
    driver.SetBraking(brake_input)
    
    # ---------------------------------------------------------
    # 5.2 Synchronize Systems
    # ---------------------------------------------------------
    
    # Synchronize the vehicle
    kraz.Synchronize(current_time, driver.GetInputs())
    
    # Synchronize terrain (if needed)
    ground.Synchronize(current_time)
    
    # Synchronize driver
    driver.Synchronize(current_time)
    
    # ---------------------------------------------------------
    # 5.3 Advance Simulation
    # ---------------------------------------------------------
    
    # Advance vehicle dynamics
    kraz.Advance(simulation_step)
    
    # Advance terrain (if animated)
    ground.Advance(simulation_step)
    
    # Advance driver system
    driver.Advance(simulation_step)
    
    # Advance physics system
    system.DoStepDynamics(simulation_step)
    
    # ---------------------------------------------------------
    # 5.4 Update Visualization
    # ---------------------------------------------------------
    
    # Update visualization
    vis.BeginScene()
    vis.DrawAll()
    
    # Draw additional info on screen
    draw_info = irr.CChIrrAppDrawTools(vis.GetDevice())
    
    # Create info text
    info_text = f"""
    === KRAZ Vehicle Simulation ===
    Time: {current_time:.2f} s / {simulation_duration:.2f} s
    Speed: {current_speed * 3.6:.1f} km/h
    Desired Speed: {desired_speed * 3.6:.1f} km/h
    Throttle: {throttle_input * 100:.1f}%
    Brake: {brake_input * 100:.1f}%
    Steering: {steering_input * 100:.1f}%
    FPS: {vis.GetDevice().getVideoDriver().getFPS()}
    """
    
    # Draw text shadow for readability
    draw_info.DrawAll(info_text, irr.vector2d_int32(12, 12), irr.video.SColor(150, 0, 0, 0))
    draw_info.DrawAll(info_text, irr.vector2d_int32(10, 10), irr.video.SColor(255, 255, 255, 255))
    
    # Draw speed gauge
    gauge_position = irr.vector2d_int32(vis.GetDevice().getVideoDriver().getScreenSize().Width - 150, 50)
    draw_info.DrawSpeedGauge(
        "SPEED",
        current_speed * 3.6,     # Convert to km/h
        0,                       # Min speed
        120,                     # Max speed
        gauge_position,
        100,                     # Radius
        irr.video.SColor(255, 0, 255, 0),   # Normal color
        irr.video.SColor(255, 255, 0, 0)    # Warning color
    )
    
    # Draw throttle/brake gauges
    gauge_position2 = irr.vector2d_int32(50, 50)
    draw_info.DrawThrottleBrakeGauges(
        throttle_input,
        brake_input,
        gauge_position2,
        50,
        irr.video.SColor(255, 0, 255, 0)
    )
    
    vis.EndScene()
    
    # ---------------------------------------------------------
    # 5.5 Record Data
    # ---------------------------------------------------------
    
    # Record simulation data
    if current_time - last_output_time >= output_interval:
        time_history.append(current_time)
        speed_history.append(current_speed)
        steering_history.append(steering_input)
        throttle_history.append(throttle_input)
        brake_history.append(brake_input)
        last_output_time = current_time
        
        # Print status every output interval
        print(f"[{current_time:6.2f}s] Speed: {current_speed * 3.6:6.1f} km/h | "
              f"Throttle: {throttle_input * 100:5.1f}% | "
              f"Brake: {brake_input * 100:5.1f}% | "
              f"Steering: {steering_input * 100:5.1f}%")
    
    # Update current time
    current_time += simulation_step
    frame_count += 1
    
    # Optional: Sleep to maintain real-time execution
    # (remove for maximum speed simulation)
    elapsed = vis.GetDevice().getTimer().getRealTime() - step_start_time
    sleep_time = int((simulation_step * 1000 / real_time_factor - elapsed) * 1000)
    if sleep_time > 0:
        import time
        time.sleep(sleep_time / 1000000.0)

# ============================================================================
# 6. SIMULATION COMPLETION
# ============================================================================

print("\n" + "=" * 60)
print("SIMULATION COMPLETED")
print("=" * 60)

# Print final statistics
print(f"\nSimulation Statistics:")
print(f"  - Total simulation time: {current_time:.2f} seconds")
print(f"  - Total frames: {frame_count}")
print(f"  - Average FPS: {frame_count / current_time:.1f}")
print(f"  - Average speed: {np.mean(speed_history) * 3.6:.1f} km/h")
print(f"  - Max speed: {np.max(speed_history) * 3.6:.1f} km/h")
print(f"  - Min speed: {np.min(speed_history) * 3.6:.1f} km/h")

# Save recorded data to file
import json

data_output = {
    "simulation_parameters": {
        "timestep": simulation_step,
        "duration": simulation_duration,
        "desired_speed": desired_speed,
        "initial_speed": initial_speed
    },
    "time_history": time_history,
    "speed_history": [s * 3.6 for s in speed_history],  # Convert to km/h
    "steering_history": steering_history,
    "throttle_history": throttle_history,
    "brake_history": brake_history,
    "statistics": {
        "average_speed_kmh": float(np.mean(speed_history) * 3.6),
        "max_speed_kmh": float(np.max(speed_history) * 3.6),
        "min_speed_kmh": float(np.min(speed_history) * 3.6),
        "total_distance_m": float(np.sum(speed_history) * simulation_step)
    }
}

# Save to JSON file
with open("simulation_data.json", "w") as f:
    json.dump(data_output, f, indent=2)

print(f"\nData saved to: simulation_data.json")
print(f"Total distance traveled: {data_output['statistics']['total_distance_m']:.1f} meters")

# Clean up
vis.GetDevice().closeDevice()

print("\nSimulation cleanup complete.")
print("=" * 60)