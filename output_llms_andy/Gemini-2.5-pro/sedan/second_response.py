import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

print(veh) # Original print statement, kept as is

"""
!!!! Set this path before running the demo!
"""
# This ensures Chrono data paths are correctly set.
# It's good practice to keep it if CHRONO_DATA_DIR might not be universally set.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------------------
# Parameters
# ----------------------------

# Initial vehicle locations and orientations
initLoc1 = chrono.ChVector3d(0, -2, 0.5) # Offset vehicle 1 slightly
initRot1 = chrono.ChQuaterniond(1, 0, 0, 0)

initLoc2 = chrono.ChVector3d(0, 2, 0.5)  # Place vehicle 2 next to vehicle 1
initRot2 = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE # No chassis collision for simplicity

# Type of tire model (RIGID, TMEASY, PAC02, FIALA, etc.)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height (world frame y)
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera (relative to vehicle1 chassis)
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
# contact_vis = False # Unused in original, kept commented

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size # Tire step size can be different from system step size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Sinusoidal steering parameters
steer_amplitude = 0.6  # radians
steer_frequency = 0.3  # Hz
throttle_value = 0.3   # Constant throttle for both vehicles
braking_value = 0.0    # Constant braking for both vehicles

# --------------
# Create System
# --------------

# Create the Chrono physical system based on contact method
if contact_method == chrono.ChContactMethod_NSC:
    my_system = chrono.ChSystemNSC()
elif contact_method == chrono.ChContactMethod_SMC:
    my_system = chrono.ChSystemSMC()
    # Note: If using SMC, material properties for terrain would need to be ChContactMaterialSMC
else:
    print("Error: Unknown contact method.")
    exit(1)

my_system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81)) # Set global gravity
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # Set collision system type


# --------------------
# Create Vehicle 1
# --------------------
# Create the Sedan vehicle, set parameters, and initialize
vehicle1 = veh.Sedan(system=my_system,
                     fixed=False,
                     tire_model=tire_model,
                     contact_method=contact_method, # Ensures vehicle is configured for the system's contact type
                     chassis_collision_type=chassis_collision_type)

vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc1, initRot1))
vehicle1.SetTireStepSize(tire_step_size)
vehicle1.Initialize()

vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)

# --------------------
# Create Vehicle 2
# --------------------
vehicle2 = veh.Sedan(system=my_system,
                     fixed=False,
                     tire_model=tire_model,
                     contact_method=contact_method,
                     chassis_collision_type=chassis_collision_type)

vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot2))
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()

vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)


# ------------------
# Create the terrain
# ------------------
# Terrain material (NSC specific)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(my_system) # Pass the shared system
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight - 0.1), chrono.QUNIT), # Lower terrain slightly for better vis
                         terrainLength, terrainWidth, 0.2) # Added thickness to patch

# Changed texture to concrete.jpg
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# -------------------
# Create Irrlicht App
# -------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan Demo - Dual Vehicle Sinusoidal Steering')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) # Camera will track vehicle1 by default
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()

# Attach both vehicles to the visualizer
vis.AttachVehicle(vehicle1.GetVehicle())
vis.AttachVehicle(vehicle2.GetVehicle())


# ---------------
# Simulation loop
# ---------------

# Output vehicle masses
print("VEHICLE 1 MASS: ", vehicle1.GetVehicle().GetMass())
print("VEHICLE 2 MASS: ", vehicle2.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

# Create driver input structures (to be populated in the loop)
driver_inputs_veh1 = veh.DriverInputs()
driver_inputs_veh2 = veh.DriverInputs()

while vis.Run():
    time = my_system.GetChTime()

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs (sinusoidal steering)
    # Vehicle 1
    driver_inputs_veh1.m_steering = steer_amplitude * math.sin(2 * math.pi * steer_frequency * time)
    driver_inputs_veh1.m_throttle = throttle_value
    driver_inputs_veh1.m_braking = braking_value

    # Vehicle 2 (e.g., with a phase shift in steering or different amplitude/frequency)
    driver_inputs_veh2.m_steering = steer_amplitude * math.sin(2 * math.pi * steer_frequency * time + math.pi) # 180 deg phase shift
    # driver_inputs_veh2.m_steering = -steer_amplitude * math.sin(2 * math.pi * steer_frequency * time) # Alternative: opposite steering
    driver_inputs_veh2.m_throttle = throttle_value
    driver_inputs_veh2.m_braking = braking_value

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time) # Synchronize terrain (e.g. if it's moving or deformable)
    vehicle1.Synchronize(time, driver_inputs_veh1, terrain) # Synchronize vehicle1
    vehicle2.Synchronize(time, driver_inputs_veh2, terrain) # Synchronize vehicle2
    vis.Synchronize(time, driver_inputs_veh1) # Synchronize visualization (can use inputs from one vehicle for HUD)

    # Advance simulation for one timestep for all modules
    # Crucially, advance vehicle subsystems first, then the main system step
    vehicle1.GetVehicle().Advance(step_size)  # Advances vehicle1 subsystems (tires, powertrain)
    vehicle2.GetVehicle().Advance(step_size)  # Advances vehicle2 subsystems
    terrain.Advance(step_size)                # Advance terrain state (if applicable)
    my_system.DoStepDynamics(step_size)       # Advance the full physical system
    vis.Advance(step_size)                    # Advance visualization

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

del my_system # Explicitly delete system (good practice, though Python's GC will handle it)