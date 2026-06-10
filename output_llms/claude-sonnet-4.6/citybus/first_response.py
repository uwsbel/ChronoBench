import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# =============================================================================
# Initial vehicle parameters
# =============================================================================

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (NONE, PRIMITIVES, HULLS, MESH)
chassis_collision_type = veh.CollisionType_NONE

# Tire model (RIGID, TMEASY, PAC89, PAC02, FIALA, ANCF, REISSNER)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain dimensions
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# =============================================================================
# Create the CityBus vehicle
# =============================================================================

# Create the vehicle system
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(chassis_collision_type)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
bus.SetTireType(tire_model)
bus.SetTireStepSize(tire_step_size)

# Initialize the vehicle
bus.Initialize()

# Set visualization types for vehicle subsystems
bus.SetChassisVisualizationType(chassis_vis_type)
bus.SetSuspensionVisualizationType(suspension_vis_type)
bus.SetSteeringVisualizationType(steering_vis_type)
bus.SetWheelVisualizationType(wheel_vis_type)
bus.SetTireVisualizationType(tire_vis_type)

# =============================================================================
# Create the terrain
# =============================================================================

terrain = veh.RigidTerrain(bus.GetSystem())

# Create a terrain patch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.CSYSNORM.rot),
                         terrainLength, terrainWidth)

# Set terrain texture
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Initialize terrain
terrain.Initialize()

# =============================================================================
# Create the vehicle Irrlicht interface
# =============================================================================

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('CityBus Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 14.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(bus.GetVehicle())

# =============================================================================
# Create the interactive driver system
# =============================================================================

driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# =============================================================================
# Simulation loop
# =============================================================================

# Output vehicle mass
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

print("Starting simulation...")

while vis.Run():
    time = bus.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (order is important!)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    bus.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    bus.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

print("Simulation complete!")