import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set the path to the Chrono data files
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_height = 0      # terrain height
terrain_height = 0.1      # terrain height
terrain_length = 100.0    # size in X direction
terrain_width = 100.0     # size in Y direction

# Create the terrain patches with different textures
patch_mat1 = chrono.ChContactMaterialNSC()
patch_mat1.SetFriction(0.9)
patch_mat1.SetRestitution(0.01)

patch_mat2 = chrono.ChContactMaterialNSC()
patch_mat2.SetFriction(0.8)
patch_mat2.SetRestitution(0.02)

patch_mat3 = chrono.ChContactMaterialNSC()
patch_mat3.SetFriction(0.7)
patch_mat3.SetRestitution(0.03)

patch_mat4 = chrono.ChContactMaterialNSC()
patch_mat4.SetFriction(0.6)
patch_mat4.SetRestitution(0.04)

# Create the terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Add patches to the terrain
terrain.AddPatch(patch_mat1, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrain_length, terrain_width, terrain_height)

terrain.AddPatch(patch_mat2, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrain_length, terrain_width, terrain_height)

terrain.AddPatch(patch_mat3, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrain_length, terrain_width, terrain_height)

terrain.AddPatch(patch_mat4, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrain_length, terrain_width, terrain_height)

# Adding a bump to the terrain
terrain.AddPatch(patch_mat1, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrain_length, terrain_width, terrain_height)

# Create the Gator vehicle, set parameters, and initialize
vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(1e-3)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(1.0 / steering_time)
driver.SetThrottleDelta(1.0 / throttle_time)
driver.SetBrakingDelta(1.0 / braking_time)

driver.Initialize()

# Simulation loop
render_steps = math.ceil(1.0 / 50)  # FPS = 50

step_number = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        step_number += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(1.0 / 50)
    terrain.Advance(1.0 / 50)
    vehicle.Advance(1.0 / 50)
    vis.Advance(1.0 / 50)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    chrono.ChRealtimer().Step(1.0 / 50)