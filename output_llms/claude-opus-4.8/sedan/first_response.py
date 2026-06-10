# PyChrono BMW E90 Sedan Simulation
# Vehicle dynamics on rigid terrain with TMEASY tires and interactive driver

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# -------------------------------------------------------------------
# 1. Initialize PyChrono environment and data path
# -------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# -------------------------------------------------------------------
# Simulation parameters
# -------------------------------------------------------------------
# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type   = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type   = veh.VisualizationType_PRIMITIVES
wheel_vis_type      = veh.VisualizationType_MESH
tire_vis_type       = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model
tire_model = veh.TireModelType_TMEASY

# Rigid terrain dimensions
terrainHeight = 0
terrainLength = 100.0  # size in X direction
terrainWidth  = 100.0  # size in Y direction

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3

# -------------------------------------------------------------------
# 2. Create and configure the BMW E90 vehicle
# -------------------------------------------------------------------
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Apply visualization types
vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetSuspensionVisualizationType(suspension_vis_type)
vehicle.SetSteeringVisualizationType(steering_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)
vehicle.SetTireVisualizationType(tire_vis_type)

# -------------------------------------------------------------------
# 3. Create the rigid terrain
# -------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth)

# Customizable texture and color on the terrain
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# -------------------------------------------------------------------
# 4. Create the Irrlicht visualization with chase camera
# -------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('BMW E90 Sedan Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()

# Add scene elements: logo, skybox, lighting
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -------------------------------------------------------------------
# 5. Create the interactive driver system
# -------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Set time response for steering, throttle, and braking inputs
steering_time = 1.0   # time to go 0 -> 1 (or 0 -> -1)
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size := step_size / steering_time)
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)

driver.Initialize()

# -------------------------------------------------------------------
# 6. Simulation loop
# -------------------------------------------------------------------
# Output some vehicle information
print("Vehicle mass: ", vehicle.GetVehicle().GetMass())

vehicle.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Collect driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (synchronize)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance the simulation of all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)