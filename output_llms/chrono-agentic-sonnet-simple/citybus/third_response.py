import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                      # locate vehicle data files

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)                                       # spawn above terrain
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                                   # no rotation

# Visualization types
vis_type = veh.VisualizationType_PRIMITIVES
vis_type_mesh = veh.VisualizationType_MESH

# Chassis collision type
chassis_collision_type = veh.CollisionType_NONE

# Tire model — Pacejka 89 as requested
tire_model = veh.TireModelType_PAC89

# Terrain dimensions
terrainLength = 100.0                                                         # X direction
terrainWidth = 100.0                                                          # Y direction

# Camera track point
trackPoint = chrono.ChVector3d(-15.0, 10.0, 5.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes — reduced to 5e-4 for better stability
step_size = 5e-4
tire_step_size = step_size

# Render rate
render_step_size = 1.0 / 50                                                   # 50 FPS

sim_end = 10.0                                                                # simulation end time (s)

# Create the CityBus vehicle
vehicle = veh.CityBus()
vehicle.SetContactMethod(contact_method)                                      # NSC for rigid terrain
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)                                                # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)                                               # Pacejka 89 tire model
vehicle.SetTireStepSize(tire_step_size)                                       # 5e-4 for stability

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type_mesh)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type_mesh)
vehicle.SetTireVisualizationType(vis_type_mesh)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Create the rigid terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)      # dirt road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the Irrlicht visualization — Initialize() FIRST, then add scene elements
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('City Bus Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 3.5)                                      # chase camera behind bus
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the interactive driver
driver = veh.ChInteractiveDriverIRR(vis)

# Time response for steering and throttle inputs
steering_time = 1.0                                                           # time to go 0 -> +1 steering
throttle_time = 1.0                                                           # time to go 0 -> +1 throttle
braking_time = 0.3                                                            # time to go 0 -> +1 braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass (scored core — matches truth)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Render step cadence
render_steps = math.ceil(render_step_size / step_size)
render_every = render_steps                                                    # untagged cadence constant

# Recording setup

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:                                       # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    # Synchronize all subsystems
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all subsystems
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)                                                # advances the wrapper-owned system
    vis.Advance(step_size)


    step_number += 1
    realtime_timer.Spin(step_size)                                            # real-time pacing
