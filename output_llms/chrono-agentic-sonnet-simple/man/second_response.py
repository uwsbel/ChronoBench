import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-20, 0, 1.5)                      # spawn at (-20, 0, 1.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                    # no rotation

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH                          # mesh visualization

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE                # no chassis collision

# Tire model
tire_model = veh.TireModelType_TMEASY                          # TMEASY for rigid terrain

# Terrain parameters
terrainLength = 100.0                                          # size in X direction
terrainWidth = 100.0                                           # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)               # point on chassis to track

# Contact method
contact_method = chrono.ChContactMethod_NSC                    # NSC for rigid terrain

# Simulation step sizes
step_size = 1e-3                                               # physics step (s)
tire_step_size = step_size                                     # tire sub-step

# Render frame rate
render_step_size = 1.0 / 50                                   # 50 FPS rendering
render_steps = math.ceil(render_step_size / step_size)        # steps per render frame

# Create the MAN 5t vehicle, set parameters, and initialize
vehicle = veh.MAN_5t()                                         # MAN_5t — changed from MAN_10t
vehicle.SetContactMethod(contact_method)                       # NSC contact
vehicle.SetChassisCollisionType(chassis_collision_type)        # no chassis collision
vehicle.SetChassisFixed(False)                                 # MANDATORY — must move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot)) # initial pose
vehicle.SetTireType(tire_model)                                # TMEASY tires
vehicle.SetTireStepSize(tire_step_size)                        # tire step size

vehicle.Initialize()                                           # initialize vehicle

vehicle.SetChassisVisualizationType(vis_type)                 # mesh chassis
vehicle.SetSuspensionVisualizationType(vis_type)              # mesh suspension
vehicle.SetSteeringVisualizationType(vis_type)                # mesh steering
vehicle.SetWheelVisualizationType(vis_type)                   # mesh wheels
vehicle.SetTireVisualizationType(vis_type)                    # mesh tires

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED

# Create the rigid terrain of hills using a height map
patch_mat = chrono.ChContactMaterialNSC()                      # NSC material for rigid terrain
patch_mat.SetFriction(0.9)                                     # friction coefficient
patch_mat.SetRestitution(0.01)                                 # low restitution
terrain = veh.RigidTerrain(vehicle.GetSystem())                # rigid terrain
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),         # height map for hills
    64.0, 64.0, 0.0, 3.0,                                      # length, width, hMin, hMax
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6, 6)  # grass texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                  # terrain color hint
terrain.Initialize()                                           # finalize terrain

# Create the vehicle Irrlicht interface (Initialize FIRST, then add scene elements)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()               # wheeled vehicle vis
vis.SetWindowTitle('MAN 5t Demo')                              # window title
vis.SetWindowSize(1280, 1024)                                  # window size
vis.SetChaseCamera(trackPoint, 15.0, 0.5)                     # chase camera
vis.Initialize()                                               # MUST initialize first
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Chrono logo
vis.AddLightDirectional()                                      # directional light
vis.AddSkyBox()                                                # sky box
vis.AttachVehicle(vehicle.GetVehicle())                        # attach vehicle to vis

# Create the interactive driver
driver = veh.ChInteractiveDriverIRR(vis)                       # keyboard driver

steering_time = 1.0                                            # s to go 0 -> +1 steering
throttle_time = 1.0                                            # s to go 0 -> +1 throttle
braking_time = 0.3                                             # s to go 0 -> +1 braking
driver.SetSteeringDelta(render_step_size / steering_time)      # steering ramp
driver.SetThrottleDelta(render_step_size / throttle_time)      # throttle ramp
driver.SetBrakingDelta(render_step_size / braking_time)        # braking ramp

driver.Initialize()                                            # init driver

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())        # scored: vehicle mass banner

# Recording setup

realtime_timer = chrono.ChRealtimeStepTimer()                  # real-time pacing timer
step_number = 0                                                # step counter

while vis.Run():
    time = vehicle.GetSystem().GetChTime()                     # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                        # get driver inputs


    driver.Synchronize(time)                                   # sync driver
    terrain.Synchronize(time)                                  # sync terrain
    vehicle.Synchronize(time, driver_inputs, terrain)          # sync vehicle
    vis.Synchronize(time, driver_inputs)                       # sync vis

    driver.Advance(step_size)                                  # advance driver
    terrain.Advance(step_size)                                 # advance terrain
    vehicle.Advance(step_size)                                 # advance vehicle (steps system)
    vis.Advance(step_size)                                     # advance vis

    step_number += 1                                           # increment step counter
    realtime_timer.Spin(step_size)                            # real-time pacing
