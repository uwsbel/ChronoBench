import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# visualization type
vis_type = veh.VisualizationType_MESH

# collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# tire model
tire_model = veh.TireModelType_TMEASY

# terrain size
terrainLength = 100.0                    # size in X direction
terrainWidth = 100.0                     # size in Y direction

# camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# contact method
contact_method = chrono.ChContactMethod_NSC

# simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# render step size
render_step_size = 1.0 / 50              # FPS = 50
render_every = max(1, round(render_step_size / step_size))  # render cadence

# create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)                           # chassis must be free
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# create rigid terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# create the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                                      # time to go 0 to +1 steering
throttle_time = 1.0                                      # time to go 0 to +1 throttle
braking_time = 0.3                                       # time to go 0 to +1 braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# initialize sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# offset pose for sensors attached to the chassis
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

# create an IMU sensor and add it to the manager
imu = sens.ChAccelerometerSensor(
    vehicle.GetChassisBody(),            # body IMU is attached to
    10,                                  # update rate in Hz
    offset_pose,                         # offset pose
    sens.ChNoiseNone(),                  # noise model
)
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())  # host access to IMU data (scored core)
manager.AddSensor(imu)

# create a GPS sensor and add it to the manager
gps = sens.ChGPSSensor(
    vehicle.GetChassisBody(),            # body GPS is attached to
    10,                                  # update rate in Hz
    offset_pose,                         # offset pose
    chrono.ChVector3d(-89.400, 43.070, 260.0),  # GPS reference point (lat/lon/alt)
    sens.ChNoiseNone(),                  # noise model
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())    # host access to GPS data (scored core)
manager.AddSensor(gps)

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

# review-only recording setup

# simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()        # real-time step timer
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()           # current simulation time

    if step_number % render_steps == 0:             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()              # get driver inputs

    # synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # advance simulation for one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # update sensor manager each step
    manager.Update()


    step_number += 1
    realtime_timer.Spin(step_size)                   # real-time pacing
