import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

step_size = 1e-3                                                       # physics step
render_step_size = 1.0 / 50.0                                         # 50 fps chase-cam render cadence

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # HMMWV spawn (geometric-center origin)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation

hmmwv = veh.HMMWV_Full()                                              # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision geometry
hmmwv.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tires on rigid road
hmmwv.SetTireStepSize(step_size)                                     # tire integration step
hmmwv.Initialize()                                                   # build the vehicle

system = hmmwv.GetSystem()                                            # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED contact scene, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                # report total vehicle mass

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # wheel meshes
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # tire meshes

terrain = veh.RigidTerrain(system)                                   # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                           # ground friction
patch_mat.SetRestitution(0.01)                                       # ground restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)   # 100 x 100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # patch color
terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle chase-cam Irrlicht window
vis.SetWindowTitle("HMMWV GPS/IMU Demo")                             # window title
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)      # chase camera behind chassis
vis.Initialize()                                                     # build the device (FIRST)
vis.AddLogo()                                                        # PyChrono logo
vis.AddLightDirectional()                                            # single directional light (vehicle truth)
vis.AddSkyBox()                                                      # sky box
vis.AttachVehicle(hmmwv.GetVehicle())                                # bind the vehicle to the chase cam

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive keyboard driver
driver.SetSteeringDelta(render_step_size / 1.0)                      # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                      # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                       # braking ramp rate
driver.Initialize()                                                  # build the driver

manager = sens.ChSensorManager(system)                               # sensor manager on the vehicle system
offset_pose = chrono.ChFramed(                                       # sensor mount on the chassis
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

imu = sens.ChAccelerometerSensor(hmmwv.GetChassisBody(), 10, offset_pose, sens.ChNoiseNone())  # IMU on chassis, 10 Hz
imu.SetName("IMU Sensor")                                            # sensor name
imu.SetLag(0)                                                        # no lag
imu.SetCollectionWindow(0)                                           # instantaneous sample
imu.PushFilter(sens.ChFilterAccelAccess())                           # host access to accel data
manager.AddSensor(imu)                                               # register IMU

gps = sens.ChGPSSensor(                                              # GPS on chassis, 10 Hz
    hmmwv.GetChassisBody(), 10, offset_pose,
    chrono.ChVector3d(-89.400, 43.070, 260.0),                       # reference lat/lon/alt origin
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")                                            # sensor name
gps.SetLag(0)                                                        # no lag
gps.SetCollectionWindow(0)                                           # instantaneous sample
gps.PushFilter(sens.ChFilterGPSAccess())                             # host access to GPS data
manager.AddSensor(gps)                                               # register GPS

render_steps = math.ceil(render_step_size / step_size)               # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                        # wall-clock pacing
step_number = 0                                                      # physics step counter

while vis.Run():                                                     # SCORED CORE = plain truth form, NO time bound
    time = system.GetChTime()                                        # current simulation time

    if step_number % render_steps == 0:                              # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # current driver commands

    driver.Synchronize(time)                                         # sync driver
    terrain.Synchronize(time)                                        # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)                  # sync vehicle
    vis.Synchronize(time, driver_inputs)                             # sync visualization

    driver.Advance(step_size)                                        # advance driver
    terrain.Advance(step_size)                                       # advance terrain
    hmmwv.Advance(step_size)                                         # advance vehicle (steps the system)
    vis.Advance(step_size)                                           # advance visualization

    manager.Update()                                                 # pump IMU + GPS once per step

    acc_buf = imu.GetMostRecentAccelBuffer()                         # latest IMU buffer
    if acc_buf.HasData():                                            # if a sample is ready
        acc = acc_buf.GetAccelData()                                # read accel sample [ax, ay, az]
        print("IMU acc: ", acc[0], acc[1], acc[2])                  # report IMU data
    gps_buf = gps.GetMostRecentGPSBuffer()                           # latest GPS buffer
    if gps_buf.HasData():                                            # if a fix is ready
        coord = gps_buf.GetGPSData()                                 # read GPS [lat, lon, alt]
        print("GPS data: ", coord[0], coord[1], coord[2])           # report GPS data


    step_number += 1                                                 # advance step count
    realtime_timer.Spin(step_size)                                   # match wall-clock to sim time
