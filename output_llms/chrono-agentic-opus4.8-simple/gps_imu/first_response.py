import math                                                           # render cadence + loop math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 1e-3                                                      # physics step (s)
sim_end = 20.0                                                        # simulation duration (s)
init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn (HMMWV origin ~0.5 m up)
init_rot = chrono.QUNIT                                              # no initial yaw

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision shape
hmmwv.SetChassisFixed(False)                                        # MANDATORY — chassis must be free
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMeasy tire on rigid terrain
hmmwv.SetTireStepSize(step_size)                                   # tire sub-step
hmmwv.Initialize()                                                  # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheels mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)        # tires mesh

system = hmmwv.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # flat rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
patch_mat.SetFriction(0.9)                                         # terrain friction
patch_mat.SetRestitution(0.01)                                     # terrain bounciness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # terrain color
terrain.Initialize()                                               # build terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-aware Irrlicht window
vis.SetWindowTitle("HMMWV GPS/IMU Sensors")                        # window title (before Initialize)
vis.SetWindowSize(1280, 1024)                                      # window size (before Initialize)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)       # chase camera on chassis
vis.Initialize()                                                  # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo (after Initialize)
vis.AddSkyBox()                                                    # sky box (after Initialize)
vis.AddLightDirectional()                                         # vehicle scenes use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                             # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive driver bound to vis
render_step_size = 1.0 / 50.0                                     # 50 fps interactive cadence
driver.SetSteeringDelta(render_step_size / 1.0)                  # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                 # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                  # braking rate
driver.Initialize()                                              # build the driver

chassis_body = hmmwv.GetChassisBody()                            # body the sensors ride on
manager = sens.ChSensorManager(system)                          # oversee all sensors

offset_pose = chrono.ChFramed(                                   # sensor mount on the chassis frame
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

imu = sens.ChAccelerometerSensor(chassis_body, 100, offset_pose, sens.ChNoiseNone())  # IMU at 100 Hz
imu.SetName("IMU Sensor")                                        # sensor name
imu.SetLag(0)                                                    # no lag
imu.SetCollectionWindow(0)                                       # instantaneous collection
imu.PushFilter(sens.ChFilterAccelAccess())                      # host access to acceleration buffer
manager.AddSensor(imu)                                           # register IMU

gps = sens.ChGPSSensor(                                          # GPS sensor on the chassis
    chassis_body, 100, offset_pose,
    chrono.ChVector3d(-89.400, 43.070, 260.0),                  # reference lat/lon/alt origin
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")                                        # sensor name
gps.SetLag(0)                                                    # no lag
gps.SetCollectionWindow(0)                                       # instantaneous collection
gps.PushFilter(sens.ChFilterGPSAccess())                        # host access to GPS buffer
manager.AddSensor(gps)                                           # register GPS

render_steps = math.ceil(render_step_size / step_size)          # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                  # spin to match wall-clock to sim time
step_number = 0                                                  # physics step counter


while vis.Run():
    time = system.GetChTime()                                  # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                         # read driver commands

    driver.Synchronize(time)                                   # update driver
    terrain.Synchronize(time)                                  # update terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)            # update vehicle
    vis.Synchronize(time, driver_inputs)                       # update visuals

    driver.Advance(step_size)                                  # advance driver
    terrain.Advance(step_size)                                 # advance terrain
    hmmwv.Advance(step_size)                                   # advance vehicle (steps the system)
    vis.Advance(step_size)                                     # advance visuals

    manager.Update()                                           # update GPS + IMU each step

    acc_buf = imu.GetMostRecentAccelBuffer()                   # latest accelerometer reading
    if acc_buf.HasData():                                      # only after the first IMU tick
        acc = acc_buf.GetAccelData()                           # accelerometer reading (X, Y, Z)
    gps_buf = gps.GetMostRecentGPSBuffer()                     # latest GPS reading
    if gps_buf.HasData():                                      # only after the first GPS tick
        gps_data = gps_buf.GetGPSData()                        # (lat, lon, alt, t)

    step_number += 1                                           # advance step counter
    realtime_timer.Spin(step_size)                            # match wall clock to sim time

    if time >= sim_end:                                        # stop at the end of the run
        break
