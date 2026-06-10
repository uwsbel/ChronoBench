import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

time_step = 1e-3                                                      # integration step (s)
tire_step_size = time_step                                           # tire force-model step
sim_end = 8.0                                                         # total simulated time (s)

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn (m)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog vehicle
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                         # TMeasy tire on rigid road
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()                                                 # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)      # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)        # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)         # tire mesh

system = hmmwv.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch material
patch_mat.SetFriction(0.9)                                         # tire-road friction
patch_mat.SetRestitution(0.01)                                     # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100 x 100 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # sandy tint
terrain.Initialize()                                               # finalize terrain

manager = sens.ChSensorManager(system)                             # oversees GPS + IMU sensors

offset_pose = chrono.ChFramed(                                      # sensor mount on the chassis
    chrono.ChVector3d(-8, 0, 1),                                    # behind/above chassis origin
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

imu = sens.ChAccelerometerSensor(                                  # IMU accelerometer
    hmmwv.GetChassisBody(), 100, offset_pose, sens.ChNoiseNone())
imu.SetName("IMU Sensor")                                          # sensor label
imu.SetLag(0)                                                      # no measurement lag
imu.SetCollectionWindow(0)                                         # instantaneous sample
imu.PushFilter(sens.ChFilterAccelAccess())                        # host access to accel data
manager.AddSensor(imu)                                             # register IMU

gps = sens.ChGPSSensor(                                            # GPS receiver
    hmmwv.GetChassisBody(), 100, offset_pose,
    chrono.ChVector3d(-89.400, 43.070, 260.0),                    # reference lat/lon/alt origin
    sens.ChNoiseNone())
gps.SetName("GPS Sensor")                                          # sensor label
gps.SetLag(0)                                                      # no measurement lag
gps.SetCollectionWindow(0)                                         # instantaneous sample
gps.PushFilter(sens.ChFilterGPSAccess())                          # host access to GPS data
manager.AddSensor(gps)                                             # register GPS

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV GPS/IMU Sensor Demo")                   # window title
vis.SetWindowSize(1280, 720)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)      # chase camera on chassis
vis.Initialize()                                                 # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                  # sky box
vis.AddLightDirectional()                                        # vehicle demos use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind chassis/wheel visuals

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive driver bound to the vis
steering_time = 1.0                                              # s to reach full steering
throttle_time = 1.0                                              # s to reach full throttle
braking_time = 0.3                                               # s to reach full brake
render_step_size = 1.0 / 50.0                                    # render cadence (s)
driver.SetSteeringDelta(render_step_size / steering_time)        # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)          # braking ramp rate
driver.Initialize()                                             # finalize driver

render_every = max(1, round(1.0 / (50.0 * time_step)))          # untagged real-time cadence constant
log_step_size = 1.0 / 20.0                                      # GPS log interval (s) -> 20 Hz
log_steps = max(1, round(log_step_size / time_step))           # physics steps between GPS logs
gps_data = []                                                  # collected GPS coordinates

step_number = 0                                                # physics step counter

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()                              # current sim time

        driver_inputs = driver.GetInputs()                    # base interactive inputs
        if time < 2.0:                                         # spin up: accelerate straight
            driver_inputs.m_throttle = 0.7
            driver_inputs.m_steering = 0.0
            driver_inputs.m_braking = 0.0
        elif time < 6.0:                                       # mid: cruise and steer
            driver_inputs.m_throttle = 0.5
            driver_inputs.m_steering = 0.3
            driver_inputs.m_braking = 0.0
        else:                                                  # after 6 s: brake to a stop
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_steering = 0.0
            driver_inputs.m_braking = 0.8

        driver.Synchronize(time)                              # driver update
        terrain.Synchronize(time)                             # terrain update
        hmmwv.Synchronize(time, driver_inputs, terrain)       # vehicle gets driver + terrain
        vis.Synchronize(time, driver_inputs)                  # vis HUD update

        driver.Advance(time_step)                             # advance driver
        terrain.Advance(time_step)                            # advance terrain
        hmmwv.Advance(time_step)                              # advances the wrapper-owned system
        vis.Advance(time_step)                                # advance vis

        manager.Update()                                      # pump sensors once per physics step

        if step_number % log_steps == 0:                      # log GPS at the log interval
            gps_buffer = gps.GetMostRecentGPSBuffer()         # most recent GPS frame
            if gps_buffer.HasData():                          # only after the sensor's first tick
                gps_data.append(gps_buffer.GetGPSData())      # append (lat, lon, alt, time)

        step_number += 1                                      # advance step counter
        if system.GetChTime() >= sim_end:
            break

print("GPS Data: ", gps_data)                                 # final GPS log dump
