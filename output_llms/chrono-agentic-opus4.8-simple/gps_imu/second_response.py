import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

time_step = 1e-3                                                       # integration step
tire_step_size = 1e-3                                                  # tire force step
sim_end = 10.0                                                        # total simulation time

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn (origin, on terrain)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation

hmmwv = veh.HMMWV_Full()                                              # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shape
hmmwv.SetChassisFixed(False)                                         # MANDATORY: chassis must move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                               # tire integration step
hmmwv.Initialize()                                                   # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid driving surface
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                          # tire-ground friction
patch_mat.SetRestitution(0.01)                                     # near-inelastic
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 200.0, 200.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                       # gray surface
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
terrain.Initialize()                                                # build the terrain

manager = sens.ChSensorManager(system)                              # oversee all sensors

offset_pose = chrono.ChFramed(                                       # sensor mount on the chassis
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

imu = sens.ChAccelerometerSensor(hmmwv.GetChassisBody(), 10, offset_pose, sens.ChNoiseNone())  # IMU @ 10 Hz
imu.SetName("IMU Sensor")                                            # name the IMU
imu.SetLag(0)                                                       # no lag
imu.SetCollectionWindow(0)                                          # instantaneous sample
imu.PushFilter(sens.ChFilterAccelAccess())                         # host access to accel data
manager.AddSensor(imu)                                              # register IMU

gps = sens.ChGPSSensor(                                              # GPS @ 10 Hz
    hmmwv.GetChassisBody(), 10, offset_pose,
    chrono.ChVector3d(-89.400, 43.070, 260.0),                     # reference lat/lon/alt
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")                                           # name the GPS
gps.SetLag(0)                                                       # no lag
gps.SetCollectionWindow(0)                                          # instantaneous sample
gps.PushFilter(sens.ChFilterGPSAccess())                           # host access to GPS data
manager.AddSensor(gps)                                              # register GPS

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht window
vis.SetWindowTitle("GPS/IMU Vehicle")                              # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera
vis.Initialize()                                                   # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                    # sky
vis.AddLightDirectional()                                          # directional light (vehicle truth)
vis.AttachVehicle(hmmwv.GetVehicle())                             # bind vehicle visuals

driver_data = veh.vector_Entry([                                   # scripted (time, steer, throttle, brake)
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                       # start at rest
    veh.DataDriverEntry(0.5, 0.0, 0.7, 0.0),                       # accelerate forward
    veh.DataDriverEntry(2.0, 0.3, 0.7, 0.0),                       # steer while driving
    veh.DataDriverEntry(6.0, 0.3, 0.0, 0.8),                       # brake after 6 s
    veh.DataDriverEntry(sim_end, 0.0, 0.0, 0.8),                   # hold brake to the end
])
driver = veh.ChDataDriver(hmmwv.GetVehicle(), driver_data)         # data-table driver
driver.Initialize()                                                # initialize the driver

render_step_size = 1.0 / 50.0                                       # 50 fps render cadence
render_every = max(1, round(render_step_size / time_step))         # untagged cadence constant

log_step_size = 1.0 / 20.0                                          # log GPS at 20 Hz
log_steps = max(1, round(log_step_size / time_step))               # steps between GPS logs
gps_data = []                                                       # collected GPS coordinates


step_number = 0                                                    # physics-step counter
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                              # begin frame
    vis.Render()                                                 # draw scene
    vis.EndScene()                                               # end frame
    for _ in range(render_every):
        time = system.GetChTime()                               # current sim time
        driver_inputs = driver.GetInputs()                      # current driver command
        driver.Synchronize(time)                                # sync driver
        terrain.Synchronize(time)                               # sync terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)         # sync vehicle
        vis.Synchronize(time, driver_inputs)                    # sync visuals
        driver.Advance(time_step)                               # advance driver
        terrain.Advance(time_step)                              # advance terrain
        hmmwv.Advance(time_step)                                # advance vehicle (steps the system)
        vis.Advance(time_step)                                  # advance visuals
        manager.Update()                                        # pump sensors once per step
        if step_number % log_steps == 0:                        # log GPS at the logging cadence
            gps_buffer = gps.GetMostRecentGPSBuffer()           # latest GPS buffer
            if gps_buffer.HasData():                            # only when filled
                gps_data.append(gps_buffer.GetGPSData())        # append [lon, lat, alt, time]
        step_number += 1                                        # next step
        if system.GetChTime() >= sim_end:
            break
print("GPS Data: ", gps_data)                                   # output the logged GPS coordinates
