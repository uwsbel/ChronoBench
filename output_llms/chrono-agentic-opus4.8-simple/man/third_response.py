import math                                                           # render-cadence / placement math
import random                                                         # random box placement
import pychrono.core as chrono                                        # core PyChrono
import pychrono.vehicle as veh                                        # wheeled vehicle catalog
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer (window view)
import pychrono.sensor as sens                                        # OptiX sensor framework (lidar)
import numpy as np                                                    # numeric buffer access

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # vehicle spawn (X, Y, Z) world frame
init_rot = chrono.QuatFromAngleZ(0)                                 # facing +X, no yaw
step_size = 2e-3                                                     # physics step (s)
tire_step_size = 1e-3                                                # tire substep (s)

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                       # MANDATORY — chassis must be free
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()                                                 # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)   # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)     # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)      # tire mesh

system = hmmwv.GetSystem()                                          # take ownership of the vehicle system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED contact scene
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch material
patch_mat.SetFriction(0.9)                                         # tire grip
patch_mat.SetRestitution(0.01)                                     # nearly inelastic ground
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100 x 100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                      # base patch tint
terrain.Initialize()                                               # finalize terrain

box_mat = chrono.ChContactMaterialNSC()                            # contact material for the boxes
box_mat.SetFriction(0.9)                                           # box friction
random.seed(0)                                                     # deterministic box layout
for i in range(20):                                               # scatter 20 random boxes
    size = random.uniform(0.5, 1.5)                               # cube edge length (m)
    px = random.uniform(-40.0, 40.0)                             # random X within the patch
    py = random.uniform(-40.0, 40.0)                             # random Y within the patch
    if abs(px) < 6.0 and abs(py) < 4.0:                          # keep the spawn lane clear
        continue
    box = chrono.ChBodyEasyBox(size, size, size, 1000, True, True, box_mat)  # visible + colliding box
    box.SetPos(chrono.ChVector3d(px, py, size / 2.0))           # rest on the ground
    box.SetFixed(True)                                          # static obstacles for the lidar
    system.AddBody(box)                                        # add to the scene

manager = sens.ChSensorManager(system)                            # oversee all sensors

offset_pose = chrono.ChFramed(                                     # lidar mount on the chassis
    chrono.ChVector3d(-8, 0, 1),                                  # behind/above the chassis origin
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),     # no tilt
)
horizontal_samples = 800                                          # lidar azimuth samples
vertical_samples = 300                                            # lidar elevation samples
update_rate = 5.0                                                # lidar update rate (Hz)
lidar = sens.ChLidarSensor(
    hmmwv.GetChassisBody(),                                      # attach to the chassis
    update_rate,                                                # physical update rate (Hz)
    offset_pose,                                                # mount pose on the chassis
    horizontal_samples,                                        # horizontal samples
    vertical_samples,                                          # vertical samples
    2 * chrono.CH_PI,                                          # 360 deg horizontal FOV
    chrono.CH_PI / 12,                                        # max vertical angle
    -chrono.CH_PI / 6,                                        # min vertical angle
    100.0,                                                    # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                          # rectangular beam
    2,                                                        # sample radius
    0.003,                                                    # vertical divergence angle
    0.003,                                                    # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                    # strongest-return mode
)
lidar.SetName("Lidar Sensor")                                    # sensor name
lidar.SetLag(0)                                                  # no lag
lidar.SetCollectionWindow(1.0 / update_rate)                    # collection window = 1 / rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                       # host access to depth + intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                    # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                     # host access to XYZI cloud
manager.AddSensor(lidar)                                        # register the lidar

render_step_size = 1.0 / 50.0                                     # render cadence target (s)
render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV Lidar Scene")                          # window title
vis.SetWindowSize(1280, 720)                                     # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)     # chase camera on the chassis
vis.Initialize()                                                # build the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo (after Initialize)
vis.AddSkyBox()                                                 # sky box (after Initialize)
vis.AddLightDirectional()                                       # directional light (vehicle truth shape)
vis.AttachVehicle(hmmwv.GetVehicle())                           # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                         # interactive driver bound to the window
steering_time = 1.0                                             # s to full steering
throttle_time = 1.0                                             # s to full throttle
braking_time = 0.3                                             # s to full brake
driver.SetSteeringDelta(render_step_size / steering_time)       # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)       # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)         # braking ramp rate
driver.Initialize()                                            # finalize the driver

sim_end = 12.0                                                  # simulation horizon (s)
realtime_timer = chrono.ChRealtimeStepTimer()                  # wall-clock pacing
step_number = 0                                               # step counter for render cadence
log_lidar = True                                              # fire the lidar diagnostic once
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                # current sim time

    if step_number % render_steps == 0:                       # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                       # current driver inputs

    driver.Synchronize(time)                                 # sync driver
    terrain.Synchronize(time)                                # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)          # sync vehicle with inputs + terrain
    vis.Synchronize(time, driver_inputs)                     # sync visual system

    driver.Advance(step_size)                                # advance driver
    terrain.Advance(step_size)                               # advance terrain
    hmmwv.Advance(step_size)                                 # advance vehicle (steps the system)
    vis.Advance(step_size)                                   # advance visualization

    manager.Update()                                         # pump the lidar once per step

    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()            # latest point-cloud buffer
    if log_lidar and xyzi_buffer.HasData():                  # only read after first lidar tick
        xyzi = xyzi_buffer.GetXYZIData()                     # numpy view of XYZI returns
        print("Lidar buffer received. Points: ", xyzi.shape[0])  # report number of returns
        log_lidar = False                                    # disable further logging


    step_number += 1                                         # advance step counter
    realtime_timer.Spin(step_size)                           # pace to wall clock
