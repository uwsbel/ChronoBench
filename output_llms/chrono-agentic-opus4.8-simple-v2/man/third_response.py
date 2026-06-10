import os
import math
import random
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

time_step = 1e-3                                                     # integration step (s)
tire_step_size = time_step                                          # tire model sub-step
sim_end = 20.0                                                       # total simulated time (s)
init_loc = chrono.ChVector3d(0, 0, 0.5)                             # chassis spawn point
init_rot = chrono.QuatFromAngleZ(0)                                # facing +X

hmmwv = veh.HMMWV_Full()                                            # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision shell
hmmwv.SetChassisFixed(False)                                      # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                             # tire integration step
hmmwv.Initialize()                                                # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)  # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)    # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)     # tire mesh

system = hmmwv.GetSystem()                                        # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())            # report total vehicle mass

terrain = veh.RigidTerrain(system)                               # flat rigid ground under the vehicle
patch_mat = chrono.ChContactMaterialNSC()                       # NSC patch material
patch_mat.SetFriction(0.9)                                      # tire grip
patch_mat.SetRestitution(0.01)                                  # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200 x 200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass texture (was tile4.jpg)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                  # base tint
terrain.Initialize()                                           # build terrain bodies

box_mat = chrono.ChContactMaterialNSC()                        # contact material for the random boxes
box_mat.SetFriction(0.9)                                       # box-ground friction
box_mat.SetRestitution(0.01)                                   # nearly inelastic boxes
random.seed(0)                                                 # deterministic box layout
num_boxes = 25                                                 # number of random scattered boxes
for i in range(num_boxes):                                     # scatter boxes around the drive path
    size = random.uniform(0.5, 1.5)                            # cube edge length (m)
    px = random.uniform(-30.0, 60.0)                           # random X in the drive corridor
    py = random.uniform(-30.0, 30.0)                           # random Y across the field
    box = chrono.ChBodyEasyBox(size, size, size, 1000, True, True, box_mat)  # visualize + collide
    box.SetPos(chrono.ChVector3d(px, py, size / 2.0))          # rest on the ground plane
    box.SetFixed(True)                                         # static obstacle for the lidar to see
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.6))  # bluish boxes
    system.Add(box)                                            # add the obstacle to the scene

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()              # vehicle-aware Irrlicht window
vis.SetWindowTitle("HMMWV Lidar Scene")                      # window title
vis.SetWindowSize(1280, 1024)                                # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)  # follow the chassis
vis.Initialize()                                             # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo (after Initialize)
vis.AddSkyBox()                                              # sky backdrop
vis.AddLightDirectional()                                    # directional light (vehicle-truth style)
vis.AttachVehicle(hmmwv.GetVehicle())                        # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                     # interactive driver (truth default)
steering_time = 1.0                                          # s to full steering
throttle_time = 1.0                                          # s to full throttle
braking_time = 0.3                                           # s to full brake
driver.SetSteeringDelta(time_step / steering_time)          # steering ramp rate
driver.SetThrottleDelta(time_step / throttle_time)          # throttle ramp rate
driver.SetBrakingDelta(time_step / braking_time)            # braking ramp rate
driver.Initialize()                                         # build the driver

manager = sens.ChSensorManager(system)                      # oversee all sensors
chassis_body = hmmwv.GetChassisBody()                       # body the lidar rides on

horizontal_samples = 800                                    # lidar horizontal beam count
vertical_samples = 300                                      # lidar vertical beam count
update_rate = 5.0                                           # lidar update rate (Hz)
offset_pose = chrono.ChFramed(                              # mount the lidar above the chassis
    chrono.ChVector3d(-1.0, 0, 1.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,                                          # attach body
    update_rate,                                          # update_rate (Hz) — physical rate
    offset_pose,                                          # offset pose on the chassis
    horizontal_samples,                                  # h_samples
    vertical_samples,                                    # v_samples
    2 * chrono.CH_PI,                                    # horizontal_fov (rad) — full 360
    chrono.CH_PI / 12,                                  # max_vert_angle (rad)
    -chrono.CH_PI / 6,                                  # min_vert_angle (rad)
    100.0,                                              # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,                    # beam shape
    2,                                                  # sample_radius
    0.003,                                              # vertical divergence_angle
    0.003,                                              # horizontal divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,              # return mode
)
lidar.SetName("Lidar Sensor")                            # sensor name
lidar.SetLag(0)                                          # no lag
lidar.SetCollectionWindow(1.0 / update_rate)            # collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # raw depth preview
lidar.PushFilter(sens.ChFilterDIAccess())              # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())          # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())           # host access to XYZI
manager.AddSensor(lidar)                               # register the lidar (after all filters)

render_step_size = 1.0 / 50.0                           # 50 Hz render cadence
render_every = max(1, round(render_step_size / time_step))  # untagged cadence constant
realtime_timer = chrono.ChRealtimeStepTimer()           # keep wall-clock in sync with sim time
while vis.Run() and system.GetChTime() < sim_end:       # main real-time loop
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):                       # advance physics between frames
        sim_time = system.GetChTime()                   # current simulation time
        driver_inputs = driver.GetInputs()              # current driver command
        driver.Synchronize(sim_time)                    # sync driver
        terrain.Synchronize(sim_time)                   # sync terrain
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)  # sync vehicle with terrain
        vis.Synchronize(sim_time, driver_inputs)        # sync visualization
        driver.Advance(time_step)                       # advance driver
        terrain.Advance(time_step)                      # advance terrain
        hmmwv.Advance(time_step)                         # advance vehicle (steps the system)
        vis.Advance(time_step)                           # advance visualization
        manager.Update()                                # pump the lidar once per physics step
        di_buffer = lidar.GetMostRecentDIBuffer()       # most recent depth+intensity buffer
        if di_buffer.HasData():                          # only read after the first lidar tick
            print('Lidar buffer. Beams: {0}x{1}'.format(di_buffer.Width, di_buffer.Height))
        if system.GetChTime() >= sim_end:
            break
    realtime_timer.Spin(time_step)                       # spin so wall-clock matches sim time
