import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

# vehicle initial location
init_loc = chrono.ChVector3d(0, -5, 0.4)                             # changed from (0,0,0.4) per prompt
init_rot = chrono.QuatFromAngleZ(0)                                  # facing +X

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                         # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tires
hmmwv.SetTireStepSize(1e-3)                                          # tire step size
hmmwv.Initialize()

system = hmmwv.GetSystem()                                           # vehicle owns the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # truth's literal banner

# rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                            # NSC material
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # large flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

blue_tex_path = chrono.GetChronoDataFile("textures/blue.png")         # blue texture path

# box object: 1x1x1 at (0, 0, 0.5), blue texture
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.6)
box_body = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000, True, True, box_mat)  # box 1m^3
box_body.SetPos(chrono.ChVector3d(0, 0, 0.5))                        # positioned per prompt
box_body.SetFixed(True)                                              # static scene prop
box_body.GetVisualShape(0).SetTexture(blue_tex_path)                 # apply blue texture
system.AddBody(box_body)

# cylinder object: radius=0.5, height=1 at (0, 0, 1.5), blue texture
cyl_mat = chrono.ChContactMaterialNSC()
cyl_mat.SetFriction(0.6)
cyl_body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.5, 1.0, 1000, True, True, cyl_mat)  # radius=0.5, h=1
cyl_body.SetPos(chrono.ChVector3d(0, 0, 1.5))                        # positioned per prompt
cyl_body.SetFixed(True)                                              # static scene prop
cyl_body.GetVisualShape(0).SetTexture(blue_tex_path)                 # apply blue texture
system.AddBody(cyl_body)

# vehicle visualization (ChWheeledVehicleVisualSystemIrrlicht)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Lidar Sensor")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)         # chase cam
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                            # vehicle truth uses directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# driver (interactive) — scripted inputs applied in loop per prompt
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)                                        # steering rate
driver.SetThrottleDelta(0.02)                                        # throttle rate
driver.SetBrakingDelta(0.06)                                         # braking rate
driver.Initialize()

# sensor manager — lidar does not need scene lights
manager = sens.ChSensorManager(system)

# lidar sensor on chassis at offset (0, 0, 2)
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),                                    # offset pose per prompt
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    hmmwv.GetChassisBody(),                                          # attach to chassis
    5.0,                                                             # update rate (Hz)
    lidar_offset,
    800,                                                             # 800 horizontal samples
    300,                                                             # 300 vertical channels
    2 * chrono.CH_PI,                                               # 360 deg horizontal FOV
    chrono.CH_PI / 12,                                              # max vertical angle
    -chrono.CH_PI / 6,                                              # min vertical angle
    100.0,                                                           # max range
    sens.LidarBeamShape_RECTANGULAR,                                 # rectangular beam shape
    2,                                                               # sample radius
    0.003,                                                           # divergence angle
    0.003,                                                           # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                           # strongest return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)                                 # 1 / update_rate

# lidar filter chain: depth+intensity, XYZI point cloud, visualization (scored core)
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))            # depth viz
lidar.PushFilter(sens.ChFilterDIAccess())                            # depth+intensity access
lidar.PushFilter(sens.ChFilterPCfromDepth())                         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point cloud viz
lidar.PushFilter(sens.ChFilterXYZIAccess())                          # XYZI host access
manager.AddSensor(lidar)

step_size = 1e-3                                                     # physics step (s)
sim_end = 15.0                                                       # simulation duration (s)
render_fps = 50.0
render_step_size = 1.0 / render_fps                                  # render interval (s)
render_steps = math.ceil(render_step_size / step_size)               # steps per render frame

step_number = 0

while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
    sim_time = hmmwv.GetSystem().GetChTime()

    if step_number % render_steps == 0:                              # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()
    driver_inputs.m_steering = 0.5                                   # scripted steering per prompt
    driver_inputs.m_throttle = 0.2                                   # scripted throttle per prompt
    driver_inputs.m_braking = 0.0                                    # no braking

    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    hmmwv.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)                                         # advances the wrapper-owned system
    vis.Advance(step_size)

    manager.Update()                                                 # update all sensors once per step


    step_number += 1

    if hmmwv.GetSystem().GetChTime() >= sim_end:
        break
