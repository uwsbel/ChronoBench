import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 2e-3                                                      # physics time step
tire_step_size = 1e-3                                                 # tire sub-step

init_loc = chrono.ChVector3d(0, -5, 0.4)                             # vehicle spawn (prompt: y = -5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # no initial heading rotation

hmmwv = veh.HMMWV_Full()                                              # catalog HMMWV wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
hmmwv.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                               # tire integration step
hmmwv.Initialize()                                                   # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)    # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)      # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)       # tire mesh

system = hmmwv.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid flat terrain
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                          # terrain friction
patch_mat.SetRestitution(0.01)                                     # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0) # 100 x 100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # terrain color
terrain.Initialize()                                                # finalize terrain

box_mat = chrono.ChContactMaterialNSC()                             # box contact material
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, box_mat)     # 1 x 1 x 1 box prop
box.SetPos(chrono.ChVector3d(0, 0, 0.5))                           # box position (prompt)
box.SetFixed(True)                                                  # box is a static prop
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # blue texture
system.AddBody(box)                                                 # register the box

cyl_mat = chrono.ChContactMaterialNSC()                            # cylinder contact material
cyl = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.5, 1, 1000, True, True, cyl_mat)  # r=0.5, h=1
cyl.SetPos(chrono.ChVector3d(0, 0, 1.5))                          # cylinder position (prompt)
cyl.SetFixed(True)                                                 # cylinder is a static prop
cyl.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # blue texture
system.AddBody(cyl)                                                # register the cylinder

manager = sens.ChSensorManager(system)                            # sensor manager on the vehicle system

lidar_offset = chrono.ChFramed(                                    # lidar offset pose on the chassis
    chrono.ChVector3d(0.0, 0, 2),                                 # 2 m above the chassis origin
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),     # no extra rotation
)
lidar = sens.ChLidarSensor(
    hmmwv.GetChassisBody(),                                       # rides on the vehicle chassis
    5.0,                                                          # update rate (Hz)
    lidar_offset,                                                 # offset pose
    800,                                                          # horizontal samples (prompt)
    300,                                                          # vertical channels (prompt)
    2 * chrono.CH_PI,                                            # 360 deg horizontal FOV (prompt)
    chrono.CH_PI / 12,                                           # max vertical angle (prompt)
    -chrono.CH_PI / 6,                                           # min vertical angle (prompt)
    100.0,                                                       # max range (prompt)
    sens.LidarBeamShape_RECTANGULAR,                            # rectangular beam (prompt)
    2,                                                           # sample radius (prompt)
    0.003,                                                       # vertical divergence angle (prompt)
    0.003,                                                       # horizontal divergence angle (prompt)
    sens.LidarReturnMode_STRONGEST_RETURN,                     # strongest return (prompt)
)
lidar.SetName("Lidar Sensor")                                    # sensor name
lidar.SetLag(0)                                                  # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                            # collection window = 1 / update_rate

lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))   # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                       # depth + intensity host access
lidar.PushFilter(sens.ChFilterPCfromDepth())                   # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # XYZI viz
lidar.PushFilter(sens.ChFilterXYZIAccess())                    # XYZI host access
manager.AddSensor(lidar)                                        # register the lidar

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()               # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV Lidar Scene")                        # window title
vis.SetWindowSize(1280, 1024)                                 # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)  # chase camera on chassis
vis.Initialize()                                              # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                              # sky box
vis.AddLightDirectional()                                   # directional light (vehicle truth)
vis.AttachVehicle(hmmwv.GetVehicle())                       # bind vehicle visual assets

driver_inputs = veh.DriverInputs()                          # scripted driver inputs (prompt)
driver_inputs.m_steering = 0.5                              # steering 0.5 (prompt)
driver_inputs.m_throttle = 0.2                              # throttle 0.2 (prompt)
driver_inputs.m_braking = 0.0                               # no braking

render_step_size = 1.0 / 50.0                               # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)     # physics steps per rendered frame
sim_end = 10.0                                              # simulation duration


realtime_timer = chrono.ChRealtimeStepTimer()              # wall-clock pacing
step_number = 0                                            # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                             # current sim time

    if step_number % render_steps == 0:                   # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs.m_steering = 0.5                        # hold steering at 0.5 (prompt)
    driver_inputs.m_throttle = 0.2                        # hold throttle at 0.2 (prompt)

    terrain.Synchronize(time)                             # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)       # sync vehicle with driver + terrain
    vis.Synchronize(time, driver_inputs)                  # sync visual system

    terrain.Advance(step_size)                            # advance terrain
    hmmwv.Advance(step_size)                              # advance vehicle (steps the system)
    vis.Advance(step_size)                                # advance visual system

    manager.Update()                                      # pump sensors once per step


    step_number += 1                                      # advance counter
    realtime_timer.Spin(step_size)                        # spin in place to match wall clock
