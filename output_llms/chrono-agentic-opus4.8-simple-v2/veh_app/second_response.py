import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate bundled vehicle data files

initLoc = chrono.ChVector3d(0, -5, 0.4)                             # vehicle spawn location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation
step_size = 1e-3                                                     # integration step
tire_step_size = 1e-3                                               # tire substep
sim_end = 20.0                                                      # simulation duration (s)

hmmwv = veh.HMMWV_Full()                                            # full HMMWV catalog vehicle
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                       # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))        # place chassis at spawn
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMeasy tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                              # tire model substep
hmmwv.Initialize()                                                 # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh visuals
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh visuals
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh visuals

system = hmmwv.GetSystem()                                         # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # report total vehicle mass

terrain = veh.RigidTerrain(system)                                # rigid flat terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                         # NSC terrain material
patch_mat.SetFriction(0.9)                                        # tire grip
patch_mat.SetRestitution(0.01)                                    # near-inelastic ground
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100x100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # patch tint
terrain.Initialize()                                              # build terrain collision/visuals

box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)            # 1x1x1 box prop with collision
box.SetPos(chrono.ChVector3d(0, 0, 0.5))                          # placed ahead of the vehicle
box.SetFixed(True)                                                # static obstacle
box.GetVisualShape(0).SetTexture(veh.GetDataFile("terrain/textures/blue.png"))  # blue texture
system.AddBody(box)                                               # register the box body

cyl = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.5, 1, 1000, True, True)  # r=0.5 h=1 cylinder prop
cyl.SetPos(chrono.ChVector3d(0, 0, 1.5))                          # stacked above the box
cyl.SetFixed(True)                                                # static obstacle
cyl.GetVisualShape(0).SetTexture(veh.GetDataFile("terrain/textures/blue.png"))  # blue texture
system.AddBody(cyl)                                               # register the cylinder body

manager = sens.ChSensorManager(system)                            # sensor manager on the shared system
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),       # point light for sensor rendering
                            chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),       # second point light
                            chrono.ChColor(1, 1, 1), 500.0)

offset_pose = chrono.ChFramed(                                    # lidar mount pose on the chassis
    chrono.ChVector3d(0.0, 0, 2),                                 # 2 m above chassis origin
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),      # no tilt
)
lidar = sens.ChLidarSensor(
    hmmwv.GetChassisBody(),                                       # ride on the vehicle chassis
    5.0,                                                          # update_rate (Hz)
    offset_pose,                                                  # offset pose on the chassis
    800,                                                          # horizontal samples
    300,                                                          # vertical channels
    2 * chrono.CH_PI,                                             # 360 deg horizontal FOV
    chrono.CH_PI / 12,                                            # max vertical angle
    -chrono.CH_PI / 6,                                            # min vertical angle
    100.0,                                                        # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                             # rectangular beam shape
    2,                                                            # sample radius
    0.003,                                                        # vertical divergence angle
    0.003,                                                        # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                       # strongest return mode
)
lidar.SetName("Lidar Sensor")                                    # sensor name
lidar.SetLag(0)                                                  # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                            # collection window = 1 / update_rate

lidar.PushFilter(sens.ChFilterDIAccess())                       # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                   # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                    # host access to XYZI point cloud
manager.AddSensor(lidar)                                       # register the lidar

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                # vehicle-specific Irrlicht window
vis.SetWindowTitle("HMMWV with Lidar")                         # window title
vis.SetWindowSize(1280, 1024)                                  # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # chase camera on the chassis
vis.Initialize()                                              # create the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                              # sky box
vis.AddLightDirectional()                                   # directional light (vehicle truth idiom)
vis.AttachVehicle(hmmwv.GetVehicle())                      # bind vehicle visual assets

driver_inputs = veh.DriverInputs()                          # driver-input struct
driver_inputs.m_steering = 0.5                              # constant steering = 0.5
driver_inputs.m_throttle = 0.2                              # constant throttle = 0.2
driver_inputs.m_braking = 0.0                               # no braking

render_step_size = 1.0 / 50.0                              # render cadence (s)
render_steps = math.ceil(render_step_size / step_size)     # physics steps per rendered frame

realtime_timer = chrono.ChRealtimeStepTimer()             # spin the loop in real time
step_number = 0                                           # physics step counter
while vis.Run():
    time = system.GetChTime()                            # current sim time

    if step_number % render_steps == 0:                  # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs.m_steering = 0.5                       # steering held at 0.5 in the loop
    driver_inputs.m_throttle = 0.2                       # throttle held at 0.2 in the loop

    terrain.Synchronize(time)                            # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)     # sync vehicle with driver inputs + terrain
    vis.Synchronize(time, driver_inputs)                # sync visualization

    terrain.Advance(step_size)                           # advance terrain
    hmmwv.Advance(step_size)                             # advance vehicle (steps the system)
    vis.Advance(step_size)                               # advance visualization
    manager.Update()                                     # pump sensors once per step


    step_number += 1                                     # advance step counter
    realtime_timer.Spin(step_size)                       # match wall-clock to sim time
    if time >= sim_end:
        break
