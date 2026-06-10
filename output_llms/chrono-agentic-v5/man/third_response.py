"""HMMWV on flat rigid terrain instrumented with an OptiX lidar sensor.

Models a full HMMWV wheeled vehicle (NSC contact) driving on a flat rigid-terrain
patch textured with grass. A ChSensorManager hosts a 2D-style lidar sensor mounted
on the chassis, and a field of randomly-placed rigid boxes populates the terrain so
the lidar returns a non-trivial depth/point-cloud scene. The Irrlicht window provides
the interactive review view; the lidar provides the sensed point cloud.

System type: NSC (rigid terrain catalog vehicle).
Main bodies: HMMWV chassis + wheels/spindles, rigid terrain patch, random box obstacles.
Expected behavior: the vehicle rests on (and can drive across) the grass terrain while
the lidar continuously scans the surrounding boxes; the manager updates every step.
"""

import math
import random

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 2e-3
sim_end = 12.0
render_fps = 50.0

terrain_length = 100.0
terrain_width = 100.0
init_loc = chrono.ChVector3d(0, 0, 0.5)          # chassis-origin spawn (HMMWV center origin)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

num_boxes = 30                                    # random obstacle field for the lidar
box_size = 1.0
box_area = 40.0                                   # boxes scattered within +/- box_area/2
lidar_update_rate = 5.0                           # physical Hz for the lidar
lidar_h_samples = 800
lidar_v_samples = 300

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV catalog wrapper (owns its own NSC ChSystem)
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # rigid-terrain compatible tire
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                           # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = hmmwv.GetChassisBody()    # cache: main chassis rigid body, reused below
# wheels/spindles created inside the wrapper via hmmwv.GetVehicle().GetAxles();
# suspension + steering joints are created inside the wrapper too.
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain === flat rigid patch textured with grass
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.6, 0.7, 0.4))
terrain.Initialize()

# === Random boxes === scattered rigid obstacles for the lidar to sense
random.seed(42)                                     # deterministic obstacle field
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.9)
box_mat.SetRestitution(0.01)
for i in range(num_boxes):
    bx = random.uniform(-box_area / 2.0, box_area / 2.0)
    by = random.uniform(-box_area / 2.0, box_area / 2.0)
    if abs(by) < 6.0:                               # keep the forward driving corridor clear
        continue
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(bx, by, box_size / 2.0))
    box.SetFixed(True)
    system.Add(box)

# === Sensor manager === hosts the lidar; point lights for any rendered sensor
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)

# === Lidar === chassis-mounted scanning lidar (depth + point-cloud streams)
lidar_offset = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 1.5),
                               chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    chassis,                          # mounted on the chassis
    lidar_update_rate,                # update_rate (Hz) — physical rate
    lidar_offset,                     # offset pose on the chassis
    lidar_h_samples,                  # horizontal samples
    lidar_v_samples,                  # vertical samples
    2 * chrono.CH_PI,                 # horizontal FOV (full 360 deg)
    chrono.CH_PI / 12,                # max vertical angle
    -chrono.CH_PI / 6,                # min vertical angle
    100.0,                            # max range (m)
    sens.LidarBeamShape_RECTANGULAR,  # beam shape
    2,                                # sample radius
    0.003,                            # vertical divergence angle
    0.003,                            # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)   # lidar: window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(lidar_h_samples, lidar_v_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())            # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())          # host access to XYZI
manager.AddSensor(lidar)

# === Visualization === full vehicle Irrlicht scene: window + sky + chase camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Lidar on Grass Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive driver bound to the visual system (truth shape)
driver = veh.ChInteractiveDriverIRR(vis)
render_step_size = 1.0 / render_fps
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop === advance vehicle + terrain + lidar; render throttled

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_every == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)              # advances the wrapper-owned system
        vis.Advance(time_step)

        manager.Update()                      # update ALL sensors — once per physics step

        step_number += 1
        realtime_timer.Spin(time_step)        # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:     # solver divergence / bad sim state
    import traceback
    traceback.print_exc()
    raise
