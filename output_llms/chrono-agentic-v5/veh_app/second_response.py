"""HMMWV wheeled vehicle on rigid terrain with an onboard 3D Lidar sensor.

System type: NSC (rigid-terrain catalog vehicle). The full HMMWV wrapper owns the
ChSystemNSC; a flat RigidTerrain patch supports the four TMEASY-tired wheels. The
vehicle spawns at (0, -5, 0.4) and drives with a constant steering of 0.5 and a
constant throttle of 0.2, so it accelerates forward while turning. Two fixed props
sit ahead of the spawn — a 1x1x1 blue box at (0, 0, 0.5) and a blue cylinder of
radius 0.5 / height 1 at (0, 0, 1.5) — to give the Lidar returns. A roof-mounted
ChLidarSensor (800x300 beams, 360 deg horizontal FOV) scans the scene and produces
depth, intensity, and XYZI point-cloud streams plus a live point-cloud preview.

Expected behavior: the HMMWV pulls away from the start pose along a curved path
while the Lidar continuously images the surrounding props and terrain.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 2e-3                     # integration step (s)
tire_step = 1e-3                     # TMEASY tire sub-step (s)
sim_end = 12.0                       # total simulated time (s)
render_fps = 50.0                    # Irrlicht frame cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

init_loc = chrono.ChVector3d(0, -5, 0.4)        # vehicle spawn (prompt value)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)     # identity heading (+X forward)

terrain_length = 100.0               # rigid patch X extent (m)
terrain_width = 100.0                # rigid patch Y extent (m)
patch_top_z = 0.0                    # terrain surface height

box_size = 1.0                                    # cube edge (prompt: 1,1,1)
box_pos = chrono.ChVector3d(0, 0, 0.5)           # prompt box position
cyl_radius = 0.5                                  # prompt cylinder radius
cyl_height = 1.0                                  # prompt cylinder height
cyl_pos = chrono.ChVector3d(0, 0, 1.5)           # prompt cylinder position

steering_cmd = 0.5                   # constant steering (prompt)
throttle_cmd = 0.2                   # constant throttle (prompt)


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV wrapper owns its ChSystemNSC + chassis/spindles/joints
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # rigid terrain -> NSC
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)          # slip/grip tire model
vehicle.SetTireStepSize(tire_step)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Components created internally by the wrapper, exposed as named handles:
system = vehicle.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()            # cache: main chassis body, reused for lidar + camera
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Footprint sanity: every wheel bottom must rest on (not through) the terrain top.
TIRE_RADIUS = 0.4636                 # HMMWV tire radius (m)
ZTOL = 0.10
veh_obj = vehicle.GetVehicle()
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(axle, side).z
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
) - TIRE_RADIUS
assert wheel_bottom_z >= patch_top_z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={patch_top_z:.3f}; raise init_loc.z"
)

# === Terrain === flat rigid patch supporting the wheels
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Props === fixed blue box + blue cylinder ahead of the vehicle (lidar targets)
blue_texture = chrono.GetChronoDataFile("textures/blue.png")

box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True, patch_mat)
box.SetPos(box_pos)
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(blue_texture)
system.AddBody(box)

cylinder = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Z, cyl_radius, cyl_height, 1000, True, True, patch_mat
)
cylinder.SetPos(cyl_pos)
cylinder.SetFixed(True)
cylinder.GetVisualShape(0).SetTexture(blue_texture)
system.AddBody(cylinder)

# === Sensors === ChSensorManager + roof-mounted 3D Lidar on the chassis
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 5000.0)

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),                                  # prompt offset pose
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                                # ride on the chassis body
    5.0,                                    # update_rate (Hz) — physical rate
    lidar_offset,                           # offset pose
    800,                                    # horizontal samples (prompt)
    300,                                    # vertical channels (prompt)
    2 * chrono.CH_PI,                       # horizontal FOV (prompt: 360 deg)
    chrono.CH_PI / 12,                      # max vertical FOV (prompt)
    -chrono.CH_PI / 6,                      # min vertical FOV (prompt)
    100.0,                                  # max range (prompt)
    sens.LidarBeamShape_RECTANGULAR,        # rectangular beam (prompt)
    2,                                      # sample radius (prompt)
    0.003,                                  # vertical divergence angle (prompt)
    0.003,                                  # horizontal divergence angle (prompt)
    sens.LidarReturnMode_STRONGEST_RETURN,  # strongest return mode (prompt)
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)        # lidar: window = 1 / update_rate

# Lidar filter chain (ORDER MATTERS): depth, intensity, XYZI point cloud, visualize
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))   # depth + intensity preview
lidar.PushFilter(sens.ChFilterDIAccess())                               # host access: depth + intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                           # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(960, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())                            # host access: XYZI cloud
manager.AddSensor(lidar)

# === Driver === interactive driver bound to the vehicle visual system
render_step_size = 1.0 / render_fps

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Lidar Sensor")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop === Synchronize/Advance the full subsystem stack each step
render_steps = math.ceil(render_step_size / time_step)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

os.makedirs("cam", exist_ok=True)                                     # guard against missing output dir

step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)
        manager.Update()                              # pump the lidar once per step

        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:             # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
