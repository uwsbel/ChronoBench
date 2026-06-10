"""HMMWV wheeled-vehicle simulation on flat rigid terrain with an onboard Depth Camera.

System type: ChSystemNSC (rigid-terrain catalog wheeled vehicle, owned by the
veh.HMMWV_Full wrapper). Main bodies: the HMMWV chassis + four wheels/tires, and a
flat RigidTerrain patch. An OptiX ChDepthCamera rides on the chassis with an offset
pose at (-5, 0, 2), 1280x720 resolution, 1.408 rad horizontal FOV, and a 30 m maximum
depth; a depth-map visualization filter previews its output. The vehicle state
(position X/Y/Z and heading) is logged every simulation step. Expected behavior: the
HMMWV rests on the terrain and drives forward under an interactive driver while the
depth camera produces a depth map of the scene ahead of the vehicle.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Parameters === geometry / physics constants and derived spawn pose
step_size = 2e-3                      # integration time step (s)
sim_end = 12.0                        # bounded recording horizon (s)
tire_step_size = 1e-3                 # tire model sub-step (s)

terrain_length = 400.0               # rigid patch X extent (m), spans the full forward run
terrain_width = 100.0                # rigid patch Y extent (m), centered under the vehicle
terrain_height = 0.0                 # top of the flat patch (m)

SUSPENSION_REF_HEIGHT = 0.5          # HMMWV chassis-origin height above wheel-bottom at rest
init_z = terrain_height + SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(0, 0, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Depth-camera specification (from the requested sensor configuration)
CAM_UPDATE_RATE = 30.0               # physical camera rate (Hz), not 1/dt
CAM_W = 1280                         # image width (px)
CAM_H = 720                          # image height (px)
CAM_HFOV = 1.408                     # horizontal field of view (rad)
CAM_MAX_DEPTH = 30.0                 # maximum sensed depth (m)
CAM_OFFSET = chrono.ChVector3d(-5.0, 0, 2)   # offset pose behind & above the chassis

render_step_size = 1.0 / 50.0        # render cadence (s)
render_steps = math.ceil(render_step_size / step_size)   # precomputed once

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === HMMWV full model on rigid terrain (NSC, wrapper-owned system)
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # rigid terrain road tire
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                       # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = hmmwv.GetChassisBody()                 # cache: main chassis rigid body, reused every step
# wheels/spindles: hmmwv.GetVehicle().GetSpindlePos(axle, side); terrain patch body below
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())   # report total vehicle mass

# Validate the wheels rest on (not through) the flat support after Initialize.
TIRE_RADIUS = 0.464                  # HMMWV tire radius (m), from wheel geometry
ZTOL = 0.05
veh_obj = hmmwv.GetVehicle()
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(axle, side).z
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
) - TIRE_RADIUS
assert wheel_bottom_z >= terrain_height - ZTOL, (
    f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs support top z={terrain_height:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === flat rigid patch under the vehicle
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
    terrain_length, terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === full vehicle Irrlicht scene: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV with Depth Camera")
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()             # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive real-time driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Depth camera sensor === onboard OptiX depth camera with depth-map preview
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(100, 100, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-100, 100, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0,
)

depth_offset_pose = chrono.ChFramed(
    CAM_OFFSET,
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),   # look forward along +X
)
depth_cam = sens.ChDepthCamera(
    chassis,                          # rides on the chassis body
    CAM_UPDATE_RATE,                  # 30 Hz physical update rate
    depth_offset_pose,                # offset pose at (-5, 0, 2)
    CAM_W, CAM_H,                     # 1280 x 720
    CAM_HFOV,                         # 1.408 rad horizontal FOV
    CAM_MAX_DEPTH,                    # 30 m maximum depth
)
depth_cam.SetName("Depth Camera")
depth_cam.SetLag(0)
depth_cam.SetCollectionWindow(0)
depth_cam.PushFilter(sens.ChFilterDepthToRGBA8())                 # depth -> colorized RGBA8
depth_cam.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H, "Depth Map"))  # depth-map preview
depth_cam.PushFilter(sens.ChFilterSave("cam/depth/"))            # save the depth-map stream
depth_cam.PushFilter(sens.ChFilterDepthAccess())                 # host access to depth buffer
manager.AddSensor(depth_cam)

# === Main loop === real-time Synchronize/Advance stack + per-step state logging
os.makedirs("cam", exist_ok=True)   # guard against missing output dir

realtime_timer = chrono.ChRealtimeStepTimer()
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
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)          # advances the wrapper-owned system
        vis.Advance(step_size)

        manager.Update()                  # pump the depth camera once per step

        step_number += 1
        realtime_timer.Spin(step_size)    # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
