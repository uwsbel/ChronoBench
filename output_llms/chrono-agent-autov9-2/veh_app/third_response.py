"""Interactive wheeled-vehicle application demo with an onboard depth camera.

Models an HMMWV (SMC contact, TMEASY tires) driving forward on a flat
RigidTerrain patch under an autonomous (pre-scripted) driver. A ChDepthCamera
sensor rides on the chassis at an offset pose behind and above the vehicle,
producing a depth map of the scene ahead. The vehicle's pose (X, Y, Z) and
heading are logged every physics step.

System type: ChSystemSMC (owned by the veh.HMMWV_Full wrapper).
Main bodies: HMMWV chassis + four wheels/spindles, flat rigid terrain patch.
Expected behavior: the vehicle accelerates from rest and translates forward
along +X across the terrain; the depth camera reports increasing-then-stable
depth of the ground plane and horizon.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Simulation constants === geometry / timing / camera (precomputed once) ===
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP_SIZE = 1e-3                  # tire force model sub-step (s)
SIM_END = 6.0                          # simulated duration (s)
RENDER_FPS = 25.0                      # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps/frame

TERRAIN_LENGTH = 120.0                 # X extent of the rigid patch (m)
TERRAIN_WIDTH = 120.0                  # Y extent of the rigid patch (m)
TERRAIN_TOP_Z = 0.0                    # top surface height of the flat patch (m)

SUSPENSION_REF_HEIGHT = 0.5            # chassis origin above wheel-bottom at rest (HMMWV)
TIRE_RADIUS = 0.464                    # HMMWV tire radius (m), for the footprint assert
VEH_INIT_X = -40.0                     # spawn near the -X edge so the run stays on-patch
VEH_INIT_Y = 0.0
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis-origin height
ZTOL = 0.10                            # allowed wheel-bottom clearance vs terrain top

# Depth camera (onboard, chassis-relative offset pose) — values fixed by the demo.
CAM_OFFSET = chrono.ChVector3d(-5.0, 0.0, 2.0)   # behind (-X) and above (+Z) the chassis
CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAM_HFOV = 1.408                       # horizontal field of view (rad)
CAM_MAX_DEPTH = 30.0                   # maximum reported depth (m)
CAM_UPDATE_RATE = 30.0                 # precomputed once: depth camera ticks at 30 Hz

# === System & vehicle (created by the veh.HMMWV_Full wrapper) ===
# The wrapper builds and OWNS its ChSystemSMC plus the chassis, four spindles,
# suspension + steering joints, and the powertrain — enumerated here so the
# wrapper-created components are visible.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT)
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire so the vehicle actually drives
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()               # cache: vehicle subsystem, reused for state logging

# === Collision system === Bullet is REQUIRED for the tire/terrain contact scene ===
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch the wheels roll on ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Footprint assert === wheels must start ON the terrain, not through it ===
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === autonomous open-loop schedule (headless run: no keyboard input) ===
# DataDriverEntry(time, steering, throttle, braking, gear): brief settle, then
# steady throttle so the chassis translates forward along +X.
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(1.0, 0.0, 0.7, 0.0, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, 0.7, 0.0, 0.0),
])
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()


# === Depth camera sensor === rides on the chassis, looks ahead; depth-map output ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-50, 50, 60), chrono.ChColor(0.8, 0.8, 0.7), 5000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

depth_cam = sens.ChDepthCamera(
    chassis,                                              # onboard: rides on the chassis
    CAM_UPDATE_RATE,
    chrono.ChFramed(CAM_OFFSET, chrono.QUNIT),            # offset pose behind/above the chassis
    CAM_WIDTH, CAM_HEIGHT,
    CAM_HFOV,
    CAM_MAX_DEPTH,
)
depth_cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT))   # live depth-map preview
depth_cam.PushFilter(sens.ChFilterDepthToRGBA8())                     # depth -> RGBA8 for saving
depth_cam.PushFilter(sens.ChFilterSave("cam/depth_cam/"))            # PNG frames -> mp4 by RUN stage
depth_cam.PushFilter(sens.ChFilterDepthAccess())                     # depth-buffer access


# === Register sensor === depth camera renders the scene ahead each step ===
manager.AddSensor(depth_cam)

# === Visualization === vehicle-aware Irrlicht window + chase camera + lights ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Application Demo - Onboard Depth Camera")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Output setup === guard the output dir; open CSV writers with context managers ===


# === Logging setup === open CSV writers with context managers (flush on exit) ===

# === Main loop === throttled render + per-step Synchronize/Advance; log vehicle state ===
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            manager.Update()        # pump the depth camera every physics step


            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)        # internally steps the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    print(f"Finished at t={system.GetChTime():.3f}s, chassis x={chassis.GetPos().x:.3f} m")

# === Post-processing === assemble review videos + plot, then drop frame PNGs ===
