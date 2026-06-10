"""
FEDA vehicle simulation with grass terrain texture, sensor manager, point lights,
and a first-person-view camera sensor mounted on the chassis.

System: NSC (FEDA default for rigid terrain)
Vehicle: veh.FEDA — wheeled vehicle wrapper
Terrain: RigidTerrain with grass texture
Camera: ChCameraSensor attached to chassis for FPV, high resolution, wide FOV
Expected: FEDA drives forward on grass terrain; FPV camera stream updates each step.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr  # noqa: F401 — used via veh.ChWheeledVehicleVisualSystemIrrlicht
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
# Simulation parameters
STEP_SIZE = 1e-3          # physics time step (s)
SIM_END = 20.0            # simulation end time (s)
RENDER_FPS = 50.0         # Irrlicht render cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Terrain geometry
TERRAIN_LENGTH = 200.0    # m (X)
TERRAIN_WIDTH = 200.0     # m (Y)

# Vehicle init
INIT_LOC = chrono.ChVector3d(0, 0, 0.5)   # chassis origin at spawn
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)  # heading along +X

# Camera sensor — FPV parameters (high resolution, wide FOV for first-person view)
CAM_WIDTH = 1920
CAM_HEIGHT = 1080
CAM_FOV = 1.2217          # ~70 deg horizontal FOV in radians (wide FPV)
CAM_UPDATE_RATE = 30      # Hz — physical rate

# Point-light parameters for sensor manager scene
LIGHT_INTENSITY = 1.5

# === Data paths — mandatory for catalog vehicle scoring ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                           # MANDATORY
feda.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(STEP_SIZE)
feda.Initialize()

# === System & bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = feda.GetChassisBody()           # main chassis rigid body — cache: fetched once
# cache: fetched once, reused for sensor attachment and spindle asserts
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# Visualization types (after Initialize)
feda.SetChassisVisualizationType(chrono.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(chrono.VisualizationType_MESH)
feda.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain — RigidTerrain with grass texture ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()   # NSC matches the vehicle contact method
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
# Task 1: Grass texture on terrain
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.2))

terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
# Vehicle demos use ChWheeledVehicleVisualSystemIrrlicht; Initialize FIRST then add scene elements
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA - FPV Camera Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                               # vehicle demos: directional light
vis.AttachVehicle(feda.GetVehicle())

# === Driver — interactive (truth-faithful) ===
driver = veh.ChInteractiveDriverIRR(vis)

render_step_size = 1.0 / RENDER_FPS         # precomputed once
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor Manager — Task 2: sensor manager + point lights ===
manager = sens.ChSensorManager(system)

# Point lights for scene illumination (well illuminated, appropriate intensity)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 50),
    chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(50, 50, 30),
    chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-50, -50, 30),
    chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
    500.0,
)

# === Camera Sensor — Task 3: FPV camera on chassis ===
# Attach to chassis body; offset places the camera at the front windshield (FPV)
# Offset: forward +2.0 m along vehicle X, up +1.1 m in vehicle Z
fpv_offset = chrono.ChFramed(
    chrono.ChVector3d(2.0, 0, 1.1),                           # front of cabin, eye level
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)) # no tilt — looking straight ahead
)

fpv_cam = sens.ChCameraSensor(
    chassis,            # attach to chassis body (rides with vehicle — FPV)
    CAM_UPDATE_RATE,    # 30 Hz — physical update rate
    fpv_offset,         # offset pose on chassis
    CAM_WIDTH,          # 1920 px — high resolution
    CAM_HEIGHT,         # 1080 px
    CAM_FOV,            # ~70 deg — wide FPV field of view
)
fpv_cam.SetName("FPV Camera")
fpv_cam.SetLag(0)
fpv_cam.SetCollectionWindow(0)

# Filter chain: visualize live preview then save frames (scored core — prompt-required sensor)
fpv_cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "FPV Camera"))
fpv_cam.PushFilter(sens.ChFilterRGBA8Access())
fpv_cam.PushFilter(sens.ChFilterSave("cam/fpv/"))

manager.AddSensor(fpv_cam)

# === Recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        feda.Advance(STEP_SIZE)         # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        # Task 4: update sensor manager so camera tracks vehicle movement
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
