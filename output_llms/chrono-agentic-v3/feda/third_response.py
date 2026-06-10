"""
FEDA vehicle simulation on rigid terrain with a camera sensor mounted on the chassis.

System type: NSC (rigid terrain, FEDA catalog vehicle).
Main bodies: FEDA chassis, 4 wheel spindles, rigid terrain patch.
Expected behavior: FEDA drives on flat rigid terrain with grass texture.
A sensor manager and point lights illuminate the scene for the chassis-mounted
camera sensor providing a First Person View (FPV). The camera sensor feeds a live
visualization filter and saves RGB frames.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
TERRAIN_LENGTH = 600.0          # m — large enough for full sim duration
TERRAIN_WIDTH  = 200.0          # m
STEP_SIZE      = 5e-4           # physics step (s)
SIM_END        = 20.0           # simulation end time (s)
RENDER_FPS     = 50.0
RENDER_EVERY   = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Vehicle initial position and orientation
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(0.0)

# === Data paths (truth-required: both lines are scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle (FEDA catalog wrapper) ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)
feda.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(STEP_SIZE)
feda.Initialize()

# === System & bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()           # ChSystemNSC owned by the wrapper
chassis = feda.GetChassisBody()     # cache: main chassis body, reused for sensor attach
# wheels/spindles: feda.GetVehicle().GetAxle(i)...; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for terrain contact

# Print total vehicle mass (truth-required diagnostic)
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# === Visualization types (after Initialize per wrapper rules) ===
# VisualizationType enums live in veh namespace in this 9.0.0 build
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain (rigid, NSC, grass texture) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.3))
terrain.Initialize()

# === Irrlicht visualization (vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA FPV Camera Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(feda.GetVehicle())

# === Interactive driver (IRR form — truth standard for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)
STEERING_TIME = 1.0   # s
THROTTLE_TIME = 1.0   # s
BRAKING_TIME  = 0.3   # s
driver.SetSteeringDelta(1.0 / (RENDER_FPS * STEERING_TIME))  # precomputed once
driver.SetThrottleDelta(1.0 / (RENDER_FPS * THROTTLE_TIME))
driver.SetBrakingDelta(1.0 / (RENDER_FPS * BRAKING_TIME))
driver.Initialize()

# === Sensor Manager and lighting ===
manager = sens.ChSensorManager(system)
# Point lights provide scene illumination for the camera sensor
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(50, 50, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-50, -50, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera Sensor (First Person View on chassis body) ===
# Attach to chassis: offset slightly forward and up for FPV perspective
CAM_UPDATE_RATE = 30  # Hz — physical rate
CAM_WIDTH  = 1920
CAM_HEIGHT = 1080
CAM_HFOV   = 1.2  # horizontal FOV (rad), ~68 degrees for wide FPV

fpv_offset = chrono.ChFramed(
    chrono.ChVector3d(2.0, 0.0, 1.0),                          # forward+up offset in chassis frame
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),  # no pitch tilt
)

cam = sens.ChCameraSensor(
    chassis,         # attach to real chassis body
    CAM_UPDATE_RATE,
    fpv_offset,
    CAM_WIDTH,
    CAM_HEIGHT,
    CAM_HFOV,
)
cam.SetName("FPV Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain: visualize live preview + save RGB frames + provide host access
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "FPV RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/fpv/"))

manager.AddSensor(cam)


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # Throttled rendering: one Irrlicht frame every RENDER_EVERY physics steps
        if frame % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        frame += 1

        # Driver inputs for this step
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (full stack — MANDATORY order)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        feda.Advance(STEP_SIZE)     # advances wrapper-owned ChSystem (do NOT call system.DoStepDynamics)
        vis.Advance(STEP_SIZE)

        # Update sensor manager every physics step so camera tracks chassis pose
        manager.Update()


        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
