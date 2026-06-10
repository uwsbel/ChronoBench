"""
FEDA Vehicle Simulation with Sensor Camera (First-Person View)
==============================================================
Models a FEDA wheeled vehicle on rigid terrain (SMC contact method)
with:
  - Grass terrain texture
  - ChSensorManager with point lights for illumination
  - ChCameraSensor attached to the chassis for First-Person View
    (high resolution 1920x1080, wide horizontal FOV 1.22 rad ~70 deg)
  - ChFilterVisualize preview and ChFilterSave recording
  - Interactive Irrlicht visualization window
  - manager.Update() called each physics step

System: ChSystemSMC (owned by FEDA wrapper)
Primary bodies: FEDA chassis, wheels, terrain patch
Expected behavior: FEDA vehicle rests on flat terrain; sensor camera
provides real-time first-person view from the chassis; vehicle is
driven interactively via keyboard.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
# Simulation timing
TIME_STEP = 5e-4           # physics step size (s)
SIM_END = 30.0             # simulation end time (s)
RENDER_FPS = 50.0          # Irrlicht render rate (frames/s)

# Terrain geometry
TERRAIN_LENGTH = 300.0     # terrain patch length (m)
TERRAIN_WIDTH  = 300.0     # terrain patch width (m)

# Vehicle initial position — FEDA geometric center at origin
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)   # 0.5 m above terrain (inferred default — verify)
INIT_ROT = chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 0, 1))

# Camera sensor parameters — high-res first-person view
CAM_WIDTH  = 1920          # pixels
CAM_HEIGHT = 1080          # pixels
CAM_FOV    = 1.22          # horizontal FOV (rad) ≈ 70 deg — wide first-person FOV
CAM_RATE   = 30            # update rate (Hz, physical rate not 1/dt)

# Camera offset in chassis frame — mounted slightly forward and up for FPV
CAM_OFFSET_POS = chrono.ChVector3d(1.2, 0.0, 1.5)   # forward and elevated on chassis
# Rotate camera to look forward: 0 deg about Y in chassis frame
CAM_OFFSET_ROT = chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0))

# Precomputed render cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Irrlicht driver rates for interactive control
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME  = 0.3

# === Vehicle Data Path (must be absolute to work from any cwd) ===
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === FEDA Vehicle Setup ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_SMC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                              # MANDATORY — fixed chassis won't move
feda.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_RIGID)    # RIGID tires for flat rigid terrain
feda.SetTireStepSize(TIME_STEP)
feda.Initialize()

# === System & Bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()                # ChSystemSMC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = feda.GetChassisBody()          # cache: main chassis rigid body — reused for sensor attach
# wheels/spindles: feda.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the FEDA wrapper

# === Visualization types (veh namespace in 9.0.0 source build) ===
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain (Rigid, SMC material, grass texture) ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
# Grass texture applied per prompt
patch.SetTexture(veh.GetDataPath() + "terrain/textures/grass.jpg", 200, 200)
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.3))

terrain.Initialize()

# === Irrlicht Visualization (vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA FPV Camera Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(feda.GetVehicle())

# === Interactive Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / (RENDER_FPS * STEERING_TIME) if RENDER_FPS > 0 else 0.04)
driver.SetThrottleDelta(1.0 / (RENDER_FPS * THROTTLE_TIME) if RENDER_FPS > 0 else 0.02)
driver.SetBrakingDelta(1.0 / (RENDER_FPS * BRAKING_TIME)  if RENDER_FPS > 0 else 0.06)
driver.Initialize()

# === Sensor Manager and Point Lights ===
# Sensor manager creation — required before adding sensors
manager = sens.ChSensorManager(system)
# Add multiple point lights spread across scene for well illuminated view (as per prompt)
intensity = 1.5
manager.scene.AddPointLight(
    chrono.ChVector3f(0.0, 0.0, 50.0),
    chrono.ChColor(intensity, intensity, intensity),
    1000.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(50.0, 50.0, 30.0),
    chrono.ChColor(intensity, intensity, intensity),
    800.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-50.0, 30.0, 30.0),
    chrono.ChColor(intensity, intensity, intensity),
    800.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(30.0, -50.0, 30.0),
    chrono.ChColor(intensity, intensity, intensity),
    800.0,
)

# === Camera Sensor (First-Person View on chassis) ===
# Attached to the chassis body for FPV — offset forward and elevated in chassis frame
offset_pose = chrono.ChFramed(CAM_OFFSET_POS, CAM_OFFSET_ROT)

cam = sens.ChCameraSensor(
    chassis,           # attach to real chassis body for true FPV
    CAM_RATE,          # physical update rate (Hz) — not 1/dt
    offset_pose,       # forward+elevated offset in chassis local frame
    CAM_WIDTH,         # high resolution 1920
    CAM_HEIGHT,        # high resolution 1080
    CAM_FOV,           # wide FOV for first-person view (~70 deg)
)
cam.SetName("FPV Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain: visualize preview + access buffer + save PNG frames
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "First-Person View"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/fpv/"))   # save FPV frames as PNGs -> assembled to mp4

manager.AddSensor(cam)

# === Review-only recording setup ===

# === Main Simulation Loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run():
        time = system.GetChTime()

        # Throttled Irrlicht rendering
        if frame % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        frame += 1

        # Driver inputs
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (driver -> terrain -> vehicle -> vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        feda.Advance(TIME_STEP)    # steps the wrapper-owned ChSystem
        vis.Advance(TIME_STEP)

        # Sensor manager update — once per physics step, after Advance
        manager.Update()


        # Guard simulation end time
        if time >= SIM_END:
            break

        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
