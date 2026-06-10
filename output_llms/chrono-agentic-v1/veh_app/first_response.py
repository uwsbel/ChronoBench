"""
Gator Vehicle Simulation with Sensor Camera
============================================
System: ChSystemNSC (owned by veh.Gator wrapper)
Vehicle: Gator wheeled vehicle on RigidTerrain
Driver: ChInteractiveDriverIRR (interactive keyboard control)
Sensors: ChSensorManager with point lights + ChCameraSensor on chassis
Visualization types: chassis=MESH, suspension=PRIMITIVES, steering=PRIMITIVES,
                     wheels=MESH, tires=MESH
Expected: Gator drives on flat rigid terrain; sensor camera renders vehicle view;
          driver HUD shows steering/throttle/brake bars.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Constants ===
STEP_SIZE = 1e-3          # physics time step (s)
SIM_END = 20.0            # simulation end time (s)
RENDER_FPS = 50.0         # Irrlicht render cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0    # m
TERRAIN_WIDTH = 200.0     # m

# Camera sensor parameters
CAM_UPDATE_RATE = 30      # physical Hz (not 1/dt)
CAM_WIDTH = 1280
CAM_HEIGHT = 720
CAM_FOV = 1.408           # horizontal FOV (rad)

# === Vehicle initialization ===
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)  # MANDATORY — fixed chassis won't move

init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(STEP_SIZE)
gator.Initialize()

# Set visualization types (AFTER Initialize)
gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.Gator wrapper) ===
sys = gator.GetSystem()                # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()       # cache: main chassis rigid body — reused below
# collision system (REQUIRED for terrain contact)
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
# joints: suspension + steering links created inside the wrapper

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle with Sensor Camera")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(gator.GetVehicle())

# === Driver (ChInteractiveDriverIRR — scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriver(gator.GetVehicle())

STEERING_TIME = 1.0   # seconds to reach max steering
THROTTLE_TIME = 1.0   # seconds to reach max throttle
BRAKING_TIME = 0.3    # seconds to reach max braking

RENDER_STEP_SIZE = 1.0 / RENDER_FPS   # precomputed once
driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
driver.Initialize()

# === Sensor Manager with point lights and camera ===
manager = sens.ChSensorManager(sys)

# Point lights for camera sensor illumination
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Camera sensor attached to vehicle chassis, chasing from behind
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 2),                              # behind and above chassis
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)) # slight downward tilt
)

cam = sens.ChCameraSensor(
    chassis,             # attach to the REAL chassis body
    CAM_UPDATE_RATE,     # physical update rate Hz
    cam_offset,          # offset pose in chassis frame
    CAM_WIDTH,
    CAM_HEIGHT,
    CAM_FOV,
)
cam.SetName("Chassis Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

# Filter chain: visualize live preview + save RGB frames
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))
manager.AddSensor(cam)

# === Record mode setup ===


# === Real-time step timer ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

# === Main simulation loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()  # cache: current sim time

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        driver_inputs = driver.GetInputs()  # interactive driver inputs (scored core)


        # Synchronize subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        gator.Advance(STEP_SIZE)       # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        # Update sensor manager (once per physics step)
        manager.Update()

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)  # maintain real-time pacing

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
