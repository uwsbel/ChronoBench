"""
HMMWV Vehicle with ROS2 Integration (PyChrono 9.0.x / Irrlicht)

Models an HMMWV_Full wheeled vehicle on RigidTerrain with full ROS2 bridge:
- ChROSClockHandler        — publishes /clock for ROS graph time sync
- ChROSDriverInputsHandler — subscribes throttle/steer/brake from ROS topic
- ChROSBodyHandler         — publishes chassis pose/twist to ROS topic

Contact method: NSC (rigid terrain / catalog vehicle default).
Expected behavior: vehicle rests on flat terrain, responds to ROS-published
driver commands; chassis state is broadcast over ROS topics each simulation step.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


# === Named constants ===
STEP_SIZE            = 1e-3      # physics time step (s)
SIM_END              = 20.0      # simulation end time (s)
RENDER_FPS           = 50.0
RENDER_EVERY         = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
render_step_size     = 1.0 / RENDER_FPS   # precomputed once

TERRAIN_LENGTH       = 200.0    # terrain patch length (m)
TERRAIN_WIDTH        = 200.0    # terrain patch width  (m)
TERRAIN_FRICTION     = 0.9
TERRAIN_RESTITUTION  = 0.01

INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5    # chassis origin above wheel-bottom at rest (HMMWV ≈ 0.5 m)
INIT_Z = 0.0 + SUSPENSION_REF_HEIGHT  # flat terrain top z = 0

STEERING_TIME = 1.0   # s to reach full steering
THROTTLE_TIME = 1.0   # s to reach full throttle
BRAKING_TIME  = 0.3   # s to reach full braking

# === Data paths (mandatory truth components for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
init_rot = chrono.QuatFromAngleZ(0.0)
init_pos = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)    # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)            # prompt: TMEASY tire model
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys     = hmmwv.GetSystem()       # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()  # cache: main chassis rigid body, reused every step
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)...; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the HMMWV wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())  # mandatory truth diagnostic

# === Terrain ===
terrain   = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (Initialize FIRST, then add scene elements) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV + ROS2")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()            # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive IRR — catalog vehicle default) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === ROS2 Manager + handlers ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler FIRST — publishes /clock so the ROS graph is time-synced to sim
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Driver inputs handler — SUBSCRIBES throttle/steer/brake from a ROS topic
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")
)

# 3. Body handler — publishes chassis pose/twist to ROS
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, chassis, "~/output/vehicle/state")
)

# Initialize exactly once — after ALL handlers registered, before the loop
ros_manager.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems in order: driver -> terrain -> vehicle -> vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems (hmmwv.Advance steps the wrapper-owned ChSystem)
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        # ROS update LAST — after state-producing updates
        if not ros_manager.Update(time, STEP_SIZE):
            break

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
