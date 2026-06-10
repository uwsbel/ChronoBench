"""
HMMWV Vehicle with ROS Integration.

Models an HMMWV_Full wheeled vehicle on a flat RigidTerrain (NSC contact)
with a ChInteractiveDriverIRR driver. A ChROSPythonManager publishes:
  - /clock (ChROSClockHandler) for ROS time synchronization
  - driver inputs (ChROSDriverInputsHandler) subscribing to steering/throttle/brake
  - vehicle chassis state (ChROSBodyHandler) publishing pose/twist

System type: ChSystemNSC (owned by HMMWV_Full wrapper).
Expected behavior: HMMWV rests on flat terrain, responds to driver inputs,
publishes vehicle state over ROS2 at each timestep.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


# === Constants ===
time_step = 1e-3        # simulation timestep (s)
sim_end = 10.0          # simulation end time (s)
render_fps = 50.0       # Irrlicht render rate (Hz)

TERRAIN_LENGTH = 200.0  # terrain patch length (m)
TERRAIN_WIDTH  = 200.0  # terrain patch width (m)

INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5  # chassis origin above wheel-bottom at rest (HMMWV ~0.5 m)
INIT_Z = SUSPENSION_REF_HEIGHT

# Precomputed once
render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per frame

# === Vehicle setup ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())         # bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")     # vehicle subtree

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                  # MANDATORY: fixed chassis won't move
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
        chrono.QUNIT,
    )
)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # TMEASY for robust terrain interaction
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                       # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()                 # cache: main chassis rigid body; fetched once
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)...; terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the HMMWV_Full wrapper

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (must come after Initialize)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(hmmwv.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with ROS")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()        # vehicle truths use AddLightDirectional (not AddTypicalLights)
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0   # seconds to go 0 -> +1 steering
throttle_time = 1.0   # seconds to go 0 -> +1 throttle
braking_time  = 0.3   # seconds to go 0 -> +1 brake

driver.SetSteeringDelta(1.0 / (render_fps * steering_time))
driver.SetThrottleDelta(1.0 / (render_fps * throttle_time))
driver.SetBrakingDelta(1.0 / (render_fps * braking_time))
driver.Initialize()

# === ROS Manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first — publishes /clock for ROS time synchronization
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Driver inputs handler — subscribes to steering/throttle/brake from ROS
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")
)

# 3. Body handler — publishes chassis pose/twist to ROS
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, chassis, "~/output/vehicle/state")
)

# Initialize exactly once, after all handlers are registered
ros_manager.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
        time = hmmwv.GetSystem().GetChTime()

        if step_number % render_every == 0:
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
        hmmwv.Advance(time_step)      # advances the wrapper-owned ChSystem
        vis.Advance(time_step)

        # ROS update last — after all state-producing updates
        if not ros_manager.Update(time, time_step):
            break


        step_number += 1
        realtime_timer.Spin(time_step)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # CSV closed below after the loop
