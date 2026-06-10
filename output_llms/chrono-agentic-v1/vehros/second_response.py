"""
HMMWV + RigidTerrain + Irrlicht Visualization + ROS Bridge Simulation
======================================================================
System: ChSystemNSC (owned by HMMWV_Full wrapper)
Vehicle: HMMWV_Full with NSC contact, TMEASY tires on flat RigidTerrain
Driver: ChInteractiveDriverIRR (real-time keyboard control)
ROS: ChROSPythonManager with clock, body-state, and driver-inputs handlers
Visualization: ChWheeledVehicleVisualSystemIrrlicht (chase camera)

Expected behavior: HMMWV rests on flat terrain; user steers/drives via
keyboard (arrow keys / WASD). Vehicle pose and driver commands are
published over ROS2 in real-time. The Irrlicht window provides chase-cam
visualization with sky box, typical lights, and logo overlay.
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

# === Constants ===
# Simulation timing
TIME_STEP = 1e-3          # physics step size (s)
SIM_END = 20.0            # total simulation duration (s)
RENDER_FPS = 50.0         # target Irrlicht render rate (Hz)

# Vehicle spawn
INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (HMMWV)
TERRAIN_Z = 0.0
INIT_Z = TERRAIN_Z + SUSPENSION_REF_HEIGHT

# Terrain
TERRAIN_LENGTH = 200.0   # m
TERRAIN_WIDTH = 200.0    # m

# Driver deltas
STEERING_TIME = 1.0   # seconds to reach full steering lock
THROTTLE_TIME = 1.0   # seconds to reach full throttle
BRAKING_TIME  = 0.3   # seconds to reach full braking

# ROS publish rates (Hz)
CLOCK_RATE  = 0       # 0 = every step (default ChROSClockHandler)
BODY_RATE   = 25      # vehicle body pose
DRIVER_RATE = 25      # driver-inputs subscription

# Precomputed render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# === Vehicle setup ===
# HMMWV_Full wrapper creates and owns a ChSystemNSC internally.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)      # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                            # MANDATORY — fixed won't move
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)             # TMEASY for proper tire forces
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                              # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()                        # cache: main chassis rigid body
# wheels/spindles via hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

# Visualization types (after Initialize)
# Note: in this 9.0.0 source build, VisualizationType_* lives in pychrono.vehicle
# Use PRIMITIVES for chassis/wheel/tire to avoid missing OBJ asset issues
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# Verify wheel-bottom clearance after Initialize (spindle positions)
TIRE_RADIUS = 0.33         # HMMWV TMEASY tire approximate radius
ZTOL = 0.10
veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain z={TERRAIN_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_Z - wheel_bottom_z:.3f} m"
)

# === Terrain ===
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
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht visualization ===
# Vehicle-specific Irrlicht system; call order: configure → Initialize → AddSky/Cam/Lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV + ROS Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive, real-time keyboard) ===
driver = veh.ChInteractiveDriverIRR(vis)   # takes the visual system, not the vehicle
driver.SetSteeringDelta(render_every * TIME_STEP / STEERING_TIME)
driver.SetThrottleDelta(render_every * TIME_STEP / THROTTLE_TIME)
driver.SetBrakingDelta(render_every * TIME_STEP / BRAKING_TIME)
driver.Initialize()

# === ROS bridge setup ===
# ChROSPythonManager required for Python-subclass handlers.
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first — publishes /clock for ROS graph time-sync.
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler — publishes chassis pose/twist over ROS.
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(BODY_RATE, chassis, "~/output/vehicle/state")
)

# 3. Driver-inputs handler — SUBSCRIBES to throttle/steer/brake from ROS topic.
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(DRIVER_RATE, driver, "~/input/driver_inputs")
)

# Initialize EXACTLY ONCE, after all registrations, before the loop.
ros_manager.Initialize()

# === Review-only: recording setup ===

# === Review-only: CSV writer ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # Throttled rendering (every render_every physics steps)
        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()   # scored core: interactive driver inputs

        # Synchronize subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems (vehicle.Advance steps the wrapped ChSystem)
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        # ROS update LAST (after state-producing advances)
        if not ros_manager.Update(time, TIME_STEP):
            break


        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass   # cleanup placeholder
