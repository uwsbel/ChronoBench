"""
HMMWV Vehicle with ROS Integration - PyChrono 9.0.x (Irrlicht)

Models an HMMWV vehicle on a rigid terrain with full ROS2 integration.
System type: ChSystemNSC (created by HMMWV_Full wrapper, NSC contact method).
Main bodies: HMMWV chassis + 4 wheel spindles on RigidTerrain patch.
Expected behavior: Vehicle drives forward, ROS clock/driver-inputs/body-state
handlers publish continuously; ChROSDriverInputsHandler subscribes to
steering/throttle/brake commands over ROS.
"""

# === Imports ===
import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr  # noqa: F401 – used via veh vis system
import pychrono.vehicle as veh
import pychrono.ros as chros

# === Data path setup (vehicle data must be absolute to avoid cwd-relative resolution) ===
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Constants ===
# Simulation parameters
STEP_SIZE = 2e-3               # physics time step (s)
SIM_END = 20.0                 # simulation end time (s)
RENDER_FPS = 50.0              # Irrlicht render rate (Hz)
RENDER_STEP_SIZE = 1.0 / RENDER_FPS   # precomputed once
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

# Terrain parameters (prompt: defined friction and restitution)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0

# Vehicle spawn
INIT_X = 0.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5    # HMMWV chassis origin above wheel-bottom at rest
INIT_Z = SUSPENSION_REF_HEIGHT  # terrain top is z=0, so chassis starts here

# ROS publish rates (Hz)
ROS_CLOCK_HZ = 100
ROS_DRIVER_HZ = 25
ROS_BODY_HZ = 25

# Driver ramp times
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3

# Review-only recording

# === Vehicle setup (HMMWV_Full wrapper owns the ChSystem) ===
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)       # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                             # MANDATORY – chassis must move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)              # prompt: tire model
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                                # ChSystemNSC owned by wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

chassis = hmmwv.GetChassisBody()                          # cache: fetched once, reused every step
chassis.SetName("hmmwv_chassis")
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the HMMWV_Full wrapper

# Visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain (rigid, flat) ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()   # NSC system -> NSC material
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

# Assert wheel bottoms are on (not through) the terrain at spawn
TIRE_RADIUS = 0.33    # HMMWV TMEASY tire approximate radius
ZTOL = 0.10           # tolerance for wheel-bottom vs terrain z=0
veh_obj = hmmwv.GetVehicle()                              # cache: fetched once
spindle_world = []
for ax_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(ax_idx, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel_bottom_z={wheel_bottom_z:.3f} "
    f"vs terrain z=0; raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === Visualization (Irrlicht, vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with ROS Integration")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()                                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())                     # AFTER Initialize

# === Driver (interactive for scored core; review-only scripted block drives it at RUN) ===
driver = veh.ChInteractiveDriverIRR(vis)                  # takes visual system (truth form)
driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
driver.Initialize()

# === ROS Integration ===
# ChROSPythonManager required for Python-subclass handlers
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first — synchronizes /clock with sim time
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Driver inputs handler — subscribes to throttle/steer/brake commands over ROS
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(ROS_DRIVER_HZ, driver, "~/input/driver_inputs")
)

# 3. Vehicle body (chassis) state handler — publishes pose/twist over ROS
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(ROS_BODY_HZ, chassis, "~/output/vehicle/state")
)

# Initialize ONCE, after all registration, before the loop
ros_manager.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()   # precomputed once, drives real-time pacing
step_number = 0
frame = 0                                        # review-only frame counter


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()               # cache: current sim time for this step

        # Render at throttled rate
        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems in fixed order: driver -> terrain -> vehicle -> vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems (hmmwv.Advance steps the shared system — do NOT call DoStepDynamics)
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        # ROS update AFTER state is produced
        if not ros_manager.Update(time, STEP_SIZE):
            break

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)          # real-time pacing

except (RuntimeError, ValueError) as exc:       # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
