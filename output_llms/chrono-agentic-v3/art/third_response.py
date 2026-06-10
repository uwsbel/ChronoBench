"""
ARTcar on RigidTerrain — faster configuration.

System type: ChSystemNSC (NSC, created internally by the ARTcar wrapper).
Vehicle: veh.ARTcar — a small electric car with configurable motor/tire params.
Terrain: veh.RigidTerrain flat patch.
Driver: veh.ChInteractiveDriverIRR (real-time interactive, scored-core default).

Motor and tire parameters updated for increased performance:
  - MaxMotorVoltageRatio: 0.26 (higher motor authority)
  - StallTorque: 0.4 Nm (higher low-speed torque)
  - TireRollingResistance: 0.03 (reduced rolling drag)

Expected behavior: the ARTcar accelerates more readily and reaches higher top
speed on the flat terrain compared to the baseline configuration.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Named constants ===
STEP_SIZE = 5e-4           # integration step [s]
SIM_END   = 20.0           # simulation end time [s]
RENDER_FPS = 50.0          # Irrlicht render rate [Hz]
TERRAIN_LENGTH = 300.0     # terrain patch length [m]
TERRAIN_WIDTH  = 300.0     # terrain patch width [m]
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)   # chassis spawn position
INIT_ROT = chrono.QuatFromAngleZ(0.0)          # chassis spawn orientation

# Motor / tire parameters from the delta prompt
MAX_MOTOR_VOLTAGE_RATIO = 0.26   # was 0.16 — higher motor authority
STALL_TORQUE            = 0.4    # was 0.3 Nm — higher low-speed torque
TIRE_ROLLING_RESISTANCE = 0.03   # was 0.06 — reduced rolling drag

render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# === Data paths (scored-core requirement for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle ===
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)
artcar.SetChassisCollisionType(veh.CollisionType_NONE)
artcar.SetChassisFixed(False)                       # MANDATORY — fixed chassis won't move
artcar.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
artcar.SetTireType(veh.TireModelType_RIGID)
artcar.SetTireStepSize(STEP_SIZE)
# Motor / tire performance parameters (delta from turn prompt)
artcar.SetMaxMotorVoltageRatio(MAX_MOTOR_VOLTAGE_RATIO)
artcar.SetStallTorque(STALL_TORQUE)
artcar.SetTireRollingResistance(TIRE_ROLLING_RESISTANCE)
artcar.Initialize()

# === System & bodies (created by the veh.ARTcar wrapper) ===
sys = artcar.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = artcar.GetChassisBody()         # cache: fetched once, reused every step
# Wheels/spindles: artcar.GetVehicle().GetAxles(); terrain body below
# Joints: suspension + steering links created inside the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize

print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())

# === Visualization types ===
artcar.SetChassisVisualizationType(veh.VisualizationType_MESH)
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(veh.VisualizationType_MESH)
artcar.SetTireVisualizationType(veh.VisualizationType_MESH)

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
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar — faster configuration")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()          # vehicle truths use directional light
vis.AttachVehicle(artcar.GetVehicle())

# === Driver (ChInteractiveDriverIRR — scored-core default for catalog vehicles) ===
steering_time  = 1.0
throttle_time  = 1.0
braking_time   = 0.3
render_step_size = 1.0 / RENDER_FPS  # precomputed once

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and artcar.GetSystem().GetChTime() < SIM_END:
        time = artcar.GetSystem().GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        artcar.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        artcar.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)


except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback; traceback.print_exc()
    raise
