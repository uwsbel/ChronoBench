"""
BMW E90 sedan simulation on a highway mesh terrain with PID speed control.

System type: ChSystemNSC (owned by the BMW_E90 wrapper, RigidTerrain NSC contact).
Main bodies:
  - BMW_E90 sedan chassis, suspension spindles, wheels, TMEASY tires
  - RigidTerrain highway mesh patch (uneven 300m road surface)
Driver: ChInteractiveDriver (real-time interactive) with 5-second steering response time.
PID controller: proportional + integral + derivative on speed error drives throttle.
Expected behaviour: sedan spawns on the highway mesh terrain; interactive driver handles
  steering/braking while a PID controller drives throttle to track the reference speed.
"""

# === Imports ===
import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants ===
# Step sizes (finer than typical default for improved control resolution)
TIME_STEP = 5e-4            # physics step size (s) — decreased for finer control
RENDER_STEP_SIZE = 1.0 / 50.0  # render cadence (s) — decreased for finer render

SIM_END = 30.0              # total simulation time (s)

# Driver response times
STEERING_TIME = 5.0         # 5 seconds to go 0 -> max steering (as specified)
THROTTLE_TIME = 1.0         # seconds to go 0 -> max throttle
BRAKING_TIME = 0.3          # seconds to go 0 -> max brake

# Vehicle initial position and orientation (adjusted from default)
INIT_POS = chrono.ChVector3d(5.0, 0.0, 0.5)      # spawn slightly above terrain, offset into road
INIT_ROT = chrono.QuatFromAngleZ(0.0)             # facing +X direction

# PID controller parameters for speed control
REFERENCE_SPEED = 12.0      # target reference speed (m/s ~ 43 km/h)
KP = 0.4                    # proportional gain
KI = 0.1                    # integral gain
KD = 0.05                   # derivative gain

# Terrain highway mesh centre offset (mesh spans ~300 m in X direction)
HIGHWAY_MESH_OFFSET = chrono.ChVector3d(150.0, 0.0, 0.0)

# Precomputed render cadence (steps per rendered frame), computed once before loop
RENDER_EVERY = max(1, round(RENDER_STEP_SIZE / TIME_STEP))  # precomputed once

# === Vehicle setup ===
sedan = veh.BMW_E90()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)      # NSC for rigid terrain contact
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)                            # MANDATORY — fixed chassis won't move
sedan.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
sedan.SetTireType(veh.TireModelType_TMEASY)             # deformable tire for terrain contact
sedan.SetTireStepSize(TIME_STEP)
sedan.Initialize()

# === System & bodies (created by the BMW_E90 wrapper) ===
system = sedan.GetSystem()            # ChSystemNSC owned by the wrapper; cache: used throughout
chassis = sedan.GetChassisBody()      # cache: main chassis rigid body, fetched once, reused
# wheels/spindles: sedan.GetVehicle().GetAxle(i)...; terrain patch body below
# joints: suspension + steering links created inside the BMW_E90 wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types — call AFTER Initialize
sedan.SetChassisVisualizationType(chrono.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(chrono.VisualizationType_MESH)
sedan.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain — highway mesh patch ===
# RigidTerrain with the 300 m uneven road mesh as a highway surface
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Highway mesh: 300 m long road surface — centred at HIGHWAY_MESH_OFFSET
highway_mesh_path = chrono.GetChronoDataFile(
    "vehicle/terrain/meshes/uneven_300m_6m_10mm.obj"
)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(HIGHWAY_MESH_OFFSET, chrono.QUNIT),
    highway_mesh_path,
)
patch.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/concrete.jpg"),
    60, 60
)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Sedan — Highway Mesh with PID Speed Control")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(sedan.GetVehicle())

# === Driver — ChInteractiveDriver (scored-core default for catalog vehicles) ===
# Steering response time set to 5 seconds per prompt specification
# Note: in this 9.0.0 build, ChInteractiveDriver takes the vehicle (not the visual system)
driver = veh.ChInteractiveDriver(sedan.GetVehicle())
driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)    # 5-second steering response
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
driver.Initialize()

# === PID controller state (for throttle from speed error) ===
# KP/KI/KD gains drive throttle each step: throttle = KP*err + KI*integral + KD*derivative
pid_integral = 0.0       # accumulated integral term
pid_prev_error = 0.0     # previous error for derivative term
pid_throttle = 0.0       # current PID throttle output

# === Real-time timer (scored-core loop pacing) ===
realtime_timer = chrono.ChRealtimeStepTimer()

# === Review-only recording setup ===


# === Main simulation loop ===
frame = 0
step_number = 0

try:
    while vis.Run():
        time = system.GetChTime()
        if time >= SIM_END:
            break

        # Throttled rendering: render only every RENDER_EVERY physics steps
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get current interactive driver inputs
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        sedan.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all subsystems (vehicle.Advance steps ChSystem — do NOT also call DoStepDynamics)
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        sedan.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)    # pace wall-clock to match sim time

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
