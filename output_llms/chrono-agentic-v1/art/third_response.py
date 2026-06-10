"""
ARTcar wheeled-vehicle simulation on rigid terrain with updated motor parameters.

System type: ChSystemNSC (non-smooth contact), owned by the veh.ARTcar wrapper.
Main bodies: ARTcar chassis, 4 wheel assemblies (owned by wrapper), RigidTerrain patch.

Expected behavior: The ARTcar drives forward faster than the baseline because:
  - MaxMotorVoltageRatio updated from 0.16 to 0.26 (higher motor voltage ratio)
  - StallTorque updated from 0.3 to 0.4 Nm (higher peak drive torque)
  - TireRollingResistance updated from 0.06 to 0.03 (lower rolling losses)
These three combined changes make the vehicle noticeably faster on flat terrain.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh

import pychrono.irrlicht as chronoirr   # for vis

# === Constants ===
# Simulation parameters — precomputed once
TIME_STEP         = 1e-3
TIRE_STEP_SIZE    = TIME_STEP
SIM_END           = 15.0
RENDER_FPS        = 50.0
RENDER_STEP_SIZE  = 1.0 / RENDER_FPS                              # precomputed once
RENDER_STEPS      = math.ceil(RENDER_STEP_SIZE / TIME_STEP)       # precomputed once

# Updated motor and tire parameters (the key changes in this turn)
MAX_MOTOR_VOLTAGE_RATIO = 0.26    # updated from 0.16
STALL_TORQUE            = 0.4     # updated from 0.3  (Nm)
TIRE_ROLLING_RESISTANCE = 0.03    # updated from 0.06

# Vehicle spawn
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w,x,y,z)

# Chase camera target on chassis
TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 0.2)

# Terrain size
TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH  = 100.0

# === ARTcar vehicle setup ===
# veh.ARTcar() owns its own ChSystemNSC internally — do NOT pass a pre-created system.
car = veh.ARTcar()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(veh.CollisionType_NONE)
car.SetChassisFixed(False)             # MANDATORY — fixed chassis never moves
car.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
car.SetTireType(veh.TireModelType_TMEASY)
car.SetTireStepSize(TIRE_STEP_SIZE)
car.SetMaxMotorVoltageRatio(MAX_MOTOR_VOLTAGE_RATIO)
car.SetStallTorque(STALL_TORQUE)
car.SetTireRollingResistance(TIRE_ROLLING_RESISTANCE)

car.Initialize()

# Set visualization types for all vehicle subsystems (must be after Initialize)
# Note: VisualizationType lives in pychrono.vehicle in this 9.0.0 build
car.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === System & bodies (created by the veh.ARTcar wrapper) ===
# Get the system owned by the wrapper; set collision system (REQUIRED for wheel/terrain contact).
system = car.GetSystem()              # ChSystemNSC owned by ARTcar wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis_body = car.GetChassisBody()   # cache: main chassis body, reused each step for logging
# wheels/suspension/tires: created internally by car.Initialize()
# terrain: RigidTerrain patch added below

# === Terrain (rigid flat ground) ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    TERRAIN_LENGTH, TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver (ChDataDriver — open-loop schedule matching the catalog-vehicle demo) ===
# Ramp up throttle at t=0.5 s, then hold constant to observe the faster speed.
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.1, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.0, 1.0, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, 1.0, 0.0),
])
driver = veh.ChDataDriver(car.GetVehicle(), driver_data)
driver.Initialize()

# === Visualization (wheeled vehicle Irrlicht with chase camera) ===
# ChWheeledVehicleVisualSystemIrrlicht for ARTcar.
# SetChaseCamera + window config BEFORE Initialize; scene elements AFTER.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar — Updated Motor Parameters (VoltageRatio=0.26, StallTorque=0.4, Rolling=0.03)")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(TRACK_POINT, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(car.GetVehicle())

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame       = 0

try:
    while vis.Run():
        sim_time = system.GetChTime()


        # Throttled rendering — BeginScene/Render/EndScene once per render frame
        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs and synchronize all subsystems
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        car.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        car.Advance(TIME_STEP)     # advances the wrapper-owned system internally
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
