"""
HMMWV simulation with a custom driver class (MyDriver).

System: ChSystemNSC (owned by the HMMWV_Full wrapper).
Main bodies: HMMWV chassis, 4 wheel spindles, rigid terrain patch.
Custom driver: MyDriver inherits veh.ChDriver and overrides Synchronize to
  apply a 0.5s startup delay, a throttle ramp to 0.7 after 0.2s, and a
  sinusoidal steering pattern starting at 2s.
Simulation ends when sim time reaches 4 seconds.
"""

# === Imports ===
import math
import os
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
STEP_SIZE    = 1e-3          # physics time step (s)
SIM_END      = 4.0           # simulation end time (s) — per prompt
RENDER_FPS   = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH  = 200.0

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)   # spawn above terrain
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)    # facing +X

DRIVER_DELAY   = 0.5   # seconds before any driver input applies
THROTTLE_DELAY = 0.2   # ramp-up duration after delay ends (s)
MAX_THROTTLE   = 0.7   # target throttle — per prompt
STEER_START    = 2.0   # wall time at which sinusoidal steering begins (s)
STEER_AMP      = 0.5   # sinusoidal steering amplitude (rad fraction)
STEER_FREQ     = 0.5   # steering oscillation frequency (Hz)


# === Custom Driver Class ===
class MyDriver(veh.ChDriver):
    """Custom driver with delayed inputs, throttle ramp, and sinusoidal steering.

    Parameters
    ----------
    vehicle_obj : veh.ChVehicle  -- the vehicle object (from hmmwv.GetVehicle())
    delay : float                -- seconds to wait before any control input is applied
    """

    def __init__(self, vehicle_obj, delay: float = 0.5):
        super().__init__(vehicle_obj)
        self._delay = delay

    def Synchronize(self, time: float) -> None:
        """Override: set throttle, steering, braking as a function of simulation time."""
        effective_time = time - self._delay
        if effective_time <= 0.0:
            # Delay period: no input
            self.SetThrottle(0.0)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
            return

        # Throttle ramp: gradually increase to MAX_THROTTLE over THROTTLE_DELAY seconds
        throttle = min(MAX_THROTTLE, MAX_THROTTLE * effective_time / THROTTLE_DELAY)
        self.SetThrottle(throttle)
        self.SetBraking(0.0)

        # Sinusoidal steering starting at STEER_START wall-clock seconds
        if time >= STEER_START:
            steering = STEER_AMP * math.sin(2.0 * math.pi * STEER_FREQ * (time - STEER_START))
            self.SetSteering(steering)
        else:
            self.SetSteering(0.0)


# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()          # ChSystemNSC owned by the wrapper  # cache: fetched once
chassis = hmmwv.GetChassisBody()     # main chassis rigid body            # cache: fetched once
# wheels/spindles: hmmwv.GetVehicle().GetAxles()
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types (after Initialize)
vis_type = chrono.VisualizationType_MESH
hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver ===
driver = MyDriver(hmmwv.GetVehicle(), delay=DRIVER_DELAY)
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Custom Driver (MyDriver)")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())
vis.AttachDriver(driver)

# === Real-time step timer ===
realtime_timer = chrono.ChRealtimeStepTimer()

# === Main loop ===
frame       = 0
step_number = 0


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()  # cache: fetched once per outer step

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize: driver -> terrain -> vehicle -> vis
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        # Advance
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)      # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # teardown placeholder
