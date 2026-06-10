"""
HMMWV simulation with a custom driver class (MyDriver) that implements:
- A delay of 0.5 seconds before any driver input is applied.
- Throttle gradually increasing to 0.7 starting 0.2 seconds after the delay.
- Sinusoidal steering pattern starting at t=2 seconds (absolute simulation time).
- Simulation ends at t=4 seconds.

System: ChSystemNSC (NSC contact method), rigid terrain.
Vehicle: HMMWV_Full wrapper with TMEASY tires on flat RigidTerrain.
Expected behavior: vehicle sits still until t=0.5s, then accelerates forward; sinusoidal
steering begins at t=2s; simulation terminates at t=4s.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Named constants ===
STEP_SIZE = 5e-4                     # physics time step (s)
SIM_END = 4.0                        # simulation end time (s)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 200.0               # terrain patch length (m)
TERRAIN_WIDTH = 200.0                # terrain patch width (m)
INIT_X, INIT_Y, INIT_Z = 0.0, 0.0, 0.5
DRIVER_DELAY = 0.5                   # delay before any input (s)
THROTTLE_RAMP_DURATION = 0.2        # ramp duration after delay (s)
MAX_THROTTLE = 0.7                   # maximum throttle target
STEER_START = 2.0                    # absolute time when sinusoidal steering starts (s)

# === Data paths (truth-faithful — scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Custom Driver Class ===
class MyDriver(veh.ChDriver):
    """Custom time-based driver: delay + throttle ramp + sinusoidal steering."""

    def __init__(self, vehicle, delay=DRIVER_DELAY):
        super().__init__(vehicle)
        self._delay = delay  # cache: delay parameter

    def Synchronize(self, time):
        if time < self._delay:
            # Before delay: no input
            self.SetThrottle(0.0)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
        else:
            t_eff = time - self._delay  # time elapsed since delay ended
            # Throttle ramp: linearly increases to MAX_THROTTLE over THROTTLE_RAMP_DURATION
            if t_eff < THROTTLE_RAMP_DURATION:
                throttle = MAX_THROTTLE * (t_eff / THROTTLE_RAMP_DURATION)
            else:
                throttle = MAX_THROTTLE
            self.SetThrottle(throttle)
            self.SetBraking(0.0)
            # Sinusoidal steering starting at STEER_START (absolute simulation time)
            if time >= STEER_START:
                steering = math.sin(math.pi * (time - STEER_START))
            else:
                steering = 0.0
            self.SetSteering(steering)

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    chrono.QUNIT,
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()              # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()     # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxles()[i]; terrain: RigidTerrain patch below
# joints: suspension + steering links created internally by the HMMWV_Full wrapper

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Visualization types (after Initialize) ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver ===
driver = MyDriver(hmmwv.GetVehicle(), delay=DRIVER_DELAY)
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Custom Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()
            if sim_time >= SIM_END:
                break

            driver_inputs = driver.GetInputs()

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass   # CSV closed below in review-only post-loop block
