"""
HMMWV simulation with a custom driver class (MyDriver).

System type: NSC (ChSystemNSC owned by HMMWV_Full wrapper)
Vehicle: HMMWV Full model on flat RigidTerrain
Driver: MyDriver subclass of veh.ChDriver — delayed start (0.5 s),
        throttle ramps to 0.7 after 0.2 s of the driver's active window,
        sinusoidal steering pattern starting at 2 s of sim time.
Simulation ends when time reaches 4 seconds.

Expected behavior: Vehicle stands still for the first 0.5 s (driver delay),
then accelerates forward with gradually increasing throttle and begins
sinusoidal steering from t=2 s until the sim ends at t=4 s.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Data paths (truth-mandatory for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Constants ===
step_size = 1e-3          # physics time step [s]
sim_end = 4.0             # simulation end time [s]  (prompt requirement)
render_fps = 50.0
render_steps = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH  = 200.0
INIT_X, INIT_Y, INIT_Z = 0.0, 0.0, 0.5

# === Custom Driver Class ===
class MyDriver(veh.ChDriver):
    """Custom scripted driver with a delay, throttle ramp, and sinusoidal steering."""

    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self._delay = delay  # seconds before any input is applied

    def Synchronize(self, time):
        # No input before the delay expires
        if time < self._delay:
            self.SetThrottle(0.0)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
            return

        # Time elapsed since the driver became active
        t_active = time - self._delay

        # Throttle: ramp linearly from 0 to 0.7 over 0.2 s, then hold
        if t_active < 0.2:
            throttle = 0.7 * (t_active / 0.2)
        else:
            throttle = 0.7
        self.SetThrottle(throttle)
        self.SetBraking(0.0)

        # Steering: sinusoidal pattern starting at sim time 2 s
        if time >= 2.0:
            steering = math.sin(2.0 * math.pi * 0.5 * (time - 2.0))
        else:
            steering = 0.0
        self.SetSteering(steering)


# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    chrono.ChQuaterniond(1, 0, 0, 0)
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                          # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED
chassis = hmmwv.GetChassisBody()                    # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetAxle(i); suspension + steering joints created by wrapper

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Visualization types ===
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver (custom MyDriver with 0.5 s delay) ===
driver = MyDriver(hmmwv.GetVehicle(), delay=0.5)
driver.Initialize()

# === Visualization — full Irrlicht window ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — Custom Driver with Delay and Sinusoidal Steering")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()  # cache: fetched once per outer step

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
