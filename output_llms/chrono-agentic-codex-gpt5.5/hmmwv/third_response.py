"""HMMWV rigid-terrain vehicle simulation with a custom delayed driver.

This PyChrono 9.0 NSC vehicle scene uses the catalog HMMWV_Full wrapper,
RigidTerrain contact, and a MyDriver subclass of veh.ChDriver. The vehicle waits
through an input delay, ramps throttle to 0.7, begins sinusoidal steering after
the delayed two-second mark, and exits when simulation time reaches four seconds.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === Tunable vehicle, terrain, and timing values kept explicit.
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 4.0
RENDER_FPS = 50.0
RENDER_STEPS = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
SPAWN_HEIGHT = 0.5
TIRE_RADIUS = 0.47
WHEEL_Z_TOL = 0.08

DRIVER_DELAY = 0.5
THROTTLE_RAMP_TIME = 0.2
MAX_THROTTLE = 0.7
STEERING_START = 2.0
STEERING_AMPLITUDE = 0.35
STEERING_FREQUENCY = 0.5


# === Custom driver === Scripted ChDriver subclass requested by the task.
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay

    def Synchronize(self, time):
        delayed_time = max(0.0, time - self.delay)

        if delayed_time <= 0.0:
            throttle = 0.0
        elif delayed_time < THROTTLE_RAMP_TIME:
            throttle = MAX_THROTTLE * delayed_time / THROTTLE_RAMP_TIME
        else:
            throttle = MAX_THROTTLE

        if delayed_time < STEERING_START:
            steering = 0.0
        else:
            steering = STEERING_AMPLITUDE * math.sin(
                2.0 * math.pi * STEERING_FREQUENCY * (delayed_time - STEERING_START)
            )

        self.SetThrottle(throttle)
        self.SetSteering(steering)
        self.SetBraking(0.0)


# === Vehicle and system === HMMWV wrapper owns the ChSystem and vehicle bodies.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, SPAWN_HEIGHT), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle_core = hmmwv.GetVehicle()  # cache: wrapper vehicle used for mass, terrain, and driver
chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for camera tracking and logs
# wheels, suspension links, steering links, driveline, and powertrain are created by HMMWV_Full.

print("VEHICLE MASS: ", vehicle_core.GetMass())

spindle_positions = [
    vehicle_core.GetSpindlePos(axle, side)
    for axle in range(vehicle_core.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
]
wheel_bottom_z = min(pos.z for pos in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= -WHEEL_Z_TOL, (
    f"wheel bottom z={wheel_bottom_z:.3f} is below rigid terrain beyond tolerance"
)

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === Rigid flat terrain provides contact support for the HMMWV.
terrain = veh.RigidTerrain(system)
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


# === Visualization === Vehicle-specific Irrlicht visualization follows the HMMWV.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV custom delayed driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)


# === Driver === MyDriver replaces the default interactive driver.
driver = MyDriver(vehicle_core, DRIVER_DELAY)
driver.Initialize()


# === Main loop === Synchronize and advance the full vehicle subsystem stack.
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:
    import traceback

    traceback.print_exc()
    raise
finally:
    pass
