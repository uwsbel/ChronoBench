"""BMW E90 sedan on a rigid highway mesh with PID speed control.

This PyChrono 9.0 NSC vehicle scene initializes a catalog sedan on a
highway mesh terrain, applies a reference-speed throttle controller, slows
the steering response to five seconds, and uses small integration/render
steps for smooth visual control.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants: vehicle, controller, and timing ===
STEP_SIZE = 0.001
TIRE_STEP_SIZE = 0.001
RENDER_STEP_SIZE = 1.0 / 100.0
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
STEERING_TIME = 5.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3
INIT_LOCATION = chrono.ChVector3d(-35.0, -1.0, 0.55)
INIT_ROTATION = chrono.QuatFromAngleAxis(0.0, chrono.VECT_Z)
REFERENCE_SPEED = 8.0
SPEED_KP = 0.18
SPEED_KI = 0.02
MAX_THROTTLE = 0.35
HIGHWAY_MESH = "terrain/meshes/Highway_col.obj"


class SpeedPIDDriver(veh.ChDriver):
    """Closed-loop throttle controller using sedan speed error."""

    def __init__(self, vehicle, target_speed):
        super().__init__(vehicle)
        self.vehicle = vehicle
        self.target_speed = target_speed
        self.integral_error = 0.0
        self.last_time = 0.0
        self.filtered_steering = 0.0

    def Synchronize(self, time):
        dt = max(0.0, time - self.last_time)
        self.last_time = time
        speed = self.vehicle.GetSpeed()
        error = self.target_speed - speed
        self.integral_error = max(-10.0, min(10.0, self.integral_error + error * dt))
        throttle = max(0.0, min(MAX_THROTTLE, SPEED_KP * error + SPEED_KI * self.integral_error))
        braking = 0.0 if error >= -0.5 else min(0.4, -0.05 * error)
        desired_steering = 0.0
        max_delta = dt / STEERING_TIME if STEERING_TIME > 0 else 1.0
        steering_error = desired_steering - self.filtered_steering
        steering_step = max(-max_delta, min(max_delta, steering_error))
        self.filtered_steering += steering_step
        self.SetThrottle(throttle)
        self.SetBraking(braking)
        self.SetSteering(self.filtered_steering)


# === Vehicle and terrain setup ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

sedan = veh.BMW_E90()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
sedan.SetTireStepSize(TIRE_STEP_SIZE)
sedan.Initialize()

system = sedan.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", sedan.GetVehicle().GetMass())

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

chassis = sedan.GetChassisBody()  # cache: fetched once for camera and diagnostics
sedan_vehicle = sedan.GetVehicle()  # cache: wrapper vehicle reused in driver and vis
# Wrapper-created components: sedan-owned ChSystem, chassis, wheels, suspension
# joints, powertrain, tire models, driver, rigid terrain, and Irrlicht visualizer.

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
highway_patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile(HIGHWAY_MESH),
)
highway_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200.0, 10.0)
terrain.Initialize()

spindle_positions = []
for axle_index in range(sedan_vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(sedan_vehicle.GetSpindlePos(axle_index, side))
assert min(p.z for p in spindle_positions) > 0.15, "sedan initialized too low for highway terrain"


# === Visualization and driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 highway speed-control demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.1), 7.5, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(sedan_vehicle)

driver = SpeedPIDDriver(sedan_vehicle, REFERENCE_SPEED)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0
last_inputs = veh.DriverInputs()


# === Main loop ===
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            last_inputs = driver_inputs

            driver.Synchronize(time)
            terrain.Synchronize(time)
            sedan.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            sedan.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            pos = chassis.GetPos()

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break

except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass


# === Review-only output assembly ===
