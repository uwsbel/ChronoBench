"""HMMWV circular-path follower on rigid NSC terrain.

This standalone PyChrono 9.0 simulation uses the HMMWV_Full vehicle wrapper,
flat rigid terrain, and a constant-throttle path driver whose PID steering
controller follows a circular path. Two sphere markers show the steering
controller sentinel and target points while the HMMWV drives around the course.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 12.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
PATH_RADIUS = 35.0
PATH_RUN = 5.0
PATH_TURNS = 2
CONSTANT_THROTTLE = 0.3
STEERING_LOOKAHEAD = 5.0
STEERING_KP = 0.8
STEERING_KI = 0.0
STEERING_KD = 0.0
MARKER_RADIUS = 0.8
MARKER_Z = 1.2


class ConstantThrottlePathDriver(veh.ChDriver):
    """Path steering with fixed throttle so throttle remains exactly scripted."""

    def __init__(self, vehicle, path, step_size):
        super().__init__(vehicle)
        self.vehicle = vehicle
        self.step_size = step_size
        self.steering_controller = veh.ChPathSteeringController(path)
        self.steering_controller.SetLookAheadDistance(STEERING_LOOKAHEAD)
        self.steering_controller.SetGains(STEERING_KP, STEERING_KI, STEERING_KD)

    def Synchronize(self, time):
        steering = self.steering_controller.Advance(
            self.vehicle.GetRefFrame(), time, self.step_size
        )
        self.SetSteering(max(-1.0, min(1.0, steering)))
        self.SetThrottle(CONSTANT_THROTTLE)
        self.SetBraking(0.0)


def make_marker(system, name, color):
    marker = chrono.ChBody()
    marker.SetName(name)
    marker.SetFixed(True)
    sphere = chrono.ChVisualShapeSphere(MARKER_RADIUS)
    sphere.SetColor(color)
    marker.AddVisualShape(sphere, chrono.ChFramed())
    system.AddBody(marker)
    return marker


# === Vehicle and system ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: ChVehicle handle reused by driver, vis, and logging
chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for review logging
# Wrapper-created components: chassis, suspensions, steering links, wheels, tires,
# powertrain, and joints are owned by veh.HMMWV_Full; terrain and markers are added below.
print("VEHICLE MASS: ", vehicle.GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Path driver and markers ===
path = veh.CirclePath(INIT_LOC, PATH_RADIUS, PATH_RUN, True, PATH_TURNS)
driver = ConstantThrottlePathDriver(vehicle, path, STEP_SIZE)
driver.Initialize()
steering_controller = driver.steering_controller  # cache: controller supplies steering and marker locations

sentinel_marker = make_marker(system, "sentinel_marker", chrono.ChColor(0.0, 0.2, 1.0))
target_marker = make_marker(system, "target_marker", chrono.ChColor(1.0, 0.0, 0.0))


# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Circular Path Follower")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Review-only outputs ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            sentinel = steering_controller.GetSentinelLocation()
            target = steering_controller.GetTargetLocation()
            sentinel_marker.SetPos(chrono.ChVector3d(sentinel.x, sentinel.y, MARKER_Z))
            target_marker.SetPos(chrono.ChVector3d(target.x, target.y, MARKER_Z))

            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    print(f"simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:  # output path or frame capture failure
    print(f"output failed: {exc}")
    raise
finally:
    pass


# === Post-processing ===
