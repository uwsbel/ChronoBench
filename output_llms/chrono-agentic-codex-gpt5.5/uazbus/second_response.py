"""UAZ bus double lane-change on rigid concrete terrain.

This PyChrono 9.0 NSC vehicle simulation initializes a UAZ bus at x = -40 m,
drives it across a flat rigid terrain patch textured with concrete, and applies
a scripted double lane-change maneuver followed by braking.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Values are named once so the vehicle setup, terrain, and loop share one source.
STEP_SIZE = 2.0e-3
SIM_END = 10.0
RENDER_STEP_SIZE = 1.0 / 50.0
RENDER_STEPS = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)  # precomputed once
TERRAIN_LENGTH = 220.0
TERRAIN_WIDTH = 80.0
INIT_LOCATION = chrono.ChVector3d(-40.0, 0.0, 0.5)
INIT_ROTATION = chrono.QUNIT


class DoubleLaneChangeDriver(veh.ChDriver):
    """Scripted driver that steers left-right-left, recenters, then brakes."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 1.0:
            steering = 0.0
            throttle = 0.35
            braking = 0.0
        elif time < 2.4:
            steering = 0.34
            throttle = 0.55
            braking = 0.0
        elif time < 3.8:
            steering = -0.34
            throttle = 0.55
            braking = 0.0
        elif time < 5.1:
            steering = 0.24
            throttle = 0.45
            braking = 0.0
        elif time < 6.8:
            steering = 0.0
            throttle = 0.45
            braking = 0.0
        elif time < 8.0:
            steering = 0.0
            throttle = 0.12
            braking = 0.35
        else:
            steering = 0.0
            throttle = 0.0
            braking = 0.85

        self.SetSteering(steering)
        self.SetThrottle(throttle)
        self.SetBraking(braking)


# === Vehicle ===
# The catalog UAZ wrapper owns the ChSystem; all terrain and visualization use it.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

bus = veh.UAZBUS()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(STEP_SIZE)
bus.Initialize()

system = bus.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = bus.GetChassisBody()  # cache: position and speed queried every step
vehicle = bus.GetVehicle()  # cache: passed to driver, visualization, and mass print
print("VEHICLE MASS: ", vehicle.GetMass())

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain ===
# A rigid NSC patch gives the bus a flat concrete road for the lane-change test.
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 80)
patch.SetColor(chrono.ChColor(0.75, 0.75, 0.72))
terrain.Initialize()


# === Driver ===
# The schedule is scored core because this demo uses scripted vehicle inputs.
driver = DoubleLaneChangeDriver(vehicle)
driver.Initialize()


# === Visualization ===
# Vehicle-aware Irrlicht renders chassis, wheels, terrain, HUD, and chase view.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZ Bus Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Review recording ===


# === Main loop ===
# Synchronize and advance the full vehicle stack once per step.
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver.Synchronize(time)
        driver_inputs = driver.GetInputs()  # cache: consumed by vehicle and HUD sync

        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        bus.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:
    print("Simulation failed during vehicle stepping:", exc)
    raise
finally:
    pass


# === Post-processing ===
