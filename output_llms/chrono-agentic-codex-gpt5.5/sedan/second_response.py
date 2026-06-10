"""Two BMW E90 sedans on rigid concrete terrain using NSC contact.

The scene builds two catalog sedan vehicles in one shared Chrono system, places
them with distinct initial poses, gives each vehicle its own scripted driver, and
applies sinusoidal steering inputs while both vehicles drive across a concrete
rigid terrain patch.
"""

# === Imports and constants ===
import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_STEPS = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 300.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9

PRIMARY_POS = chrono.ChVector3d(-8.0, -1.75, 0.50)
SECONDARY_POS = chrono.ChVector3d(-12.0, 1.75, 0.50)
PRIMARY_ROT = chrono.QUNIT
SECONDARY_ROT = chrono.QuatFromAngleZ(0.08)

STEERING_AMPLITUDE = 0.22
STEERING_FREQUENCY = 0.45
THROTTLE = 0.32


class SineSteerDriver(veh.ChDriver):
    """Scripted sedan driver with constant throttle and sinusoidal steering."""

    def __init__(self, vehicle, amplitude, frequency, phase):
        super().__init__(vehicle)
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase

    def Synchronize(self, time):
        steering = self.amplitude * math.sin(2.0 * math.pi * self.frequency * time + self.phase)
        self.SetSteering(steering)
        self.SetThrottle(THROTTLE)
        self.SetBraking(0.0)


# === Vehicle setup ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

sedan = veh.BMW_E90()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(PRIMARY_POS, PRIMARY_ROT))
sedan.SetTireType(veh.TireModelType_TMEASY)
sedan.SetTireStepSize(TIRE_STEP_SIZE)
sedan.Initialize()

system = sedan.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", sedan.GetVehicle().GetMass())

second_sedan = veh.BMW_E90(system)
second_sedan.SetContactMethod(chrono.ChContactMethod_NSC)
second_sedan.SetChassisCollisionType(veh.CollisionType_NONE)
second_sedan.SetChassisFixed(False)
second_sedan.SetInitPosition(chrono.ChCoordsysd(SECONDARY_POS, SECONDARY_ROT))
second_sedan.SetTireType(veh.TireModelType_TMEASY)
second_sedan.SetTireStepSize(TIRE_STEP_SIZE)
second_sedan.Initialize()
print("SECOND VEHICLE MASS: ", second_sedan.GetVehicle().GetMass())

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

second_sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
second_sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
second_sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
second_sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
second_sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System-owned bodies and vehicle handles ===
primary_vehicle = sedan.GetVehicle()  # cache: wrapper vehicle reused by driver and visualization
secondary_vehicle = second_sedan.GetVehicle()  # cache: second wrapper vehicle reused by driver
primary_chassis = sedan.GetChassisBody()  # cache: chassis body logged every step
secondary_chassis = second_sedan.GetChassisBody()  # cache: second chassis body logged every step
# wrapper-created components: chassis bodies, suspensions, steering links, wheels, tires, and driveline.


# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 80)
patch.SetColor(chrono.ChColor(0.55, 0.55, 0.55))
terrain.Initialize()


# === Drivers ===
driver = SineSteerDriver(primary_vehicle, STEERING_AMPLITUDE, STEERING_FREQUENCY, 0.0)
driver.Initialize()
second_driver = SineSteerDriver(secondary_vehicle, STEERING_AMPLITUDE, STEERING_FREQUENCY, math.pi)
second_driver.Initialize()


# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Two BMW E90 Sedans on Concrete")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 12.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(primary_vehicle)
vis.AttachVehicle(secondary_vehicle)


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:

    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()
        second_driver_inputs = second_driver.GetInputs()

        driver.Synchronize(time)
        second_driver.Synchronize(time)
        terrain.Synchronize(time)
        sedan.Synchronize(time, driver_inputs, terrain)
        second_sedan.Synchronize(time, second_driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        second_driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        sedan.Advance(STEP_SIZE)
        second_sedan.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (OSError, IOError) as exc:  # disk or permission errors during runtime file handling
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid numerical state
    traceback.print_exc()
    raise
finally:
    pass
