"""UAZ bus rigid-terrain simulation using NSC contact.

The script initializes the catalog UAZ bus at x=-40 m on a flat concrete road,
then applies a scripted double lane-change maneuver followed by braking.  The
main bodies are the wrapper-created bus chassis, suspension, wheels, tires, and
the rigid terrain patch; the expected behavior is forward motion with alternating
steering inputs and a controlled stop.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants === simulation parameters and maneuver schedule
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 14.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 220.0
TERRAIN_WIDTH = 80.0
INIT_POS = chrono.ChVector3d(-40.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
MAX_STEERING = 0.45
CRUISE_THROTTLE = 0.55
BRAKE_START = 7.5


class DoubleLaneChangeDriver(veh.ChDriver):
    """Scripted driver that steers left-right-left and then brakes."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 1.0:
            steering = 0.0
            throttle = 0.35
            braking = 0.0
        elif time < 2.5:
            steering = MAX_STEERING
            throttle = CRUISE_THROTTLE
            braking = 0.0
        elif time < 4.0:
            steering = -MAX_STEERING
            throttle = CRUISE_THROTTLE
            braking = 0.0
        elif time < 5.5:
            steering = MAX_STEERING * 0.75
            throttle = CRUISE_THROTTLE
            braking = 0.0
        elif time < BRAKE_START:
            steering = 0.0
            throttle = CRUISE_THROTTLE
            braking = 0.0
        else:
            steering = 0.0
            throttle = 0.0
            braking = 0.8

        self.SetSteering(steering)
        self.SetThrottle(throttle)
        self.SetBraking(braking)


# === Vehicle And Terrain === catalog wrapper owns the ChSystem and rigid bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused throughout
chassis = vehicle.GetChassisBody()  # cache: chassis body reused for logs and camera target
veh_core = vehicle.GetVehicle()  # cache: vehicle interface reused for mass and wheel checks
# Wrapper-created components: system, chassis, suspension, steering links, wheels, tires, and driveline.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", veh_core.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 40)
patch.SetColor(chrono.ChColor(0.65, 0.65, 0.65))
terrain.Initialize()

spindle_positions = []
for axle_id in range(veh_core.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_core.GetSpindlePos(axle_id, side))
tire_radius = veh_core.GetAxle(0).m_wheels[0].GetTire().GetRadius()  # cache: tire size for spawn check
wheel_bottom_z = min(pos.z for pos in spindle_positions) - tire_radius
assert wheel_bottom_z >= -0.05, f"UAZ bus starts below the concrete road: wheel bottom z={wheel_bottom_z:.3f}"


# === Visualization And Driver === Irrlicht vehicle window with scripted inputs
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZ bus double lane change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_core)

driver = DoubleLaneChangeDriver(veh_core)
driver.Initialize()


# === Main Loop === synchronize and advance driver, terrain, vehicle, and renderer
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:
    print(f"Simulation failed during vehicle integration: {exc}")
    traceback.print_exc()
    raise
finally:
    pass
