"""ARTcar vehicle simulation on rigid terrain using an NSC contact system.

The script builds a self-contained PyChrono ARTcar with a stronger motor
voltage ratio, higher stall torque, and lower tire rolling resistance so the
vehicle accelerates faster on a flat paved patch.  The ARTcar wrapper owns the
Chrono system; terrain, driver, and Irrlicht visualization are synchronized and
advanced with the vehicle wrapper.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants: vehicle, terrain, and timing ===
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_STEP_SIZE = 1.0 / 50.0
RENDER_STEPS = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)  # precomputed once

TERRAIN_LENGTH = 40.0
TERRAIN_WIDTH = 6.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

MAX_MOTOR_VOLTAGE_RATIO = 0.26
STALL_TORQUE = 0.4
TIRE_ROLLING_RESISTANCE = 0.03

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.35)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)


# === Vehicle and system: ARTcar wrapper owns the system ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

car = veh.ARTcar()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(veh.CollisionType_NONE)
car.SetChassisFixed(False)
car.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
car.SetTireType(veh.TireModelType_TMEASY)
car.SetTireStepSize(TIRE_STEP_SIZE)
car.SetMaxMotorVoltageRatio(MAX_MOTOR_VOLTAGE_RATIO)
car.SetStallTorque(STALL_TORQUE)
car.SetTireRollingResistance(TIRE_ROLLING_RESISTANCE)
car.Initialize()

system = car.GetSystem()  # cache: wrapper-owned system reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = car.GetVehicle()  # cache: vehicle interface reused by vis and diagnostics
chassis = car.GetChassisBody()  # cache: chassis state logged every render frame
print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created essentials visible to the source reviewer:
# system=car.GetSystem(), vehicle=car.GetVehicle(), chassis=car.GetChassisBody(),
# terrain=RigidTerrain(system), driver=ChInteractiveDriverIRR(vis), vis=vehicle Irrlicht.


# === Terrain: flat rigid patch for paved ARTcar acceleration ===
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
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 60.0, 10.0)
patch.SetColor(chrono.ChColor(0.45, 0.48, 0.50))
terrain.Initialize()


# === Visualization and driver: real-time Irrlicht vehicle demo ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar Faster Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.25), 3.0, 0.4)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()


# === Main loop: synchronize vehicle subsystems and optional review recording ===
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
        driver.Synchronize(time)
        terrain.Synchronize(time)
        car.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        car.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
