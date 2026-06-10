"""Gator vehicle demo with primitive visualization and simple chassis contact.

This PyChrono NSC simulation builds a John Deere Gator wrapper on a rigid
terrain patch. The vehicle uses primitive visual assets, primitive chassis
collision, and a less responsive keyboard driver so steering, throttle, and
braking commands ramp in more slowly during interactive control.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants === named simulation values make the vehicle setup reproducible
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once
TERRAIN_LENGTH = 60.0
TERRAIN_WIDTH = 20.0
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.4)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
STEERING_RESPONSE_TIME = 4.0
THROTTLE_RESPONSE_TIME = 4.0
BRAKING_RESPONSE_TIME = 1.2


# === Vehicle and terrain === wrapper owns the system, chassis, wheels, and driveline
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
gator.SetTireType(veh.TireModelType_RIGID)
gator.SetTireStepSize(TIRE_STEP_SIZE)
gator.Initialize()

system = gator.GetSystem()  # cache: wrapper-owned ChSystemNSC reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = gator.GetChassisBody()  # cache: main rigid chassis body reused for logging
vehicle = gator.GetVehicle()  # cache: wrapper vehicle handle reused by vis and drivers
# Wrapper-created components: chassis, axles, suspension, steering, tires, driveline, and system.
print("VEHICLE MASS: ", vehicle.GetMass())

gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetTireCollisionType(veh.CollisionType_PRIMITIVES)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

system.GetCollisionSystem().BindAll()


# === Visualization and driver === Irrlicht window is created before the keyboard driver
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Primitive Visualization")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.8), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_RESPONSE_TIME)
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_RESPONSE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_RESPONSE_TIME)
driver.Initialize()


# === Main loop === real-time subsystem synchronization and vehicle advancement
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        for _ in range(RENDER_STEPS):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(time)
            terrain.Synchronize(time)
            gator.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)
            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            gator.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid Chrono state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # output path or disk failure during review logging
    traceback.print_exc()
    raise
finally:
    pass
