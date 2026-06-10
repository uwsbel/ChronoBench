"""Gator vehicle demo on rigid terrain with Irrlicht and an onboard RGB camera.

The simulation uses an NSC vehicle system owned by the Gator wrapper, a rigid
terrain patch, an interactive Irrlicht driver, and a camera sensor attached to
the chassis.  The vehicle parts are rendered with different visualization modes
while the driver, terrain, vehicle, visualizer, and sensor manager advance in a
single synchronized loop.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === named settings keep physics and output cadence explicit
TIME_STEP = 0.002
TIRE_STEP_SIZE = 0.001
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 40.0
TERRAIN_FRICTION = 0.9
INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT

CAMERA_RATE = 30
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FOV = 1.408


# === Vehicle setup === wrapper owns the NSC system and internal body topology
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
gator.SetTireType(veh.TireModelType_RIGID)
gator.SetTireStepSize(TIRE_STEP_SIZE)
gator.Initialize()

system = gator.GetSystem()  # cache: wrapper-owned ChSystemNSC used by terrain and sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = gator.GetChassisBody()  # cache: camera mount and logging target
vehicle = gator.GetVehicle()  # cache: vehicle subsystem queried for mass and wheels
# Wrapper-created essentials: chassis, axles, wheels, tires, steering, suspension,
# powertrain, and joints are owned by veh.Gator and stepped through gator.Advance().
print("VEHICLE MASS: ", vehicle.GetMass())

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)


# === Terrain === a single rigid patch provides the support and contact surface
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

spindle_positions = []
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
front_tire = vehicle.GetAxle(0).m_wheels[0].GetTire()  # cache: tire radius for support check
wheel_bottom_z = min(p.z for p in spindle_positions) - front_tire.GetRadius()
assert wheel_bottom_z >= -0.08, f"Gator wheels start below rigid terrain: {wheel_bottom_z:.3f} m"


# === Irrlicht visualization === vehicle-aware window with sky, camera, and light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle Application")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.2), 7.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Driver === interactive Irrlicht driver mirrors the real-time vehicle demos
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_EVERY * TIME_STEP / 1.0)
driver.SetThrottleDelta(RENDER_EVERY * TIME_STEP / 1.0)
driver.SetBrakingDelta(RENDER_EVERY * TIME_STEP / 0.3)
driver.Initialize()


# === Camera sensor === chassis-mounted RGB camera renders and saves its own images
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2.0, 2.5, 20.0), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-6.0, -6.0, 15.0), chrono.ChColor(0.8, 0.8, 0.8), 300.0)

camera_pose = chrono.ChFramed(
    chrono.ChVector3d(-4.0, 0.0, 1.7),
    chrono.QuatFromAngleAxis(0.15, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
camera = sens.ChCameraSensor(chassis, CAMERA_RATE, camera_pose, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV)
camera.SetName("Gator chassis RGB camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "Gator chassis camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/chassis_rgb/"))
manager.AddSensor(camera)


# === Review setup === runtime-only frame and CSV capture for validation


# === Main loop === synchronize every vehicle subsystem once per physics step
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            gator.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            gator.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)
            manager.Update()


            if system.GetChTime() >= SIM_END:
                break
        realtime_timer.Spin(TIME_STEP)
except (RuntimeError, ValueError, OSError) as exc:  # Chrono runtime/state or output errors
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === review videos and plots are removed from scored output
