"""Gator vehicle application with rigid terrain, interactive driving, visualization, and a chassis camera.

This PyChrono 9.0 NSC simulation initializes a catalog Gator vehicle on a flat
RigidTerrain patch, assigns different visualization modes to its major vehicle
subsystems, attaches an interactive Irrlicht driver, and adds a sensor-manager
RGB camera mounted to the chassis. The vehicle, terrain, visualizer, driver, and
sensor manager are synchronized and advanced together so the Gator can be driven
while the camera renders images from the moving chassis.
"""

import math

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 240.0
TERRAIN_WIDTH = 40.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.50)
INIT_ROT = chrono.QUNIT
WHEEL_Z_TOL = 0.08

CAMERA_RATE = 30
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FOV = 1.408
CAMERA_OFFSET = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0.0, 2.0),
    chrono.QuatFromAngleAxis(0.18, chrono.ChVector3d(0.0, 1.0, 0.0)),
)


# === Vehicle and system ===
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

system = gator.GetSystem()  # cache: vehicle-owned system reused by terrain, sensors, and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = gator.GetVehicle()  # cache: wrapper vehicle handle reused for mass, axles, and visualization
chassis = gator.GetChassisBody()  # cache: chassis body reused by camera and logging

print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created essentials: vehicle-owned ChSystem, chassis body, axles,
# steering/suspension joints, tires, rigid terrain, IRR visualizer, and driver.
spindle_world = []
tire_radii = []
for axle_index in range(vehicle.GetNumberAxles()):
    axle = vehicle.GetAxle(axle_index)
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(vehicle.GetSpindlePos(axle_index, side))
        tire_radii.append(axle.GetWheel(side).GetTire().GetRadius())
wheel_bottom_z = min(p.z for p in spindle_world) - max(tire_radii)
assert wheel_bottom_z >= -WHEEL_Z_TOL, (
    f"Gator starts below rigid terrain: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise INIT_POS.z by {-wheel_bottom_z:.3f} m"
)

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)


# === Rigid terrain ===
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


# === Vehicle visualization and driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator vehicle application")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.2), 7.0, 0.5)
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


# === Sensor manager and chassis camera ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 20.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-8.0, -4.0, 12.0),
    chrono.ChColor(0.8, 0.8, 0.8),
    250.0,
)

cam = sens.ChCameraSensor(
    chassis,
    CAMERA_RATE,
    CAMERA_OFFSET,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
)
cam.SetName("Gator Chassis RGB Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "Gator chassis camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)


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

            driver.Synchronize(time)
            terrain.Synchronize(time)
            gator.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            gator.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            manager.Update()

            buf = cam.GetMostRecentRGBA8Buffer()
            if buf.HasData():  # guard: sensor buffer may be empty before the first camera tick
                _ = buf.GetRGBA8Data()

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
except (OSError, IOError) as exc:
    raise RuntimeError("review output file operation failed") from exc
except (RuntimeError, ValueError) as exc:
    raise
finally:
    pass
