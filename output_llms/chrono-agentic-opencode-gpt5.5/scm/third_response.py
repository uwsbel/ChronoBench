"""HMMWV on SCM deformable terrain with fixed box obstacles and a chassis camera sensor.

The simulation uses an SMC vehicle system owned by the HMMWV wrapper, Bullet
collision, TMeasy tires for soft-soil traction, and randomly scattered box
objects placed clear of the vehicle spawn.  A Chrono::Sensor RGB camera rides on
the chassis while Irrlicht provides the real-time review view.
"""

import math
import random

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === named values keep terrain, vehicle, objects, and sensors coherent
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 0.002
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 25.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 40.0
TERRAIN_DELTA = 0.08
VEHICLE_INIT = chrono.ChVector3d(-18.0, 0.0, 0.55)
VEHICLE_YAW = 0.0
VEHICLE_CLEAR_X = 5.0
VEHICLE_CLEAR_Y = 3.0

BOX_SIZE = chrono.ChVector3d(0.8, 0.8, 0.8)
BOX_DENSITY = 500.0
BOX_COUNT = 10
BOX_SEED = 303
SUPPORT_FAMILY = 4
TIRE_FAMILY = 1
CHASSIS_FAMILY = 3


def footprint_overlaps(cx, cy, sx, sy, ox, oy, osx, osy, margin=0.1):
    return abs(cx - ox) * 2.0 < sx + osx + 2.0 * margin and abs(cy - oy) * 2.0 < sy + osy + 2.0 * margin


def add_visual_material(body, color):
    material = chrono.ChVisualMaterial()
    material.SetDiffuseColor(color)
    material.SetSpecularColor(chrono.ChColor(0.15, 0.15, 0.15))
    material.SetRoughness(0.8)
    body.GetVisualShape(0).AddMaterial(material)


def generate_box_positions():
    rng = random.Random(BOX_SEED)
    positions = []
    attempts = 0
    while len(positions) < BOX_COUNT and attempts < 400:
        attempts += 1
        x = rng.uniform(-8.0, 22.0)
        y = rng.uniform(-12.0, 12.0)
        if abs(y) < 3.2:
            continue
        if footprint_overlaps(x, y, BOX_SIZE.x, BOX_SIZE.y, VEHICLE_INIT.x, VEHICLE_INIT.y, VEHICLE_CLEAR_X, VEHICLE_CLEAR_Y):
            continue
        if any(footprint_overlaps(x, y, BOX_SIZE.x, BOX_SIZE.y, px, py, BOX_SIZE.x, BOX_SIZE.y, 0.4) for px, py in positions):
            continue
        positions.append((x, y))
    assert len(positions) == BOX_COUNT, "could not place all boxes without overlap"
    return positions


# === Vehicle system === wrapper creates the SMC system, chassis, suspension, wheels, and joints
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(VEHICLE_INIT, chrono.QuatFromAngleZ(VEHICLE_YAW)))
vehicle.SetTireType(veh.TireModelType_TMEASY)  # SCM needs non-rigid tires
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemSMC reused by terrain, props, sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
chassis = vehicle.GetChassisBody()  # cache: camera, moving patch, and logging use this body
vehicle_core = vehicle.GetVehicle()  # cache: mass, spindles, visualization, and tire setup
front_left_spindle = vehicle_core.GetSpindlePos(0, veh.LEFT)  # cache: visible wrapper-created body position
print("VEHICLE MASS: ", vehicle_core.GetMass())
print("FRONT LEFT SPINDLE: ", front_left_spindle)

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain and contact objects === SCM soil supports tire ruts; a support slab holds fixed box obstacles
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(2.0e6, 0.0, 1.1, 0.0, 30.0, 0.01, 2.0e8, 3.0e4)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(5.0, 3.0, 1.0))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.12)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 40)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_DELTA)

contact_mat = chrono.ChContactMaterialSMC()
contact_mat.SetFriction(0.85)
contact_mat.SetRestitution(0.01)
contact_mat.SetYoungModulus(2.0e7)

support = chrono.ChBodyEasyBox(TERRAIN_LENGTH, TERRAIN_WIDTH, 0.2, 1000.0, True, True, contact_mat)
support.SetName("asset_support_ground")
support.SetPos(chrono.ChVector3d(0.0, 0.0, -0.1))
support.SetFixed(True)
support.EnableCollision(True)
add_visual_material(support, chrono.ChColor(0.42, 0.36, 0.28))
support_cm = support.GetCollisionModel()
support_cm.SetFamily(SUPPORT_FAMILY)
support_cm.DisallowCollisionsWith(TIRE_FAMILY)
support_cm.DisallowCollisionsWith(CHASSIS_FAMILY)
system.AddBody(support)

boxes = []
for i, (x_pos, y_pos) in enumerate(generate_box_positions()):
    box = chrono.ChBodyEasyBox(BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, BOX_DENSITY, True, True, contact_mat)
    box.SetName(f"random_box_{i:02d}")
    box.SetPos(chrono.ChVector3d(x_pos, y_pos, BOX_SIZE.z / 2.0))
    box.SetFixed(True)
    box.EnableCollision(True)
    add_visual_material(box, chrono.ChColor(0.75, 0.22 + 0.04 * (i % 4), 0.18))
    assert not footprint_overlaps(x_pos, y_pos, BOX_SIZE.x, BOX_SIZE.y, VEHICLE_INIT.x, VEHICLE_INIT.y, VEHICLE_CLEAR_X, VEHICLE_CLEAR_Y), "box spawned inside vehicle footprint"
    system.AddBody(box)
    boxes.append(box)

tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
chassis_mat = chrono.ChContactMaterialSMC()
chassis_mat.SetFriction(0.7)
chassis_mat.SetRestitution(0.01)
chassis.AddCollisionShape(
    chrono.ChCollisionShapeBox(chassis_mat, 4.2, 2.1, 1.0),
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.35), chrono.QUNIT),
)
chassis.EnableCollision(True)
chassis_cm = chassis.GetCollisionModel()
chassis_cm.SetFamily(CHASSIS_FAMILY)
chassis_cm.DisallowCollisionsWith(SUPPORT_FAMILY)
for axle in vehicle_core.GetAxles():
    for iw in range(2):
        wheel = axle.m_wheels[iw]
        tire = wheel.GetTire()
        spindle = wheel.GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire.GetRadius() + 0.04, tire.GetWidth()),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2.0)),
        )
        spindle.EnableCollision(True)
        spindle_cm = spindle.GetCollisionModel()
        spindle_cm.SetFamily(TIRE_FAMILY)
        spindle_cm.DisallowCollisionsWith(TIRE_FAMILY)
        spindle_cm.DisallowCollisionsWith(SUPPORT_FAMILY)
system.GetCollisionSystem().BindAll()


# === Sensor camera === chassis-mounted RGB camera with point lights and standard filters
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(0.0, -6.0, 8.0), chrono.ChColor(1.0, 1.0, 1.0), 80.0)
manager.scene.AddPointLight(chrono.ChVector3f(10.0, 6.0, 6.0), chrono.ChColor(0.8, 0.8, 0.8), 80.0)
manager.scene.AddPointLight(chrono.ChVector3f(-8.0, 0.0, 6.0), chrono.ChColor(0.6, 0.6, 0.6), 60.0)

camera_offset = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0.0, 2.0),
    chrono.QuatFromAngleAxis(0.24, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
camera = sens.ChCameraSensor(chassis, 30.0, camera_offset, 1280, 720, 1.408)
camera.SetName("Chassis RGB Camera")
camera.SetLag(0.0)
camera.SetCollectionWindow(0.0)
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Chassis RGB Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/chassis_rgb/"))
manager.AddSensor(camera)


# === Visualization and driver === vehicle-aware Irrlicht view plus truth-shaped interactive driver
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("SCM HMMWV with Boxes and Camera Sensor")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()


# === Main loop === synchronize full vehicle stack, update sensors, and record review outputs when requested
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0
try:

    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()  # cache: one input sample feeds all subsystem Synchronize calls
            driver.Synchronize(time)
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            manager.Update()


            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:
    print(f"file output failed: {exc}")
    raise
except (RuntimeError, ValueError) as exc:
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
