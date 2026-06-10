"""HMMWV soft-soil scene with box obstacles and a chassis-mounted RGB camera sensor.

This self-contained PyChrono 9.0 script models an SMC HMMWV driving on SCM
deformable terrain. Deterministically placed ChBodyEasyBox obstacles populate
the scene while keeping a clear vehicle spawn corridor. A ChSensorManager adds
point lights and an RGB camera sensor attached to the chassis.
"""

import math
import random
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === reproducible vehicle, terrain, object, and sensor parameters
STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.001
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_STEPS = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 70.0
TERRAIN_WIDTH = 36.0
TERRAIN_RESOLUTION = 0.08
INIT_POS = chrono.ChVector3d(-22.0, 0.0, 0.55)
INIT_ROT = chrono.QUNIT

BOX_COUNT = 12
BOX_SIZE = chrono.ChVector3d(1.15, 0.85, 0.75)
BOX_DENSITY = 650.0
BOX_MIN_X = -8.0
BOX_MAX_X = 24.0
BOX_MIN_Y = -13.0
BOX_MAX_Y = 13.0
VEHICLE_CLEAR_X = 5.5
VEHICLE_CLEAR_Y = 2.6

TIRE_FAMILY = 1
CHASSIS_FAMILY = 2
SUPPORT_FAMILY = 4
BOX_FAMILY = 5


# === Helpers === deterministic placement and simple object construction
def candidate_is_clear_of_vehicle(x_pos, y_pos):
    """Return True when a box center does not overlap the vehicle start footprint."""
    dx = abs(x_pos - INIT_POS.x)
    dy = abs(y_pos - INIT_POS.y)
    return dx > VEHICLE_CLEAR_X or dy > VEHICLE_CLEAR_Y


def create_box_obstacles(system, material):
    """Create randomly positioned boxes with deterministic seed and clear spawn zone."""
    rng = random.Random(42)
    boxes = []
    attempts = 0
    while len(boxes) < BOX_COUNT and attempts < 300:
        attempts += 1
        x_pos = rng.uniform(BOX_MIN_X, BOX_MAX_X)
        y_pos = rng.uniform(BOX_MIN_Y, BOX_MAX_Y)
        if not candidate_is_clear_of_vehicle(x_pos, y_pos):
            continue
        if any(math.hypot(x_pos - b.GetPos().x, y_pos - b.GetPos().y) < 2.0 for b in boxes):
            continue

        box = chrono.ChBodyEasyBox(
            BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, BOX_DENSITY, True, True, material
        )
        box.SetName(f"random_box_{len(boxes):02d}")
        box.SetPos(chrono.ChVector3d(x_pos, y_pos, BOX_SIZE.z / 2.0 + 0.02))
        box.GetCollisionModel().SetFamily(BOX_FAMILY)
        system.AddBody(box)
        boxes.append(box)
    if len(boxes) != BOX_COUNT:
        raise ValueError("deterministic placement could not create all box obstacles")
    return boxes


def add_chassis_collision(chassis_body, material):
    """Add a primitive chassis collision envelope so boxes can contact the vehicle."""
    chassis_body.AddCollisionShape(
        chrono.ChCollisionShapeBox(material, 4.2, 2.0, 1.0),
        chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.45), chrono.QUNIT),
    )
    chassis_body.EnableCollision(True)
    chassis_cm = chassis_body.GetCollisionModel()
    chassis_cm.SetFamily(CHASSIS_FAMILY)
    chassis_cm.DisallowCollisionsWith(TIRE_FAMILY)
    chassis_cm.DisallowCollisionsWith(SUPPORT_FAMILY)


def add_tire_collision_cylinders(hmmwv, system, material):
    """Add SCM ray-cast tire collision cylinders for non-rigid TMEASY tires."""
    first_wheel = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0]  # cache: reused for tire geometry
    tire_rad = first_wheel.GetTire().GetRadius()
    tire_width = first_wheel.GetTire().GetWidth()
    for axle in hmmwv.GetVehicle().GetAxles():
        for wheel in axle.m_wheels:
            spindle = wheel.GetSpindle()
            spindle.AddCollisionShape(
                chrono.ChCollisionShapeCylinder(material, tire_rad + 0.04, tire_width),
                chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2.0)),
            )
            spindle.EnableCollision(True)
            spindle_cm = spindle.GetCollisionModel()
            spindle_cm.SetFamily(TIRE_FAMILY)
            spindle_cm.DisallowCollisionsWith(TIRE_FAMILY)
            spindle_cm.DisallowCollisionsWith(SUPPORT_FAMILY)
    system.GetCollisionSystem().BindAll()


# === Vehicle === catalog HMMWV wrapper owns the SMC system and vehicle bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemSMC used everywhere
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: chassis body used for terrain, collision, sensor
vehicle_model = hmmwv.GetVehicle()  # cache: wrapper vehicle model for vis and mass
print("VEHICLE MASS: ", vehicle_model.GetMass())
# wrapper-created essentials: system, chassis, wheels, suspension, steering, and powertrain.

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Contact Materials === SMC materials for SCM, support, boxes, chassis, and tires
obstacle_mat = chrono.ChContactMaterialSMC()
obstacle_mat.SetFriction(0.75)
obstacle_mat.SetRestitution(0.02)
obstacle_mat.SetYoungModulus(2.0e7)

tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2.0e7)

support_mat = chrono.ChContactMaterialSMC()
support_mat.SetFriction(0.85)
support_mat.SetRestitution(0.01)
support_mat.SetYoungModulus(2.0e7)


# === Terrain & Objects === SCM soil, hidden support plane, and random box bodies
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(2.0e6, 0.0, 1.1, 0.0, 30.0, 0.01, 2.0e8, 3.0e4)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.12)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RESOLUTION)
terrain.SetMeshWireframe(False)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 12.0, 12.0)

support = chrono.ChBodyEasyBox(
    TERRAIN_LENGTH, TERRAIN_WIDTH, 0.2, 1000.0, False, True, support_mat
)
support.SetName("hidden_box_support_ground")
support.SetPos(chrono.ChVector3d(0, 0, -0.1))
support.SetFixed(True)
support.EnableCollision(True)
support_cm = support.GetCollisionModel()
support_cm.SetFamily(SUPPORT_FAMILY)
support_cm.DisallowCollisionsWith(TIRE_FAMILY)
support_cm.DisallowCollisionsWith(CHASSIS_FAMILY)
system.AddBody(support)

boxes = create_box_obstacles(system, obstacle_mat)  # cache: obstacle list for logging
add_chassis_collision(chassis, obstacle_mat)
add_tire_collision_cylinders(hmmwv, system, tire_mat)


# === Sensor Manager === point-lit camera sensor scene separate from Irrlicht review view
manager = sens.ChSensorManager(system)
for light_pos in (
    chrono.ChVector3f(-12.0, -10.0, 18.0),
    chrono.ChVector3f(10.0, 8.0, 16.0),
    chrono.ChVector3f(24.0, -6.0, 20.0),
):
    manager.scene.AddPointLight(light_pos, chrono.ChColor(1.0, 1.0, 1.0), 180.0)

camera_offset = chrono.ChFramed(
    chrono.ChVector3d(0.7, 0.0, 1.35),
    chrono.QuatFromAngleAxis(0.10, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
camera = sens.ChCameraSensor(chassis, 30.0, camera_offset, 1280, 720, 1.408)
camera.SetName("Chassis RGB Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(640, 360, "Chassis RGB Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(camera)


# === Visualization === vehicle Irrlicht window with chase camera, sky, lights, and grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("SCM HMMWV boxes and camera sensor")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddGrid(
    2.0,
    2.0,
    30,
    18,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)
vis.AttachVehicle(vehicle_model)


# === Driver === interactive vehicle driver plus deterministic run-time motion
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0


# === Review logging === CSV mirrors key physics values during record-mode validation


# === Main Loop === synchronize driver, SCM terrain, HMMWV, sensors, and visualization
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()
        if time < 0.35:
            driver_inputs.m_throttle = 0.0
            driver_inputs.m_braking = 0.25
        else:
            driver_inputs.m_throttle = 0.55
            driver_inputs.m_braking = 0.0
        driver_inputs.m_steering = 0.08 * math.sin(0.7 * time)

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError, OSError) as exc:
    traceback.print_exc()
    raise
finally:
    pass


# === Post Processing === assemble review videos and plot CSV only in record mode
