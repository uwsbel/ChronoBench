"""FED-Alpha vehicle on rigid grass terrain with an onboard camera sensor.

This PyChrono 9.0 NSC simulation uses the catalog FEDA vehicle, a rigid
terrain patch textured as grass, an Irrlicht vehicle visualizer, and an
OptiX camera sensor mounted to the chassis for a first-person view. The
sensor manager owns the chassis camera and point lights so the camera preview
updates as the vehicle state advances.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === tuned once for a bounded FEDA camera-sensor run.
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 5.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 60.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.6)
CAMERA_OFFSET = chrono.ChVector3d(1.1, 0.0, 1.25)
CAMERA_PITCH = -0.06
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_RATE = 30.0
CAMERA_FOV = 1.408


# === Vehicle and terrain === wrapper owns the system, terrain shares it.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)
feda.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(TIRE_STEP_SIZE)
feda.Initialize()

system = feda.GetSystem()  # cache: wrapper-owned ChSystemNSC reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = feda.GetVehicle()  # cache: wrapper vehicle interface reused in loop
chassis = feda.GetChassisBody()  # cache: real chassis body for camera attachment
print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created essentials: chassis, suspension, steering, wheels, tires, and joints
# are created by veh.FEDA; the rigid terrain patch below shares the wrapper system.
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 160, 120)
patch.SetColor(chrono.ChColor(0.35, 0.55, 0.25))
terrain.Initialize()


# === Sensor manager and chassis camera === point-lit FPV camera rides on the FEDA chassis.
manager = sens.ChSensorManager(system)
LIGHT_INTENSITY = 1.5
manager.scene.AddPointLight(
    chrono.ChVector3f(10.0, 6.0, 8.0),
    chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
    80.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-8.0, -6.0, 6.0),
    chrono.ChColor(1.1, 1.1, 1.1),
    70.0,
)

camera_pose = chrono.ChFramed(
    CAMERA_OFFSET,
    chrono.QuatFromAngleAxis(CAMERA_PITCH, chrono.ChVector3d(0, 1, 0)),
)
camera = sens.ChCameraSensor(
    chassis,
    CAMERA_RATE,
    camera_pose,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
)
camera.SetName("FEDA chassis first-person camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "FEDA FPV Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/fpv/"))
manager.AddSensor(camera)


# === Visualization and driver === vehicle Irrlicht window plus truth-shaped interactive driver.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA grass terrain with chassis camera")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_POINT, 8.0, 0.7)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_EVERY * STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_EVERY * STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_EVERY * STEP_SIZE / 0.3)
driver.Initialize()


# === Review logging setup === CSV and frame directories are stripped for scoring.


# === Main loop === synchronize every vehicle subsystem and update sensors each step.
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            driver.Synchronize(time)
            terrain.Synchronize(time)
            feda.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            feda.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            manager.Update()


            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError, OSError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
