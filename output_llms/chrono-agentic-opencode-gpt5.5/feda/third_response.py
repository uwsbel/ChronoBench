"""FEDA vehicle on grass-textured rigid terrain with an onboard RGB camera.

This NSC vehicle simulation uses the PyChrono FEDA wrapper, a flat rigid terrain
patch with grass texture, an Irrlicht vehicle visualizer, and a chassis-mounted
first-person ChCameraSensor. The vehicle can be driven interactively while the
sensor manager updates the camera stream as the chassis moves.
"""

import math

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === keep terrain, timing, and camera parameters explicit
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 160.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
CHASE_TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)

CAMERA_RATE = 30.0
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FOV = 1.25
CAMERA_OFFSET = chrono.ChVector3d(0.85, 0.0, 1.45)
CAMERA_PITCH = -0.08
CAMERA_LOOKAHEAD = chrono.ChVector3d(24.0, 0.0, 1.25)


# === Vehicle wrapper === FEDA owns the Chrono system and drivetrain bodies
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

system = feda.GetSystem()  # cache: wrapper-owned ChSystem reused for terrain and sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = feda.GetChassisBody()  # cache: real body for FPV camera attachment
vehicle = feda.GetVehicle()  # cache: wrapper vehicle object reused by vis and driver
# bodies: chassis, wheels, spindles, suspension, steering, and drivetrain are created by veh.FEDA
# joints: suspension and steering constraints are created by the wrapper and stepped through feda.Advance
print("VEHICLE MASS: ", vehicle.GetMass())

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === rigid flat ground with prompt-requested grass texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.6, 0.8, 0.45))
terrain.Initialize()


# === Sensor manager and camera === point-lit FPV camera follows the chassis
manager = sens.ChSensorManager(system)
LIGHT_INTENSITY = 1.2
manager.scene.AddPointLight(
    chrono.ChVector3f(0.0, 0.0, 25.0),
    chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(20.0, -15.0, 18.0),
    chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
    350.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-20.0, 15.0, 18.0),
    chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
    350.0,
)

camera_offset_pose = chrono.ChFramed(
    CAMERA_OFFSET,
    chrono.QuatFromAngleAxis(CAMERA_PITCH, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
fpv_camera = sens.ChCameraSensor(
    chassis,
    CAMERA_RATE,
    camera_offset_pose,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
)
fpv_camera.SetName("FEDA Chassis FPV Camera")
fpv_camera.SetLag(0)
fpv_camera.SetCollectionWindow(0)
fpv_camera.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "FEDA FPV Camera"))
fpv_camera.PushFilter(sens.ChFilterRGBA8Access())
fpv_camera.PushFilter(sens.ChFilterSave("cam/fpv/"))
manager.AddSensor(fpv_camera)


# === Visualization and driver === vehicle Irrlicht window plus interactive driver
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA grass terrain with chassis FPV camera")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_TRACK_POINT, 8.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()


# === Main loop === synchronize vehicle, terrain, visualization, and camera sensor
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        chassis_pos = chassis.GetPos()
        chassis_rot = chassis.GetRot()
        review_eye = chassis_pos + chassis_rot.RotateBack(CAMERA_OFFSET)
        review_target = chassis_pos + chassis_rot.RotateBack(CAMERA_LOOKAHEAD)
        vis.UpdateCamera(review_eye, review_target)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame += 1

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


            step_number += 1
            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / invalid Chrono state
    print(f"Simulation runtime failure: {exc}")
    raise
except (OSError, IOError) as exc:  # output path or sensor-frame file failure
    print(f"Simulation output failure: {exc}")
    raise
finally:
    pass
