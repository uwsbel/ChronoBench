"""HMMWV rigid-terrain GPS/IMU simulation.

This PyChrono NSC vehicle scene drives an HMMWV over a flat rigid terrain with
constant steering and throttle. GPS and accelerometer sensors are mounted on the
vehicle chassis at an offset pose of (0, 0, 1), producing live access buffers for
trajectory and inertial data.
"""

# === Imports ===
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants ===
step_size = 1e-3
tire_step_size = step_size
sim_end = 8.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_steps = max(1, math.ceil(render_step_size / step_size))  # precomputed once

terrain_length = 100.0
terrain_width = 100.0
terrain_friction = 0.9
terrain_restitution = 0.01

init_loc = chrono.ChVector3d(0.0, 0.0, 0.6)
init_rot = chrono.QUNIT
sensor_rate = 10.0
sensor_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)
constant_steering = 0.6
constant_throttle = 0.5


class ConstantDriver(veh.ChDriver):
    """Scripted driver that maintains fixed steering and throttle."""

    def __init__(self, vehicle, steering, throttle):
        super().__init__(vehicle)
        self._steering = steering
        self._throttle = throttle

    def Synchronize(self, time):
        self.SetSteering(self._steering)
        self.SetThrottle(self._throttle)
        self.SetBraking(0.0)


# === Vehicle system ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

chassis = hmmwv.GetChassisBody()  # cache: sensor body and per-step logging target
vehicle_handle = hmmwv.GetVehicle()  # cache: driver constructor and visible wrapper handle
# Wrapper-created components: owned ChSystem, chassis body, suspension links,
# wheel/tire bodies, terrain contact, vehicle visual system, and scripted driver.


# === Terrain ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("GPS and IMU HMMWV")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_handle)


# === Sensors ===
manager = sens.ChSensorManager(system)

imu = sens.ChAccelerometerSensor(chassis, sensor_rate, sensor_offset_pose, sens.ChNoiseNone())
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(imu)

gps = sens.ChGPSSensor(
    chassis,
    sensor_rate,
    sensor_offset_pose,
    gps_reference,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)


# === Driver ===
driver = ConstantDriver(vehicle_handle, constant_steering, constant_throttle)
driver.Initialize()


# === Review recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError, OSError) as exc:
    print(f"Simulation failed: {exc}")
    raise
finally:
    pass


# === Review post-processing ===
