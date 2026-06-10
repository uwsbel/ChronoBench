"""Vehicle application with rigid obstacles and a chassis-mounted lidar.

This PyChrono 9.0 NSC simulation runs an HMMWV on flat rigid terrain from
chrono.ChVector3d(0, -5, 0.4).  A blue box and blue cylinder stand on the path,
and a lidar mounted above the chassis produces depth, intensity, and XYZI point
cloud outputs while the vehicle drives with fixed steering and throttle.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

INIT_LOC = chrono.ChVector3d(0, -5, 0.4)
INIT_ROT = chrono.QUNIT

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 40.0
OBSTACLE_DENSITY = 1000.0
BOX_SIZE = chrono.ChVector3d(1.0, 1.0, 1.0)
BOX_POS = chrono.ChVector3d(0.0, 0.0, 0.5)
CYL_RADIUS = 0.5
CYL_HEIGHT = 1.0
CYL_POS = chrono.ChVector3d(0.0, 0.0, 1.5)

LIDAR_UPDATE_RATE = 5.0
LIDAR_HORIZONTAL_SAMPLES = 800
LIDAR_VERTICAL_CHANNELS = 300
LIDAR_HORIZONTAL_FOV = 2 * chrono.CH_PI
LIDAR_MAX_VERTICAL_FOV = chrono.CH_PI / 12
LIDAR_MIN_VERTICAL_FOV = -chrono.CH_PI / 6
LIDAR_MAX_RANGE = 100.0
LIDAR_SAMPLE_RADIUS = 2
LIDAR_DIVERGENCE = 0.003
LIDAR_OFFSET = chrono.ChVector3d(0.0, 0.0, 2.0)

DRIVER_STEERING = 0.5
DRIVER_THROTTLE = 0.2
DRIVER_BRAKING = 0.0


class FixedInputDriver(veh.ChDriver):
    """Small scored-core driver that holds the requested steering and throttle."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetSteering(DRIVER_STEERING)
        self.SetThrottle(DRIVER_THROTTLE)
        self.SetBraking(DRIVER_BRAKING)


def apply_blue_texture(body):
    """Apply a visible blue material/texture to an easy factory body."""
    shape = body.GetVisualShape(0)
    shape.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    shape.SetColor(chrono.ChColor(0.1, 0.1, 0.9))


def main():
    # === Vehicle and system ===
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    system = hmmwv.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

    chassis = hmmwv.GetChassisBody()  # cache: chassis body reused by sensors and logging
    vehicle_model = hmmwv.GetVehicle()  # cache: wrapper-created vehicle object
    # Wrapper-created essentials: owned ChSystem, chassis, axles/spindles, tires,
    # rigid terrain, vehicle-aware Irrlicht visualizer, and fixed-input driver.

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain and obstacle bodies ===
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    obstacle_mat = chrono.ChContactMaterialNSC()
    obstacle_mat.SetFriction(0.8)
    obstacle_mat.SetRestitution(0.05)

    box = chrono.ChBodyEasyBox(
        BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, OBSTACLE_DENSITY, True, True, obstacle_mat
    )
    box.SetName("blue_box")
    box.SetPos(BOX_POS)
    apply_blue_texture(box)
    system.Add(box)

    cylinder = chrono.ChBodyEasyCylinder(
        chrono.ChAxis_Z, CYL_RADIUS, CYL_HEIGHT, OBSTACLE_DENSITY, True, True, obstacle_mat
    )
    cylinder.SetName("blue_cylinder")
    cylinder.SetPos(CYL_POS)
    apply_blue_texture(cylinder)
    system.Add(cylinder)

    # === Sensor manager and lidar ===
    manager = sens.ChSensorManager(system)

    lidar_offset_pose = chrono.ChFramed(
        LIDAR_OFFSET,
        chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
    )
    lidar = sens.ChLidarSensor(
        chassis,
        LIDAR_UPDATE_RATE,
        lidar_offset_pose,
        LIDAR_HORIZONTAL_SAMPLES,
        LIDAR_VERTICAL_CHANNELS,
        LIDAR_HORIZONTAL_FOV,
        LIDAR_MAX_VERTICAL_FOV,
        LIDAR_MIN_VERTICAL_FOV,
        LIDAR_MAX_RANGE,
        sens.LidarBeamShape_RECTANGULAR,
        LIDAR_SAMPLE_RADIUS,
        LIDAR_DIVERGENCE,
        LIDAR_DIVERGENCE,
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)
    lidar.PushFilter(
        sens.ChFilterVisualize(
            LIDAR_HORIZONTAL_SAMPLES, LIDAR_VERTICAL_CHANNELS, "Raw Lidar Depth"
        )
    )
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # === Visualization and driver ===
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV obstacle lidar application")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle_model)

    driver = FixedInputDriver(vehicle_model)
    driver.Initialize()
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0


    # === Main loop ===

    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            if step_number % RENDER_EVERY == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            time = system.GetChTime()
            driver_inputs = driver.GetInputs()

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
    except (RuntimeError, ValueError) as exc:
        print(f"simulation failed during stepping: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError, IOError) as exc:
        print(f"fatal simulation error: {exc}")
        raise
