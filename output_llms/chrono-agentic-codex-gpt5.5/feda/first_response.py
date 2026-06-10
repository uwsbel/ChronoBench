"""FEDA vehicle on rigid terrain with Irrlicht visualization.

This NSC vehicle simulation initializes a FED-Alpha catalog vehicle with PAC02
tires, mesh visualization for the vehicle subsystems, a textured rigid terrain
patch, and an interactive Irrlicht driver. The expected behavior is a real-time
vehicle demo where keyboard inputs control steering, throttle, and braking while
the camera follows the chassis over the flat terrain.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Parameters ===
# Named constants keep vehicle, terrain, and render settings visible.
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_STEP = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP / STEP_SIZE))  # precomputed once

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.55)
INIT_ROT = chrono.QuatFromAngleZ(0.0)
TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
CHASE_DISTANCE = 7.0
CHASE_HEIGHT = 0.5

TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TEXTURE_SCALE_X = 200.0
TEXTURE_SCALE_Y = 200.0

STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


def build_vehicle():
    """Create and initialize the wrapper-owned FEDA vehicle."""
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    feda = veh.FEDA()
    feda.SetContactMethod(chrono.ChContactMethod_NSC)
    feda.SetChassisCollisionType(veh.CollisionType_NONE)
    feda.SetChassisFixed(False)
    feda.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    feda.SetTireType(veh.TireModelType_PAC02)
    feda.SetTireStepSize(TIRE_STEP_SIZE)
    feda.Initialize()

    system = feda.GetSystem()  # cache: wrapper-owned system reused throughout
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

    feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
    feda.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    feda.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
    feda.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Wrapper-created essentials: system, chassis, axles, suspension, steering,
    # powertrain, tires, terrain coupling, visualization, and driver.
    chassis = feda.GetChassisBody()  # cache: fetched once for validation/logging
    return feda, system, chassis


def build_terrain(system):
    """Create the flat textured rigid terrain attached to the vehicle system."""
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
    patch.SetTexture(
        veh.GetDataFile("terrain/textures/tile4.jpg"),
        TEXTURE_SCALE_X,
        TEXTURE_SCALE_Y,
    )
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    return terrain


def build_visualizer(feda):
    """Create the Irrlicht vehicle visual system and follow camera."""
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("FEDA Rigid Terrain")
    vis.SetWindowSize(1280, 1024)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetChaseCamera(TRACK_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AddGrid(
        2.0,
        2.0,
        50,
        50,
        chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.02), chrono.QUNIT),
        chrono.ChColor(0.35, 0.35, 0.35),
    )
    vis.AttachVehicle(feda.GetVehicle())
    return vis


def build_driver(vis):
    """Create the interactive driver tied to the Irrlicht visual system."""
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP / STEERING_TIME)
    driver.SetThrottleDelta(RENDER_STEP / THROTTLE_TIME)
    driver.SetBrakingDelta(RENDER_STEP / BRAKING_TIME)
    driver.Initialize()
    return driver


def main():
    """Run the real-time FEDA vehicle simulation."""
    feda, system, chassis = build_vehicle()
    terrain = build_terrain(system)
    vis = build_visualizer(feda)
    driver = build_driver(vis)
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
            feda.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            feda.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)


            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"simulation failed: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
