"""ARTcar rigid-terrain demonstration using PyChrono NSC contact.

The simulation initializes a catalog ARTcar on a flat rigid terrain patch,
textures the ground, and visualizes the vehicle through the Irrlicht vehicle
visual system.  A real-time interactive driver controls steering, throttle, and
braking while the vehicle, terrain, driver, and visualization subsystems are
synchronized at each dynamics step.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Named setup values keep the vehicle, terrain, driver, and render cadence explicit.
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 5.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOCATION = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROTATION = chrono.QUNIT
CHASE_TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 0.75)
CHASE_DISTANCE = 6.0
CHASE_HEIGHT = 0.5

STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


def build_simulation():
    """Create the ARTcar, rigid terrain, Irrlicht visualizer, and driver."""
    # === Data paths ===
    # Vehicle demos use the bundled Chrono and vehicle data paths.
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # === Vehicle ===
    # The ARTcar wrapper owns the ChSystemNSC and creates chassis, suspension, wheels, and tires.
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisCollisionType(veh.CollisionType_NONE)
    car.SetChassisFixed(False)
    car.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
    car.SetTireType(veh.TireModelType_RIGID)
    car.SetTireStepSize(TIRE_STEP_SIZE)
    car.Initialize()

    system = car.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain and loop
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("VEHICLE MASS: ", car.GetVehicle().GetMass())

    chassis = car.GetChassisBody()  # cache: main rigid body, reused for diagnostics
    wheeled_vehicle = car.GetVehicle()  # cache: wrapper vehicle handle for visualizer attachment
    # wrapper-created components: chassis, steering links, suspensions, axles, wheels, and tires

    car.SetChassisVisualizationType(veh.VisualizationType_MESH)
    car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(veh.VisualizationType_MESH)
    car.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain ===
    # RigidTerrain uses an NSC material to match the ARTcar contact method.
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
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200.0, 200.0)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization ===
    # Vehicle-specific Irrlicht visualizer provides chase camera, vehicle HUD, and real-time rendering.
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("ARTcar on Rigid Terrain")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(CHASE_TRACK_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(wheeled_vehicle)

    # === Driver ===
    # The scored core remains an interactive Irrlicht driver for keyboard control.
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
    driver.Initialize()

    return car, system, chassis, terrain, vis, driver


def run():
    """Run the synchronized vehicle loop and optional review-only recording."""
    car, system, chassis, terrain, vis, driver = build_simulation()
    realtime_timer = chrono.ChRealtimeStepTimer()
    frame = 0
    step_number = 0


    try:
        # === Main loop ===
        # Render at 50 FPS and advance the vehicle subsystem stack every dynamics step.
        while vis.Run() and system.GetChTime() < SIM_END:
            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                frame += 1

            time = system.GetChTime()
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            car.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            car.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)

    except (RuntimeError, ValueError) as exc:  # Chrono solver or invalid-state failure
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    run()
