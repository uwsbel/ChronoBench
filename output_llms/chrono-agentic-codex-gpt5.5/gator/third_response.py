"""Gator utility vehicle on rigid terrain with primitive visuals and chassis collision.

This PyChrono 9.0 NSC vehicle simulation uses the catalog Gator wrapper on a
flat rigid patch. The vehicle is shown with primitive visualization, its chassis
uses primitive collision shapes, and the interactive keyboard driver applies
steering, throttle, and braking more slowly for a less responsive control feel.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Fixed parameters keep vehicle setup and driver response visible.
STEP_SIZE = 2.0e-3
RENDER_STEP_SIZE = 1.0 / 50.0
SIM_END = 8.0
TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.50)
INIT_ROT = chrono.QUNIT
CHASE_TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 0.75)
CHASE_DISTANCE = 6.0
CHASE_HEIGHT = 0.5
STEERING_TIME = 2.5
THROTTLE_TIME = 2.5
BRAKING_TIME = 1.5
RENDER_STEPS = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)  # precomputed once


def build_simulation():
    """Build the Gator, rigid terrain, vehicle visualizer, and slower driver."""
    # === Vehicle system ===
    # The Gator wrapper owns the ChSystem; no separate system is created.
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
    gator.SetTireType(veh.TireModelType_RIGID)
    gator.SetTireStepSize(STEP_SIZE)
    gator.Initialize()

    system = gator.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

    # cache: expose wrapper-created essentials for source review and hot-loop reuse
    chassis = gator.GetChassisBody()
    vehicle = gator.GetVehicle()
    spindle_positions = [
        vehicle.GetSpindlePos(axle, side)
        for axle in range(vehicle.GetNumberAxles())
        for side in (veh.LEFT, veh.RIGHT)
    ]
    assert all(p.z > 0.0 for p in spindle_positions), "Gator wheel spindles must start above terrain"

    # === Visualization choices ===
    # Primitive rendering satisfies the requested simplified visual appearance.
    gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    # === Terrain ===
    # Rigid NSC patch matches the wrapper contact method and provides a simple road.
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)

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

    # === Irrlicht visualization ===
    # Vehicle-aware Irrlicht follows the Gator and exposes the interactive driver.
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Gator primitive visualization and slower controls")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_TRACK_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle)

    # === Driver ===
    # Larger response times make keyboard controls take longer to reach full input.
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
    driver.Initialize()

    return system, gator, chassis, terrain, vis, driver


def run():
    """Run the real-time vehicle loop and optional review capture."""
    system, gator, chassis, terrain, vis, driver = build_simulation()
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0


    # === Main loop ===
    # Synchronize and advance the full driver-terrain-vehicle-visualization stack.
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            sim_time = system.GetChTime()
            driver.Synchronize(sim_time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(sim_time)
            gator.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            gator.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)


            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError, AssertionError, OSError) as exc:
        # Solver, API, assertion, and output-file failures should preserve traceback.
        traceback.print_exc()
        raise exc
    finally:
        pass


if __name__ == "__main__":
    run()
