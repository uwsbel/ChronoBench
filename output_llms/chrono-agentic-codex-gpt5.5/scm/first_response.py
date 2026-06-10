"""Full HMMWV on SCM deformable terrain with an Irrlicht real-time driver.

This PyChrono 9.0.0 simulation builds an SMC HMMWV_Full wrapper vehicle on
Bekker-Wong SCM soil.  The chassis, suspension, steering, wheels, and requested
rigid tires use mesh visualization.  The SCM patch follows the chassis and plots
sinkage in false color while the interactive Irrlicht driver provides steering,
throttle, and braking controls at a 50 Hz render cadence.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.001
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once
SIM_END = 8.0

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 80.0
TERRAIN_DELTA = 0.08
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.7)
INIT_ROT = chrono.QuatFromAngleZ(0.0)
MOVING_PATCH_CENTER = chrono.ChVector3d(0.0, 0.0, 0.0)
MOVING_PATCH_DIMS = chrono.ChVector3d(5.0, 3.0, 1.0)
TIRE_RADIUS_EST = 0.47
WHEEL_BOTTOM_TOL = 0.15


def build_simulation():
    # === Vehicle system ===
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    vehicle.SetTireType(veh.TireModelType_RIGID)  # prompt: rigid tire model
    vehicle.SetTireStepSize(TIRE_STEP_SIZE)
    vehicle.Initialize()

    system = vehicle.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

    chassis = vehicle.GetChassisBody()  # cache: fetched once, reused every step
    chrono_vehicle = vehicle.GetVehicle()  # cache: wrapper vehicle handle for queries

    # Wrapper-created essentials: owned SMC system, HMMWV bodies/joints, chassis,
    # tire bodies, SCM terrain, vehicle-aware Irrlicht visualization, and driver.
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    spindle_positions = []
    for axle_id in range(chrono_vehicle.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_positions.append(chrono_vehicle.GetSpindlePos(axle_id, side))
    wheel_bottom_z = min(pos.z for pos in spindle_positions) - TIRE_RADIUS_EST
    assert wheel_bottom_z >= -WHEEL_BOTTOM_TOL, (
        f"vehicle starts too low for SCM rest plane: wheel bottom z={wheel_bottom_z:.3f}"
    )

    # === SCM terrain ===
    terrain = veh.SCMTerrain(system)
    terrain.SetSoilParameters(
        2.0e6,
        0.0,
        1.1,
        0.0,
        30.0,
        0.01,
        2.0e8,
        3.0e4,
    )
    terrain.AddMovingPatch(chassis, MOVING_PATCH_CENTER, MOVING_PATCH_DIMS)
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.10)
    terrain.SetMeshWireframe(False)
    terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 80.0, 80.0)
    terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_DELTA)

    # === Irrlicht visualization ===
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV on SCM deformable terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(chrono_vehicle)

    # === Interactive driver ===
    driver = veh.ChInteractiveDriverIRR(vis)
    steering_time = 1.0
    throttle_time = 1.0
    braking_time = 0.3
    driver.SetSteeringDelta(RENDER_STEP_SIZE / steering_time)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / throttle_time)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / braking_time)
    driver.Initialize()

    return system, vehicle, terrain, vis, driver, chassis


def run():
    system, vehicle, terrain, vis, driver, chassis = build_simulation()
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0


    try:
        # === Main loop ===
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)


            step_number += 1
            realtime_timer.Spin(STEP_SIZE)

    except (RuntimeError, ValueError, AssertionError) as exc:  # solver divergence / bad state
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    run()
