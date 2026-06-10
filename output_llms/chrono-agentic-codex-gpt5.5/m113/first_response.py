"""M113 tracked vehicle on a rigid SMC terrain with Irrlicht visualization.

The script configures Chrono's M113 tracked vehicle with explicit initial
conditions, compliant terrain contact, a simple scripted driver, and a
real-time visualization loop. The vehicle accelerates forward on a flat rigid
terrain while the terrain, driver, vehicle, and visual system synchronize and
advance at each fixed timestep.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants: vehicle, terrain, visualization, and integration ===
STEP_SIZE = 5.0e-4
SIM_END = 6.0
RENDER_STEP_SIZE = 1.0 / 50.0
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 100.0
TERRAIN_THICKNESS = 1.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG_MODULUS = 8.0e6

INIT_LOCATION = chrono.ChVector3d(0.0, 0.0, 0.8)
INIT_ROTATION = chrono.QUNIT
INIT_FORWARD_SPEED = 0.0
VIS_TYPE = veh.VisualizationType_MESH

CHASSIS_CAMERA_POINT = chrono.ChVector3d(0.0, 0.0, 1.5)
CHASE_DISTANCE = 10.0
CHASE_HEIGHT = 1.0
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 1024

DRIVER_THROTTLE = 0.8
DRIVER_STEERING = 0.0
DRIVER_BRAKING = 0.0


def build_simulation():
    """Build the M113, rigid terrain, driver, and tracked-vehicle visualizer."""
    # === Vehicle setup: wrapper owns the SMC system ===
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.M113()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
    vehicle.SetInitFwdVel(INIT_FORWARD_SPEED)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned system reused throughout
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.GetSolver().AsIterative().SetMaxIterations(150)
    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

    vehicle.SetChassisVisualizationType(VIS_TYPE)
    vehicle.SetSprocketVisualizationType(VIS_TYPE)
    vehicle.SetIdlerVisualizationType(VIS_TYPE)
    vehicle.SetIdlerWheelVisualizationType(VIS_TYPE)
    vehicle.SetRoadWheelVisualizationType(VIS_TYPE)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetTrackShoeVisualizationType(VIS_TYPE)

    chassis = vehicle.GetChassisBody()  # cache: diagnostic body handle reused in logging
    tracked_vehicle = vehicle.GetVehicle()  # cache: Chrono vehicle handle for driver/visualizer

    # Wrapper-created essentials: system, chassis, suspension, tracks, powertrain,
    # terrain, driver, and tracked visualizer are named locals for review clarity.

    # === Terrain: flat rigid SMC patch with requested friction/restitution ===
    terrain = veh.RigidTerrain(system)
    terrain_material = chrono.ChContactMaterialSMC()
    terrain_material.SetFriction(TERRAIN_FRICTION)
    terrain_material.SetRestitution(TERRAIN_RESTITUTION)
    terrain_material.SetYoungModulus(TERRAIN_YOUNG_MODULUS)
    patch = terrain.AddPatch(
        terrain_material,
        chrono.CSYSNORM,
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
        TERRAIN_THICKNESS,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization: tracked-vehicle Irrlicht window, camera, and lighting ===
    vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("M113 tracked vehicle")
    vis.SetWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT)
    vis.SetChaseCamera(CHASSIS_CAMERA_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AddGrid(5.0, 5.0, 40, 20, chrono.CSYSNORM, chrono.ChColor(0.35, 0.35, 0.35))
    vis.AttachVehicle(tracked_vehicle)

    # === Driver: scripted open-loop control for forward tracked motion ===
    driver = veh.ChDriver(tracked_vehicle)
    driver.Initialize()

    return system, vehicle, chassis, terrain, vis, driver


def main():
    system, vehicle, chassis, terrain, vis, driver = build_simulation()
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    frame_number = 0

    # === Main loop: synchronize and advance every subsystem in real time ===
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            driver.SetThrottle(DRIVER_THROTTLE)
            driver.SetSteering(DRIVER_STEERING)
            driver.SetBraking(DRIVER_BRAKING)
            driver_inputs = driver.GetInputs()

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver.Synchronize(time)
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
    except OSError as exc:  # file-system failures while writing diagnostics
        traceback.print_exc()
        raise
    except RuntimeError as exc:  # Chrono solver, terrain, or visual-system failures
        traceback.print_exc()
        raise
    except ValueError as exc:  # numeric diagnostic conversion failures
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
