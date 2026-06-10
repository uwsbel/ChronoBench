"""ARTcar rigid-terrain demo using PyChrono NSC contact and Irrlicht.

The simulation builds a catalog ARTcar vehicle on a textured RigidTerrain patch,
uses an interactive Irrlicht driver for steering/throttle/braking, and advances
the vehicle, terrain, and visualization in a 50 FPS real-time loop.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Parameters === named constants define vehicle start, terrain, and cadence
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 1e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
TIRE_RADIUS = 0.23
WHEEL_Z_TOL = 0.08

STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


def run_simulation():
    # === Vehicle and system === wrapper creates the NSC system and vehicle bodies
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.SetTireStepSize(TIRE_STEP_SIZE)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused throughout
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = vehicle.GetChassisBody()  # cache: main chassis body reused for logging
    vehicle_model = vehicle.GetVehicle()  # cache: wrapper vehicle interface reused below
    # bodies: chassis, suspensions, steering, wheels, and tires are created by veh.ARTcar.
    # joints: suspension and steering links are created inside the catalog wrapper.
    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

    wheel_bottoms = []
    for axle_index in range(vehicle_model.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_pos = vehicle_model.GetSpindlePos(axle_index, side)
            wheel_bottoms.append(spindle_pos.z - TIRE_RADIUS)
    wheel_bottom_z = min(wheel_bottoms)
    assert wheel_bottom_z >= -WHEEL_Z_TOL, (
        f"ARTcar wheel bottom starts below rigid terrain: {wheel_bottom_z:.3f} m"
    )

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === rigid patch provides the contacted support plane and texture
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)

    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization and driver === vehicle Irrlicht visualizer owns the interactive driver
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("ARTcar on Rigid Terrain")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle.GetVehicle())

    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
    driver.Initialize()

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

    # === Main loop === synchronize and advance the complete vehicle subsystem stack
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            for _ in range(RENDER_STEPS):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()  # cache: pass one input struct to all subsystems

                driver.Synchronize(time)
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                vehicle.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)


                step_number += 1
                realtime_timer.Spin(STEP_SIZE)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError, AssertionError) as exc:
        traceback.print_exc()
        raise
    finally:
        pass


# === Run === execute the standalone simulation

run_simulation()
