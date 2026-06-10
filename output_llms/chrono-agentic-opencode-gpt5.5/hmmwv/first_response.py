"""Full HMMWV on flat rigid terrain using PyChrono Vehicle and Irrlicht.

The simulation uses the HMMWV_Full wrapper with NSC contact, Bullet collision,
TMEASY tires, primitive subsystem visualization, a rigid terrain patch, and an
interactive Irrlicht driver. The vehicle is expected to remain supported on the
flat terrain and respond to steering, throttle, and braking inputs in real time.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Named values keep vehicle, terrain, and loop parameters explicit and reusable.
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 5.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once
TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT


def build_simulation():
    # === Vehicle system ===
    # The HMMWV_Full wrapper owns the ChSystem; no separate system is created.
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  # prompt: TMEASY tire model
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystem reused below
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    vehicle = hmmwv.GetVehicle()  # cache: vehicle assembly reused for vis and assertions
    chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for logs and camera target
    print("VEHICLE MASS: ", vehicle.GetMass())

    # Wrapper-created essentials: chassis, suspensions, steering links, wheels,
    # tires, powertrain, and their joints are owned by veh.HMMWV_Full.
    tire_radius = vehicle.GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # cache: geometry check
    spindle_positions = []
    for axle in vehicle.GetAxles():
        for wheel in axle.m_wheels:
            spindle_positions.append(wheel.GetSpindle().GetPos())
    wheel_bottom_z = min(pos.z for pos in spindle_positions) - tire_radius
    assert wheel_bottom_z >= -0.05, (
        f"HMMWV wheel bottom z={wheel_bottom_z:.3f} is below terrain; "
        "raise INIT_LOC.z before running."
    )

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    # === Terrain ===
    # A single NSC rigid patch represents the flat support under the vehicle.
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization ===
    # Vehicle Irrlicht visualization follows the Initialize-then-scene-elements order.
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV on Rigid Terrain")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle)

    # === Driver ===
    # Interactive driver supplies keyboard steering, throttle, and braking controls.
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
    driver.Initialize()

    return hmmwv, system, vehicle, chassis, terrain, vis, driver


def run_simulation():
    hmmwv, system, vehicle, chassis, terrain, vis, driver = build_simulation()
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0


    try:
        # === Main loop ===
        # Render at 50 FPS and advance the full vehicle subsystem stack each step.
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            driver_inputs = driver.GetInputs()
            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError) as exc:
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    run_simulation()
