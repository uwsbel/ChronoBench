"""BMW E90 sedan on rigid terrain using NSC contact and Irrlicht.

The simulation builds the catalog BMW E90 Sedan with a TMEASY tire model, a
flat textured rigid terrain patch, an interactive Irrlicht driver, and a chase
camera. The vehicle is expected to respond to steering, throttle, and braking
inputs while driving over the rigid road surface.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === simulation timing, terrain, and driver response values
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = STEP_SIZE
RENDER_STEP_SIZE = 1.0 / 50.0
SIM_END = 8.0
TERRAIN_LENGTH = 500.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
CHASE_DISTANCE = 6.0
CHASE_HEIGHT = 0.5
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once


def main():
    # === Vehicle system === catalog wrapper creates the NSC system and sedan bodies
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    sedan = veh.BMW_E90()
    sedan.SetContactMethod(chrono.ChContactMethod_NSC)
    sedan.SetChassisCollisionType(veh.CollisionType_NONE)
    sedan.SetChassisFixed(False)
    sedan.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    sedan.SetTireType(veh.TireModelType_TMEASY)  # prompt: TMEASY tire model
    sedan.SetTireStepSize(TIRE_STEP_SIZE)
    sedan.Initialize()

    system = sedan.GetSystem()  # cache: wrapper-owned ChSystem reused for terrain and timing
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    vehicle = sedan.GetVehicle()  # cache: wrapper vehicle reused for mass and visualization
    chassis = sedan.GetChassisBody()  # cache: chassis body reused for logging
    axles = vehicle.GetAxles()  # cache: axle list used for footprint verification
    print("VEHICLE MASS: ", vehicle.GetMass())

    # Wrapper-created components: system, chassis, suspensions, steering, wheels,
    # tires, powertrain, and joints are owned by veh.BMW_E90; terrain is added below.
    tire_radius = axles[0].m_wheels[0].GetTire().GetRadius()
    spindle_positions = []
    for axle_index in range(vehicle.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
    wheel_bottom_z = min(pos.z for pos in spindle_positions) - tire_radius
    assert wheel_bottom_z >= -0.05, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f}; "
        "raise INIT_LOC.z"
    )

    sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
    sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
    sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === rigid textured road patch paired with NSC contact material
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.Initialize()

    # === Visualization and driver === Irrlicht chase camera plus interactive controls
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("BMW E90 Sedan on Rigid Terrain")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle)

    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
    driver.Initialize()

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

    # === Main loop === synchronize full vehicle stack and advance at fixed step
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            sedan.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            sedan.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid vehicle state
        traceback.print_exc()
        raise exc
    except (OSError, IOError) as exc:  # recording path or frame capture failure
        traceback.print_exc()
        raise exc
    finally:
        pass


if __name__ == "__main__":
    main()
