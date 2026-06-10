"""MAN 10t rigid-terrain demo using PyChrono vehicle NSC contact.

The simulation builds the catalog MAN_10t truck, TMEASY tires, a flat rigid
terrain patch, an Irrlicht chase-camera visualizer, and an interactive driver
for steering, throttle, and braking.  The expected behavior is a real-time
truck driving demonstration on a textured rigid terrain.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named settings keep the vehicle demo reproducible
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
RENDER_STEP_SIZE = RENDER_EVERY * STEP_SIZE  # precomputed once

TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.70)
INIT_ROT = chrono.QUNIT
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
CHASE_DISTANCE = 12.0
CHASE_HEIGHT = 0.6

STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


def main():
    """Build and run the MAN 10t vehicle simulation."""
    try:
        # === Vehicle === catalog wrapper owns the ChSystem and vehicle bodies
        chrono.SetChronoDataPath(chrono.GetChronoDataPath())
        veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

        truck = veh.MAN_10t()
        truck.SetContactMethod(chrono.ChContactMethod_NSC)
        truck.SetChassisCollisionType(veh.CollisionType_NONE)
        truck.SetChassisFixed(False)
        truck.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
        truck.SetTireType(veh.TireModelType_TMEASY)  # prompt: TMEASY tire model
        truck.SetTireStepSize(TIRE_STEP_SIZE)
        truck.Initialize()

        system = truck.GetSystem()  # cache: wrapper-owned ChSystem reused below
        vehicle = truck.GetVehicle()  # cache: underlying vehicle reused below
        chassis = truck.GetChassisBody()  # cache: chassis body reused for logs/asserts
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        print("VEHICLE MASS: ", vehicle.GetMass())

        # Wrapper-created components: chassis/body tree, suspension links, steering,
        # driveline, wheels, tires, and the ChSystem are owned by MAN_10t.
        spindle_positions = []
        for axle_index in range(vehicle.GetNumberAxles()):
            for side in (veh.LEFT, veh.RIGHT):
                spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
        first_tire = vehicle.GetAxle(0).m_wheels[0].GetTire()  # cache: tire geometry for spawn assert
        wheel_bottom_z = min(p.z for p in spindle_positions) - first_tire.GetRadius()
        assert wheel_bottom_z >= -0.05, (
            f"truck wheel bottom starts below terrain: {wheel_bottom_z:.3f} m"
        )

        truck.SetChassisVisualizationType(veh.VisualizationType_MESH)
        truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
        truck.SetTireVisualizationType(veh.VisualizationType_MESH)

        # === Terrain === rigid textured road surface for tire contact
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


        # === Visualization === Irrlicht vehicle chase camera with sky, logo, and light
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("MAN 10t truck on rigid terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddLightDirectional()
        vis.AttachVehicle(vehicle)

        # === Driver === interactive controls for steering, throttle, and braking
        driver = veh.ChInteractiveDriverIRR(vis)
        driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
        driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
        driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
        driver.Initialize()

        realtime_timer = chrono.ChRealtimeStepTimer()
        frame = 0

        # === Main loop === synchronize driver, terrain, vehicle, and Irrlicht visualizer
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                driver.Synchronize(time)
                terrain.Synchronize(time)
                truck.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)


                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                truck.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)
                realtime_timer.Spin(STEP_SIZE)

                if system.GetChTime() >= SIM_END:
                    break

    except (OSError, IOError) as exc:  # file output or frame directory failure
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError, AssertionError) as exc:  # Chrono setup/solver state failure
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
