"""Two BMW_E90 sedans on NSC rigid terrain with concrete texture.

The simulation uses PyChrono vehicle wrappers on a shared Bullet collision system.
Both sedans drive on the same rigid terrain with initialized positions and
orientations, independent driver systems, and sinusoidal steering commands.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Parameters === named constants keep the vehicle setup and loop reproducible
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 240.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
CHASSIS_Z = 0.5
PRIMARY_INIT_POS = chrono.ChVector3d(-20.0, -2.0, CHASSIS_Z)
SECOND_INIT_POS = chrono.ChVector3d(-25.0, 8.0, CHASSIS_Z)
PRIMARY_INIT_ROT = chrono.QUNIT
SECOND_INIT_ROT = chrono.QuatFromAngleAxis(0.08, chrono.ChVector3d(0, 0, 1))
STEERING_AMPLITUDE = 0.15
STEERING_FREQUENCY = 0.7
THROTTLE_VALUE = 0.45


def configure_driver(driver, time, phase):
    """Apply the requested sinusoidal open-loop steering profile."""
    driver.SetThrottle(THROTTLE_VALUE)
    driver.SetBraking(0.0)
    driver.SetSteering(STEERING_AMPLITUDE * math.sin(STEERING_FREQUENCY * time + phase))


def main():
    # === Vehicle system & bodies === wrappers create the shared ChSystem and sedan bodies
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.BMW_E90()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(PRIMARY_INIT_POS, PRIMARY_INIT_ROT))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(TIRE_STEP_SIZE)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned system reused by terrain and second car
    vehicle_chassis = vehicle.GetChassisBody()  # cache: primary chassis reused for logging

    vehicle2 = veh.BMW_E90(system)
    vehicle2.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle2.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle2.SetChassisFixed(False)
    vehicle2.SetInitPosition(chrono.ChCoordsysd(SECOND_INIT_POS, SECOND_INIT_ROT))
    vehicle2.SetTireType(veh.TireModelType_TMEASY)
    vehicle2.SetTireStepSize(TIRE_STEP_SIZE)
    vehicle2.Initialize()

    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    vehicle2_chassis = vehicle2.GetChassisBody()  # cache: second chassis reused for logging
    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
    print("SECOND VEHICLE MASS: ", vehicle2.GetVehicle().GetMass())
    # wrapper-created components: shared system, two chassis bodies, suspension links,
    # steering links, wheels, tires, and drivetrain assemblies are owned by BMW_E90.

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    vehicle2.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle2.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle2.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle2.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle2.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === one rigid concrete patch supports both sedans
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 300, 100)
    patch.SetColor(chrono.ChColor(0.55, 0.55, 0.55))
    terrain.Initialize()

    # === Drivers === independent driver systems command sinusoidal steering
    driver = veh.ChDriver(vehicle.GetVehicle())
    driver.Initialize()
    driver2 = veh.ChDriver(vehicle2.GetVehicle())
    driver2.Initialize()

    # === Visualization === vehicle-aware Irrlicht view with sky and directional light
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Two Sedan Concrete Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.2), 10.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle.GetVehicle())
    vis.AttachSystem(system)


    # === Main loop === synchronize and advance both vehicles on the shared terrain
    realtime_timer = chrono.ChRealtimeStepTimer()
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                configure_driver(driver, time, 0.0)
                configure_driver(driver2, time, math.pi / 2.0)
                driver_inputs = driver.GetInputs()  # cache: primary inputs used for sync and HUD
                driver2_inputs = driver2.GetInputs()  # cache: second inputs used for sync

                driver.Synchronize(time)
                driver2.Synchronize(time)
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, terrain)
                vehicle2.Synchronize(time, driver2_inputs, terrain)
                vis.Synchronize(time, driver_inputs)


                driver.Advance(STEP_SIZE)
                driver2.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                vehicle.Advance(STEP_SIZE)
                vehicle2.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)
                realtime_timer.Spin(STEP_SIZE)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid vehicle state
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    try:
        main()
    except (OSError, IOError) as exc:  # output path / file permission failures
        traceback.print_exc()
        raise
    except RuntimeError as exc:  # PyChrono runtime or renderer initialization failures
        traceback.print_exc()
        raise
    finally:
        pass
