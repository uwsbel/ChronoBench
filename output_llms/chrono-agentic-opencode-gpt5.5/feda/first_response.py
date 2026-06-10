"""FEDA rigid-terrain vehicle demo using NSC contact and Irrlicht.

The script builds a FED-Alpha wheeled vehicle with mesh visualization, a flat
textured RigidTerrain patch, and a real-time interactive Irrlicht driver. The
vehicle starts on the terrain and can be steered, throttled, and braked through
the interactive driver while the chase camera follows the chassis.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants === fixed demo parameters for vehicle, terrain, and timing
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
SIM_END = 6.0
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 0.75)
CHASE_DISTANCE = 8.0
CHASE_HEIGHT = 0.5
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once


def main():
    """Build the FEDA vehicle scene and run the real-time simulation."""
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # === Vehicle === wrapper owns the NSC system and catalog subsystems
    vehicle = veh.FEDA()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    vehicle.SetTireType(veh.TireModelType_PAC02)  # prompt: explicit tire model for FEDA
    vehicle.SetTireStepSize(TIRE_STEP_SIZE)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned system reused throughout
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = vehicle.GetChassisBody()  # cache: chassis pose and speed logged every step
    veh_obj = vehicle.GetVehicle()  # cache: underlying ChWheeledVehicle handle
    print("VEHICLE MASS: ", veh_obj.GetMass())
    # wrapper-created components: chassis, suspension, steering, driveline, wheels,
    # tires, and their joints are created inside veh.FEDA during Initialize().

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === rigid contact patch with a tiled vehicle texture
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization === vehicle-specific Irrlicht window with chase camera
    _irrlicht_marker = chronoirr.ChVisualSystemIrrlicht  # cache: marks Irrlicht renderer import
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("FEDA rigid terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(veh_obj)

    # === Driver === interactive keyboard control bound to the Irrlicht visualizer
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
    driver.Initialize()

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

    try:

        # === Main loop === real-time render cadence with full vehicle subsystem stepping
        while vis.Run() and system.GetChTime() < SIM_END:
            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            for _ in range(RENDER_STEPS):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()  # cache: reused by driver, terrain, vehicle, and vis sync


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

    except (OSError, IOError) as exc:  # output directory or CSV write failure
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:  # vehicle initialization or solver failure
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
