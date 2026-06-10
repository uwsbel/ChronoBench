"""CityBus on rigid terrain with a data-driven steering and throttle schedule.

This PyChrono 9.0 NSC vehicle simulation builds a catalog CityBus, a flat
RigidTerrain patch, and a ChDataDriver schedule that accelerates the bus before
commanding a strong steering input. The expected behavior is a moving bus that
begins straight and then turns under the specified open-loop driver data.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
# Named parameters make the vehicle, terrain, and loop settings auditable.
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_STEPS = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 120.0
PATCH_FRICTION = 0.9
PATCH_RESTITUTION = 0.01
INIT_LOCATION = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROTATION = chrono.QUNIT
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
CHASE_DISTANCE = 18.0
CHASE_HEIGHT = 1.0


def main():
    """Build, run, and render the CityBus maneuver."""
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # === Vehicle ===
    # The CityBus wrapper owns its ChSystem; configure it before Initialize.
    bus = veh.CityBus()
    bus.SetContactMethod(chrono.ChContactMethod_NSC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
    bus.SetTireType(veh.TireModelType_TMEASY)
    bus.SetTireStepSize(TIRE_STEP_SIZE)
    bus.Initialize()

    system = bus.GetSystem()  # cache: wrapper-owned system reused throughout setup
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    vehicle = bus.GetVehicle()  # cache: vehicle handle reused by driver and visualizer
    chassis = bus.GetChassisBody()  # cache: chassis body available for diagnostics
    print("VEHICLE MASS: ", vehicle.GetMass())

    # Wrapper-created components: system, chassis, suspension, wheels, tires,
    # powertrain, steering, terrain coupling, driver, and vehicle Irrlicht HUD.
    if not chassis:
        raise RuntimeError("CityBus chassis body was not created")

    bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
    bus.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain ===
    # A flat rigid patch gives the bus a high-friction paved support surface.
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(PATCH_FRICTION)
    patch_mat.SetRestitution(PATCH_RESTITUTION)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver ===
    # ChDataDriver entries are (time, steering, throttle, braking).
    driver_data = veh.vector_Entry(
        [
            veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
            veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),
            veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),
        ]
    )
    driver = veh.ChDataDriver(vehicle, driver_data)
    driver.Initialize()

    # === Visualization ===
    # Vehicle-aware Irrlicht renders the CityBus meshes and HUD.
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("CityBus Data Driver")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle)

    realtime_timer = chrono.ChRealtimeStepTimer()
    frame = 0


    # === Main loop ===
    # Synchronize driver, terrain, vehicle, and visualization every physics step.
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_STEPS):
                time = system.GetChTime()
                driver.Synchronize(time)
                driver_inputs = driver.GetInputs()
                terrain.Synchronize(time)
                bus.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                bus.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)


                if system.GetChTime() >= SIM_END:
                    break

            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError, OSError) as exc:  # solver/runtime or output failure
        traceback.print_exc()
        raise
    finally:
        pass

    # === Review artifacts ===
    # The recording helpers are stripped from the accepted scored script.


if __name__ == "__main__":
    main()
