"""ARTcar vehicle simulation on rigid terrain using NSC contact.

The vehicle starts at (1, 0, 0.5), uses primitive visualization for its parts,
mesh collision on the chassis, and FIALA tires.  A rigid terrain patch supports
the vehicle while the standard Chrono vehicle subsystems synchronize and advance
in a real-time Irrlicht loop.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
# Named constants keep the vehicle setup and review run bounded and visible.
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_STEP = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP / STEP_SIZE))  # precomputed once
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_POS = chrono.ChVector3d(1.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT


def make_artcar():
    """Build the ARTcar with the requested spawn, visualization, collision, and tire model."""
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_MESH)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
    vehicle.SetTireType(veh.TireModelType_FIALA)  # prompt: FIALA tire model
    vehicle.SetTireStepSize(TIRE_STEP_SIZE)
    vehicle.Initialize()

    system = vehicle.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    vis_type = veh.VisualizationType_PRIMITIVES
    vehicle.SetChassisVisualizationType(vis_type)
    vehicle.SetSuspensionVisualizationType(vis_type)
    vehicle.SetSteeringVisualizationType(vis_type)
    vehicle.SetWheelVisualizationType(vis_type)
    vehicle.SetTireVisualizationType(vis_type)

    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
    return vehicle, system


def make_terrain(system):
    """Create a flat rigid terrain patch with NSC contact material."""
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 80, 20)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    return terrain


def make_visual_system(vehicle):
    """Create the vehicle Irrlicht visualizer after vehicle initialization."""
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("ARTcar FIALA Tire Demo")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.2), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle.GetVehicle())
    return vis


def make_driver(vis):
    """Create the standard interactive Irrlicht driver."""
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP / 1.0)
    driver.SetThrottleDelta(RENDER_STEP / 1.0)
    driver.SetBrakingDelta(RENDER_STEP / 0.3)
    driver.Initialize()
    return driver


def run():
    """Run the ARTcar scene."""
    vehicle, system = make_artcar()
    terrain = make_terrain(system)
    vis = make_visual_system(vehicle)
    driver = make_driver(vis)


    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

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
    except (RuntimeError, ValueError) as exc:
        print(f"Vehicle simulation failed: {exc}")
        traceback.print_exc()
        raise
    finally:
        print(f"completed_steps={step_number}")


if __name__ == "__main__":
    run()
