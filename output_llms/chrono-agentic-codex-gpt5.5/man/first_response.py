"""MAN 10t truck dynamics on rigid terrain.

This PyChrono 9.0.0 NSC vehicle simulation uses the catalog MAN_10t wrapper,
TMEASY tires, a flat rigid terrain patch, an interactive Irrlicht driver, and
the vehicle Irrlicht chase-camera visual system. The truck is expected to drive
over the textured rigid ground while steering, throttle, and braking are handled
by real-time driver controls.
"""

import sys
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants === named parameters keep the vehicle and terrain setup readable
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 160.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.8)
INIT_ROT = chrono.QuatFromAngleZ(0.0)
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
CHASE_DISTANCE = 14.0
CHASE_HEIGHT = 1.0


def build_vehicle():
    """Create and initialize the MAN wrapper before taking its owned system."""
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

    system = truck.GetSystem()  # cache: wrapper-owned ChSystemNSC reused throughout
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("VEHICLE MASS: ", truck.GetVehicle().GetMass())

    truck.SetChassisVisualizationType(veh.VisualizationType_MESH)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH)

    return truck, system


def build_terrain(system):
    """Create the rigid textured support patch used by the truck tires."""
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
    return terrain


def build_visualization(truck):
    """Build the MAN vehicle visualizer with chase camera, sky, logo, and light."""
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("MAN 10t Truck on Rigid Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(truck.GetVehicle())
    return vis


def build_driver(vis):
    """Create the truth-style interactive Irrlicht driver for real-time control."""
    driver = veh.ChInteractiveDriverIRR(vis)
    steering_time = 1.0
    throttle_time = 1.0
    braking_time = 0.3
    driver.SetSteeringDelta(STEP_SIZE / steering_time)
    driver.SetThrottleDelta(STEP_SIZE / throttle_time)
    driver.SetBrakingDelta(STEP_SIZE / braking_time)
    driver.Initialize()
    return driver


def run():
    """Run the vehicle, terrain, driver, and visualizer synchronization loop."""
    truck, system = build_vehicle()

    # === System & bodies === wrapper-created system and rigid bodies made visible
    chassis = truck.GetChassisBody()  # cache: main chassis rigid body reused in logs
    vehicle_model = truck.GetVehicle()  # cache: full wheeled vehicle model
    # The MAN wrapper owns the chassis, suspension, steering, wheels, tires, joints,
    # powertrain, and the NSC system; terrain and visualization bind to that system.
    terrain = build_terrain(system)
    vis = build_visualization(truck)
    driver = build_driver(vis)

    # === Main loop === render once per frame, advance vehicle subsystems in batches
    realtime_timer = chrono.ChRealtimeStepTimer()
    frame = 0
    try:

        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()
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

                if system.GetChTime() >= SIM_END:
                    break

            realtime_timer.Spin(STEP_SIZE)

    except (RuntimeError, ValueError, OSError, IOError) as exc:
        print(f"simulation failed: {exc}", file=sys.stderr)
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    run()
