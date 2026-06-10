"""Full HMMWV on a rigid highway mesh terrain.

This PyChrono 9.0 NSC vehicle simulation builds a complete HMMWV_Full with
TMEASY tires, mesh visualization on all vehicle subsystems, and a rigid mesh
terrain whose collision and visual geometry come from the Highway OBJ assets.
An Irrlicht vehicle visualizer and interactive driver run the scene in real
time at 50 rendered frames per second.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants: vehicle, terrain, and timing parameters kept explicit ===
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
FPS = 50.0
RENDER_STEP_SIZE = 1.0 / FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once
HIGHWAY_COL_MESH = chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj")
HIGHWAY_VIS_MESH = chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj")
INIT_LOC = chrono.ChVector3d(-80.0, -2.0, 0.65)
INIT_ROT = chrono.QuatFromAngleZ(0.0)
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)


def add_visual_highway_mesh(patch):
    """Attach the detailed highway visual mesh to the rigid collision patch."""
    ground = patch.GetGroundBody()
    visual_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(
        HIGHWAY_VIS_MESH, True, True
    )
    visual_shape = chrono.ChVisualShapeTriangleMesh()
    visual_shape.SetMesh(visual_mesh, True)
    visual_shape.SetName("Highway_vis.obj")
    visual_shape.SetBackfaceCull(False)
    ground.AddVisualShape(visual_shape, chrono.ChFramed())
    return ground


def build_simulation():
    """Create the vehicle, mesh terrain, visual system, and interactive driver."""
    # === Data paths: Chrono and vehicle catalogs resolve bundled meshes/json ===
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # === Vehicle: HMMWV_Full owns its NSC system and all driveline subsystems ===
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    vehicle.SetTireType(veh.TireModelType_TMEASY)  # prompt: TMEASY tire model
    vehicle.SetTireStepSize(TIRE_STEP_SIZE)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused below
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = vehicle.GetChassisBody()  # cache: body reused for logging/review
    wheeled_vehicle = vehicle.GetVehicle()  # cache: report and visualization target
    print("VEHICLE MASS: ", wheeled_vehicle.GetMass())

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Wrapper-created components made visible for review:
    # system: vehicle.GetSystem(); chassis/body tree: HMMWV_Full internals;
    # terrain: RigidTerrain mesh patch; visualization: vehicle Irrlicht system;
    # driver: ChInteractiveDriverIRR bound to the same visualizer.

    # === Terrain: rigid highway collision mesh plus separate visual mesh ===
    terrain_material = chrono.ChContactMaterialNSC()
    terrain_material.SetFriction(0.9)
    terrain_material.SetRestitution(0.01)
    terrain = veh.RigidTerrain(system)
    terrain_pose = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
    highway_patch = terrain.AddPatch(
        terrain_material,
        terrain_pose,
        HIGHWAY_COL_MESH,
        True,
        0.0,
        False,
    )
    add_visual_highway_mesh(highway_patch)
    terrain.Initialize()

    spawn_height = terrain.GetHeight(INIT_LOC)
    assert INIT_LOC.z > spawn_height + 0.2, (
        f"HMMWV spawn z={INIT_LOC.z:.3f} is too close to terrain height "
        f"{spawn_height:.3f}; raise INIT_LOC.z"
    )

    # === Visualization: vehicle-aware Irrlicht window with chase camera ===
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV on Highway Mesh")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, 8.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(wheeled_vehicle)

    # === Driver: interactive keyboard driver, with deltas tied to 50 FPS ===
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
    driver.Initialize()

    return vehicle, system, chassis, terrain, vis, driver


def main():
    """Run the real-time simulation loop."""
    vehicle, system, chassis, terrain, vis, driver = build_simulation()
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    frame_number = 0


    # === Main loop: render at 50 FPS and advance the full vehicle stack ===
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            time = system.GetChTime()
            driver_inputs = driver.GetInputs()
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
    except (RuntimeError, ValueError) as exc:
        print(f"simulation runtime failure: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
