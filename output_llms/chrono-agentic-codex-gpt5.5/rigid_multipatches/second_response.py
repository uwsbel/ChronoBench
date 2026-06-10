"""HMMWV full-vehicle simulation on a single rigid highway mesh.

The model uses an NSC HMMWV wrapper, Bullet collision, one rigid terrain mesh
patch for contact, and a separate triangle-mesh visual shape for the highway.
The vehicle starts at the requested location and runs in a real-time Irrlicht
loop with an interactive driver.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants: requested vehicle start, terrain assets, and run cadence ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_STEP_SIZE = 1.0 / 50.0
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

INIT_POS = chrono.ChVector3d(6.0, -70.0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(chrono.CH_PI / 2.0)
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
CHASE_DISTANCE = 9.0
CHASE_HEIGHT = 0.5

FRICTION = 0.9
RESTITUTION = 0.01
CONTACT_SWEEP_RADIUS = 0.01
HIGHWAY_COLLISION_MESH = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_col.obj")
HIGHWAY_VISUAL_MESH = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_vis.obj")


def add_highway_visual_mesh(patch):
    """Attach the requested visual highway mesh to the terrain ground body."""
    ground = patch.GetGroundBody()  # cache: terrain body reused for visual attachment
    visual_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(
        HIGHWAY_VISUAL_MESH, True, True
    )
    visual_shape = chrono.ChVisualShapeTriangleMesh()
    visual_shape.SetName("highway_visual_mesh")
    visual_shape.SetMesh(visual_mesh, True)
    visual_shape.SetBackfaceCull(False)
    ground.AddVisualShape(visual_shape, chrono.ChFramed())
    return ground


def build_simulation():
    # === Vehicle: full HMMWV wrapper owns the Chrono system ===
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    system = hmmwv.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

    # Wrapper-created components: vehicle system, chassis, suspensions, steering,
    # wheels, tires, powertrain, terrain, driver, and vehicle-aware visual system.
    chassis = hmmwv.GetChassisBody()  # cache: reused for diagnostics and chase target
    vehicle_core = hmmwv.GetVehicle()  # cache: reused for spindle checks

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain: one mesh contact patch plus requested visual mesh ===
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(FRICTION)
    patch_mat.SetRestitution(RESTITUTION)

    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        HIGHWAY_COLLISION_MESH,
        True,
        CONTACT_SWEEP_RADIUS,
        False,
    )
    terrain_ground = add_highway_visual_mesh(patch)  # cache: confirms visual mesh on terrain body
    terrain.Initialize()

    spindle_positions = []
    for axle_index in range(vehicle_core.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_positions.append(vehicle_core.GetSpindlePos(axle_index, side))
    min_spindle_z = min(pos.z for pos in spindle_positions)
    assert min_spindle_z > 0.2, (
        f"HMMWV spindle height is too low for the highway terrain: {min_spindle_z:.3f}"
    )

    # === Visualization and driver: vehicle Irrlicht window with interactive controls ===
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV on highway mesh")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(hmmwv.GetVehicle())

    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
    driver.Initialize()

    return hmmwv, system, chassis, terrain, terrain_ground, vis, driver


def run():
    hmmwv, system, chassis, terrain, terrain_ground, vis, driver = build_simulation()
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

    # === Main loop: synchronize and advance the full vehicle stack ===
    try:

        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()
            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver_inputs = driver.GetInputs()

            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)

    except (RuntimeError, ValueError, AssertionError) as exc:  # solver failure / invalid state
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:  # disk or asset access failure
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    run()
