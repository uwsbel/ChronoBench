"""HMMWV driving on a single rigid height-map hill with NSC contact.

The scene uses a wrapper-owned PyChrono vehicle system, a Bullet collision
system, and one RigidTerrain patch generated from the bundled bump height map.
The vehicle drives across the textured rigid hill while Irrlicht renders the
terrain and chassis in real time.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants: keep terrain, driver, and render rates explicit ===
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_STEP_SIZE = 1.0 / 30.0
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 40.0
TERRAIN_WIDTH = 40.0
TERRAIN_MIN_HEIGHT = -1.0
TERRAIN_MAX_HEIGHT = 1.0
PATCH_TEXTURE_U = 6.0
PATCH_TEXTURE_V = 6.0

INIT_X = -15.0
INIT_Y = 0.0
INIT_Z = 1.1
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)


def build_vehicle():
    """Create the catalog HMMWV with NSC contact for rigid terrain."""
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    init_pos = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.QUNIT

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    system = hmmwv.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    return hmmwv, system


def build_terrain(system):
    """Create one rigid height-map patch and apply a dirt texture."""
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
        veh.GetDataFile("terrain/height_maps/bump64.bmp"),
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
        TERRAIN_MIN_HEIGHT,
        TERRAIN_MAX_HEIGHT,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), PATCH_TEXTURE_U, PATCH_TEXTURE_V)
    terrain.Initialize()
    return terrain


def build_visualization(hmmwv):
    """Create the vehicle-aware Irrlicht visual system after physics setup."""
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV on rigid height-map hill")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(hmmwv.GetVehicle())
    return vis


def main():
    """Build and run the rigid-terrain vehicle simulation."""
    hmmwv, system = build_vehicle()
    terrain = build_terrain(system)
    vis = build_visualization(hmmwv)

    chassis = hmmwv.GetChassisBody()  # cache: reused for review samples every step
    vehicle = hmmwv.GetVehicle()  # cache: wrapper subsystem used by driver and vis

    # === Driver: interactive core, deterministic review inputs when recording ===
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
    driver.Initialize()

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

    # === Main loop: synchronize every vehicle subsystem before advancing ===
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

        pos = chassis.GetPos()
        speed = vehicle.GetSpeed()

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError) as exc:  # Chrono runtime, invalid state, or output path failure
        traceback.print_exc()
        raise
