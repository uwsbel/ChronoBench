"""Kraz tractor-trailer vehicle simulation on rigid terrain.

This PyChrono 9.0 script builds a Kraz catalog vehicle with NSC contact, a
flat rigid terrain patch with explicit friction and restitution, an Irrlicht
vehicle visualization window, and an interactive driver. The loop synchronizes
and advances the driver, terrain, vehicle, and visual system in real time.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants: visible vehicle, terrain, and timing parameters ===
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_STEP_SIZE = 1.0 / 50.0
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_LOCATION = chrono.ChVector3d(0.0, 0.0, 0.75)
INIT_ROTATION = chrono.QUNIT
INIT_FWD_VEL = 0.0
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


def build_simulation():
    """Create the Kraz vehicle, terrain, visual system, and driver."""

    # === Data paths: catalog vehicle and visual assets ===
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # === Vehicle: Kraz wrapper owns the Chrono system ===
    kraz = veh.Kraz()
    kraz.SetContactMethod(chrono.ChContactMethod_NSC)
    kraz.SetChassisCollisionType(veh.CollisionType_NONE)
    kraz.SetChassisFixed(False)
    kraz.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
    kraz.SetInitFwdVel(INIT_FWD_VEL)
    kraz.SetTireStepSize(TIRE_STEP_SIZE)
    kraz.Initialize()

    system = kraz.GetSystem()  # cache: wrapper-owned system used by terrain and timing
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    tractor = kraz.GetTractor()  # cache: visualizer attaches to the Kraz tractor vehicle
    chassis = kraz.GetTractorChassisBody()  # cache: reused for diagnostics/logging

    # The wrapper-created essentials are system, tractor/trailer bodies, suspension,
    # tires, terrain contacts, the vehicle visualizer, and the interactive driver.
    print("VEHICLE MASS: ", tractor.GetMass())

    # === Visualization types: mesh bodywork, primitive suspension/steering ===
    kraz.SetChassisVisualizationType(
        veh.VisualizationType_MESH,
        veh.VisualizationType_MESH,
    )
    kraz.SetSuspensionVisualizationType(
        veh.VisualizationType_PRIMITIVES,
        veh.VisualizationType_PRIMITIVES,
    )
    kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetWheelVisualizationType(
        veh.VisualizationType_MESH,
        veh.VisualizationType_MESH,
    )
    kraz.SetTireVisualizationType(
        veh.VisualizationType_MESH,
        veh.VisualizationType_MESH,
    )

    # === Terrain: rigid flat patch with explicit contact material ===
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
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Irrlicht vehicle visualization: initialize before scene additions ===
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Kraz vehicle on rigid terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 14.0, 0.8)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AddGrid(
        5.0,
        5.0,
        20,
        20,
        chrono.ChCoordsysd(),
        chrono.ChColor(0.35, 0.35, 0.35),
    )
    vis.AttachVehicle(tractor)

    # === Driver: real-time interactive control bound to the Irrlicht window ===
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
    driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
    driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
    driver.Initialize()

    return kraz, system, chassis, terrain, vis, driver


def run_simulation(data_writer=None):
    """Run the explicit subsystem synchronization and real-time advance loop."""

    kraz, system, chassis, terrain, vis, driver = build_simulation()
    render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0


    # === Main loop: render once per visual frame, step all vehicle subsystems ===
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()  # cache: one input snapshot per step

        driver.Synchronize(time)
        terrain.Synchronize(time)
        kraz.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        kraz.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)


# === Entrypoint: named error handling with optional review artifacts ===
if __name__ == "__main__":
    try:
        run_simulation()
    except (RuntimeError, ValueError, OSError, IOError) as exc:
        print(f"Kraz simulation failed: {exc}")
        raise
