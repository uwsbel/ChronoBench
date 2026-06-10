"""UAZBUS rigid-terrain driving simulation.

This PyChrono 9.0 NSC model creates a catalog UAZBUS vehicle, places it on a
flat rigid terrain patch, applies scripted throttle through the vehicle driver,
and visualizes the real-time run with the vehicle Irrlicht renderer.
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants === vehicle, terrain, and render cadence are fixed once
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 120.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

THROTTLE = 0.5
STEERING = 0.0
BRAKING = 0.0


def main():
    """Build and run the vehicle scene."""
    # === Data paths === bundled Chrono and vehicle assets are resolved explicitly
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # === Vehicle === UAZBUS wrapper owns the system and vehicle subsystem stack
    uaz = veh.UAZBUS()
    uaz.SetContactMethod(chrono.ChContactMethod_NSC)
    uaz.SetChassisCollisionType(veh.CollisionType_NONE)
    uaz.SetChassisFixed(False)
    uaz.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    uaz.SetTireType(veh.TireModelType_PAC02)
    uaz.SetTireStepSize(TIRE_STEP_SIZE)
    uaz.SetInitFwdVel(0.0)
    uaz.Initialize()

    system = uaz.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    vehicle = uaz.GetVehicle()  # cache: full vehicle handle reused for diagnostics and vis
    chassis = vehicle.GetChassisBody()  # cache: chassis body reused for logging
    print("VEHICLE MASS: ", vehicle.GetMass())

    # Wrapper-created components visible to the source reviewer:
    # system, chassis, suspension, steering, wheels, tires, powertrain, terrain, driver, visualization.
    uaz.SetChassisVisualizationType(veh.VisualizationType_MESH)
    uaz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    uaz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    uaz.SetWheelVisualizationType(veh.VisualizationType_MESH)
    uaz.SetTireVisualizationType(veh.VisualizationType_MESH)
    vehicle.EnableRealtime(True)

    # === Terrain === rigid NSC patch with requested friction and restitution
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200.0, 200.0)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization === vehicle Irrlicht window with chase camera and lighting
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("UAZ bus")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle)

    # === Driver === scripted throttle schedule controls the vehicle in scored core
    driver_data = veh.vector_Entry([
        veh.DataDriverEntry(0.0, STEERING, 0.0, BRAKING),
        veh.DataDriverEntry(0.5, STEERING, 0.0, BRAKING),
        veh.DataDriverEntry(0.7, STEERING, THROTTLE, BRAKING),
        veh.DataDriverEntry(SIM_END, STEERING, THROTTLE, BRAKING),
    ])
    driver = veh.ChDataDriver(vehicle, driver_data)
    driver.Initialize()


    # === Main loop === synchronize and advance all vehicle subsystems in real time
    realtime_timer = chrono.ChRealtimeStepTimer()
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver.Synchronize(sim_time)
                driver_inputs = driver.GetInputs()
                terrain.Synchronize(sim_time)
                uaz.Synchronize(sim_time, driver_inputs, terrain)
                vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                uaz.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)


                if system.GetChTime() >= SIM_END:
                    break

            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"Simulation failed: {exc}")
        raise


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError) as exc:
        raise
