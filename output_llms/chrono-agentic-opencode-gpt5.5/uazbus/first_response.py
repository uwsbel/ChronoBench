"""PyChrono UAZBUS rigid-terrain driving demo.

This self-contained NSC vehicle simulation creates a UAZBUS catalog vehicle,
places it on a flat rigid terrain patch with specified contact friction and
restitution, and drives it forward with a scripted throttle while rendering the
real-time scene through the vehicle Irrlicht visual system.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants === fixed vehicle, terrain, and timing values for a repeatable demo
STEP_SIZE = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
RENDER_STEPS = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT

THROTTLE = 0.5
STEERING = 0.0
BRAKING = 0.0


def main():
    # === Vehicle system === UAZBUS wrapper owns the NSC system used by all subsystems
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.UAZBUS()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.SetTireStepSize(STEP_SIZE)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned system reused for terrain and timing
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = vehicle.GetChassisBody()  # cache: chassis pose logged every step
    veh_obj = vehicle.GetVehicle()  # cache: concrete vehicle handle reused by vis and driver
    print("VEHICLE MASS: ", veh_obj.GetMass())

    # Wrapper-created essentials: system, chassis, wheel/tire bodies, terrain, driver, and Irrlicht visualizer.
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === rigid flat patch gives the UAZBUS a contact support surface
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Visualization === vehicle Irrlicht window, chase camera, sky, logo, and directional light
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("UAZBUS rigid terrain")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(veh_obj)

    # === Driver === scripted core inputs keep the bus moving without keyboard focus
    driver = veh.ChDriver(veh_obj)
    driver.Initialize()

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0


    try:
        # === Main loop === synchronize and advance driver, terrain, vehicle, and visual system
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver.SetThrottle(THROTTLE)
            driver.SetSteering(STEERING)
            driver.SetBraking(BRAKING)
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
    except (RuntimeError, ValueError) as exc:  # guard: Chrono runtime/state failures
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    try:
        main()
    except (OSError, IOError) as exc:  # guard: output directory or file permission errors
        traceback.print_exc()
        raise
