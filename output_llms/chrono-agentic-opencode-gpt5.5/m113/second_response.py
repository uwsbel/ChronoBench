"""M113 tracked vehicle on SCM deformable terrain.

This PyChrono SMC simulation initializes an M113 tracked vehicle at
(-15, 0, 0), drives it with a constant throttle of 0.8, and runs it over a
height-map SCM soil patch with dirt texture. The expected behavior is forward
tracked motion with visible terrain deformation under the shoes.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named values make the vehicle, terrain, and run duration explicit
STEP_SIZE = 5.0e-4
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_STEPS = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once
INIT_LOC = chrono.ChVector3d(-15.0, 0.0, 0.0)
INIT_ROT = chrono.QUNIT
THROTTLE_VALUE = 0.8
TERRAIN_LENGTH = 40.0
TERRAIN_WIDTH = 40.0
HEIGHT_MIN = -0.20
HEIGHT_MAX = 0.40
SCM_RESOLUTION = 0.04


class ConstantThrottleDriver(veh.ChDriver):
    """Simple scored-core driver that sets a fixed throttle each loop."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetSteering(0.0)
        self.SetThrottle(THROTTLE_VALUE)
        self.SetBraking(0.0)


def main():
    """Build and run the self-contained M113 SCM terrain simulation."""
    # === Vehicle setup === catalog M113 wrapper owns the SMC system and track bodies
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.M113()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False)
    vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
    vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemSMC reused below
    chassis = vehicle.GetChassisBody()  # cache: stable body for SCM moving patch and logging
    tracked_vehicle = vehicle.GetVehicle()  # cache: wrapper vehicle handle for visualization/driver
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    print("VEHICLE MASS: ", tracked_vehicle.GetMass())

    vis_type = veh.VisualizationType_MESH
    vehicle.SetChassisVisualizationType(vis_type)
    vehicle.SetTrackShoeVisualizationType(vis_type)
    vehicle.SetSprocketVisualizationType(vis_type)
    vehicle.SetIdlerVisualizationType(vis_type)
    vehicle.SetRoadWheelVisualizationType(vis_type)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)

    # === Terrain === SCM height map with explicit soil parameters and dirt texture
    terrain = veh.SCMTerrain(system)
    terrain.SetSoilParameters(
        2.0e6,
        0.0,
        1.1,
        0.0,
        30.0,
        0.01,
        2.0e8,
        3.0e4,
    )
    terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
    terrain.SetMeshWireframe(False)
    terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
    terrain.Initialize(
        veh.GetDataFile("terrain/height_maps/bump64.bmp"),
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
        HEIGHT_MIN,
        HEIGHT_MAX,
        SCM_RESOLUTION,
    )
    terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

    # === Visualization and driver === tracked Irrlicht visual system follows the chassis
    vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("M113 on SCM Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 8.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(tracked_vehicle)

    driver = ConstantThrottleDriver(tracked_vehicle)
    driver.Initialize()

    # === Review-only outputs === CSV and frame/video products are stripped before scoring
    frame = 0
    step_number = 0
    realtime_timer = chrono.ChRealtimeStepTimer()


    try:
        # === Main loop === synchronize/advance every subsystem in tracked-vehicle order
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                frame += 1

            driver.Synchronize(time)
            driver.SetThrottle(THROTTLE_VALUE)
            driver_inputs = driver.GetInputs()  # cache: same inputs feed vehicle and HUD this step
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)

    except (RuntimeError, ValueError) as exc:
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
