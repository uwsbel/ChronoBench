"""M113 tracked-vehicle mobility test on rigid terrain with SMC contact.

The scene contains a catalog M113 tracked vehicle initialized at (-5, 0, 0.5),
a flat terrain patch, and a fixed long box obstacle placed in the vehicle path.
The driver throttle is set to 0.8 during the real-time loop so the vehicle
drives forward and interacts with the long box.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 2.0e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

INIT_LOC = chrono.ChVector3d(-5.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
THROTTLE_VALUE = 0.8

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 20.0
TERRAIN_Z = -0.16
BOX_SIZE_X = 9.0
BOX_SIZE_Y = 2.0
BOX_SIZE_Z = 0.12
BOX_POS = chrono.ChVector3d(1.0, 0.0, TERRAIN_Z + BOX_SIZE_Z / 2.0)


# === Vehicle system ===
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

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused by terrain, box, and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(150)
system.GetSolver().AsIterative().SetTolerance(1.0e-5)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

chassis = vehicle.GetChassisBody()  # cache: main chassis body reused for diagnostics
tracked_vehicle = vehicle.GetVehicle()  # cache: tracked vehicle subsystem reused by vis and driver
# The wrapper owns the chassis, track shoes, sprockets, idlers, road wheels, suspension joints, and driveline.


# === Terrain and long box ===
terrain_mat = chrono.ChContactMaterialSMC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)
terrain_mat.SetYoungModulus(2.0e7)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    terrain_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, TERRAIN_Z), chrono.QUNIT),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 40, 10)
patch.SetColor(chrono.ChColor(0.45, 0.50, 0.42))
terrain.Initialize()

box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.85)
box_mat.SetRestitution(0.02)
box_mat.SetYoungModulus(2.0e7)

long_box = chrono.ChBodyEasyBox(BOX_SIZE_X, BOX_SIZE_Y, BOX_SIZE_Z, 1000.0, True, True, box_mat)
long_box.SetName("long_box_mobility_test")
long_box.SetPos(BOX_POS)
long_box.SetFixed(True)
long_box.EnableCollision(True)
system.AddBody(long_box)


# === Visualization ===
vis_type = veh.VisualizationType_MESH
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Mobility Test")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.2), 10.0, 0.6)
vis.AttachVehicle(tracked_vehicle)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()

driver = veh.ChDriver(tracked_vehicle)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop ===

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver.SetThrottle(THROTTLE_VALUE)
            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()


            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
