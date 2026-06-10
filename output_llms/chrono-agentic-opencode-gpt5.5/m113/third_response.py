"""M113 tracked vehicle mobility demo on SMC rigid terrain.

The simulation uses PyChrono's catalog M113 tracked vehicle with SMC contact,
a flat rigid terrain patch, and a fixed low box obstacle spanning the lane. The
vehicle starts at (-5, 0, 0.5) and drives forward with a hard-coded throttle of
0.8 so its tracked mobility over the obstacle can be observed.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named values make the vehicle, terrain, and obstacle setup explicit
STEP_SIZE = 1e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once
THROTTLE_VALUE = 0.8
TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 20.0
OBSTACLE_X = 2.0
OBSTACLE_Y = 0.0
OBSTACLE_Z = 0.125
OBSTACLE_LENGTH_X = 0.25
OBSTACLE_LENGTH_Y = 8.0
OBSTACLE_HEIGHT = 0.25


# === Vehicle and system === catalog M113 owns the system and supplies the tracked bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

init_loc = chrono.ChVector3d(-5.0, 0.0, 0.5)
init_rot = chrono.QUNIT

vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemSMC reused below
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chassis = vehicle.GetChassisBody()  # cache: main rigid body reused for logging
vehicle_model = vehicle.GetVehicle()  # cache: tracked vehicle handle reused by vis/driver
# wrapper-created bodies: chassis, sprockets, idlers, road wheels, and track shoes
# wrapper-created joints: suspension, drivetrain, track constraints, and brakes
print("VEHICLE MASS: ", vehicle_model.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSprocketVisualizationType(veh.VisualizationType_MESH)
vehicle.SetIdlerVisualizationType(veh.VisualizationType_MESH)
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)


# === Terrain and obstacle === SMC contact material matches the tracked vehicle system
terrain_material = chrono.ChContactMaterialSMC()
terrain_material.SetFriction(0.9)
terrain_material.SetRestitution(0.01)
terrain_material.SetYoungModulus(2e7)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(terrain_material, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.6))
terrain.Initialize()

obstacle = chrono.ChBodyEasyBox(
    OBSTACLE_LENGTH_X,
    OBSTACLE_LENGTH_Y,
    OBSTACLE_HEIGHT,
    1000.0,
    True,
    True,
    terrain_material,
)
obstacle.SetName("long_mobility_box")
obstacle.SetPos(chrono.ChVector3d(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z))
obstacle.SetFixed(True)
system.AddBody(obstacle)


# === Visualization and driver === tracked Irrlicht visualizer follows the M113 chassis
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Mobility With Long Box")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_model)

driver = veh.ChDriver(vehicle_model)
driver.Initialize()
realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop === render at frame cadence while advancing all tracked vehicle subsystems
try:

    frame = 0
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver.Synchronize(time)
            driver.SetSteering(0.0)
            driver.SetThrottle(THROTTLE_VALUE)
            driver.SetBraking(0.0)
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
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
    raise
except (OSError, IOError) as exc:  # output path and recording I/O guard
    raise
finally:
    print("Simulation finished at time", system.GetChTime())
