"""M113 tracked vehicle on SCM deformable terrain.

This self-contained PyChrono 9.0.0 simulation uses the SMC contact method for a
tracked M113 crossing a height-map SCM soil patch.  The vehicle starts at
(-15, 0, 0), the soil uses Bekker-Wong parameters, the terrain is textured with
dirt, and the driver applies a constant 0.8 throttle in the simulation loop.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === define model, terrain, and loop values once for stable stepping
STEP_SIZE = 5.0e-4
SIM_END = 5.0
RENDER_FPS = 30.0
RENDER_STEPS = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once
INIT_LOC = chrono.ChVector3d(-15.0, 0.0, 0.0)
INIT_ROT = chrono.QUNIT
HEIGHTMAP_LENGTH = 40.0
HEIGHTMAP_WIDTH = 40.0
HEIGHTMAP_MIN = -0.45
HEIGHTMAP_MAX = 0.45
SCM_GRID = 0.10
THROTTLE_VALUE = 0.8


# === Vehicle setup === create the tracked wrapper and its owned SMC system
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
vehicle.SetWheelCollisionType(True, True)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemSMC reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
chassis = vehicle.GetChassisBody()  # cache: chassis body anchors the SCM moving patch
tracked_vehicle = vehicle.GetVehicle()  # cache: visualizer and driver use the same ChTrackedVehicle
tracked_vehicle.SetTrackShoeCollide(True)
left_track = tracked_vehicle.GetTrackAssembly(veh.LEFT)  # cache: left shoes queried for SCM forces
right_track = tracked_vehicle.GetTrackAssembly(veh.RIGHT)  # cache: right shoes queried for SCM forces
print("VEHICLE MASS: ", tracked_vehicle.GetMass())

VIS_TYPE = veh.VisualizationType_MESH
vehicle.SetChassisVisualizationType(VIS_TYPE)
vehicle.SetSprocketVisualizationType(VIS_TYPE)
vehicle.SetIdlerVisualizationType(VIS_TYPE)
vehicle.SetRoadWheelVisualizationType(VIS_TYPE)
vehicle.SetTrackShoeVisualizationType(VIS_TYPE)


# === SCM terrain === deformable height-map soil with dirt texture and moving active patch
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2.0e6,   # Bekker_Kphi
    0.0,     # Bekker_Kc
    1.1,     # Bekker_n
    0.0,     # Mohr_cohesion
    30.0,    # Mohr_friction
    0.01,    # Janosi_shear
    2.0e8,   # elastic_K
    3.0e4,   # damping_R
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.20)
terrain.AddMovingPatch(
    chassis,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(6.0, 4.0, 1.5),
)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    HEIGHTMAP_LENGTH,
    HEIGHTMAP_WIDTH,
    HEIGHTMAP_MIN,
    HEIGHTMAP_MAX,
    SCM_GRID,
)
terrain.SetMeshWireframe(False)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 8.0, 8.0)


# === Visualization === tracked-vehicle Irrlicht scene with vehicle-following camera
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 on SCM Height Map")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.3), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(tracked_vehicle)


# === Driver === base Chrono driver with loop-scripted throttle requested for this run
driver = veh.ChDriver(tracked_vehicle)
driver.Initialize()
driver.SetSteering(0.0)
driver.SetBraking(0.0)


# === Track-terrain forces === map SCM contact forces onto each track shoe for deformable terrain
def collect_track_forces(track_assembly):
    shoe_forces = veh.TerrainForces()
    for shoe_index in range(track_assembly.GetNumTrackShoes()):
        shoe_body = track_assembly.GetTrackShoe(shoe_index).GetShoeBody()
        force = chrono.ChVector3d(0.0, 0.0, 0.0)
        torque = chrono.ChVector3d(0.0, 0.0, 0.0)
        terrain.GetContactForceBody(shoe_body, force, torque)
        terrain_force = veh.TerrainForce()
        terrain_force.point = shoe_body.GetPos()
        terrain_force.force = force
        terrain_force.moment = torque
        shoe_forces.push_back(terrain_force)
    return shoe_forces


# === Main loop === synchronize driver, SCM terrain, tracked vehicle, and visualizer
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver.SetThrottle(THROTTLE_VALUE)
        driver.SetSteering(0.0)
        driver.SetBraking(0.0)
        driver_inputs = driver.GetInputs()  # cache: one input struct feeds all subsystems this step

        driver.Synchronize(time)
        terrain.Synchronize(time)
        left_forces = collect_track_forces(left_track)
        right_forces = collect_track_forces(right_track)
        vehicle.Synchronize(time, driver_inputs, left_forces, right_forces)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:
    raise
finally:
    pass
