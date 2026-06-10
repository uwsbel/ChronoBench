"""M113 tracked vehicle driving on SCM deformable (Bekker-Wong) terrain.

Models the M113 armored personnel carrier (single-pin track shoes, shafts engine +
automatic-shafts transmission, BDS driveline, simple brakes) as an SMC tracked
vehicle. The vehicle starts at world location (-15, 0, 0) and drives forward under a
hard-coded throttle of 0.8 across an SCM soft-soil patch whose surface is generated
from a height map and textured with dirt. The tracks sink into and deform the soil,
leaving ruts as the vehicle accelerates.

System type: SMC (ChSystemSMC, owned by the veh.M113 wrapper).
Main bodies: tracked-vehicle chassis, sprockets/idlers/road-wheels/track-shoes, SCM
terrain grid. Expected behavior: the tank accelerates forward (+X) and the tracks
carve visible ruts into the deformable terrain.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Simulation constants === geometry / physics / timing for this run
step_size = 5e-4                     # tracked SMC contact needs a small step
sim_end = 8.0                        # seconds of simulated driving
render_fps = 50.0                    # review-video frame cadence
throttle_value = 0.8                 # prompt: hard-coded throttle during the loop

init_loc = chrono.ChVector3d(-15, 0, 0.0)   # prompt: initial vehicle location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
init_csys = chrono.ChCoordsysd(init_loc, init_rot)

# SCM terrain extent / resolution and its height-map source (dirt texture below)
terrain_length = 60.0
terrain_width = 30.0
terrain_resolution = 0.1
height_min = 0.0
height_max = 0.5

render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / step_size)   # precomputed once

# === Data paths === anchor bundled Chrono + vehicle assets (truth-faithful)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")
heightmap_file = veh.GetDataFile("terrain/height_maps/test64.bmp")
dirt_texture = chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg")

# === Vehicle === M113 tracked APC built by the veh.M113 wrapper (owns the system)
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)   # M113 truth uses SMC
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(init_csys)
vehicle.Initialize()

vis_type = veh.VisualizationType_PRIMITIVES
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# === System & bodies (created by the veh.M113 wrapper) ===
system = vehicle.GetSystem()                          # ChSystemSMC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
solver = chrono.ChSolverBB()                          # BB solver: stable tracked-on-SCM contact
solver.SetMaxIterations(120)
solver.SetOmega(0.8)
solver.SetSharpnessLambda(1.0)
system.SetSolver(solver)
system.SetMaxPenetrationRecoverySpeed(1.5)            # damp SCM contact recovery so tracks don't blow up
chassis = vehicle.GetChassisBody()                   # cache: main chassis rigid body, reused
# tracks/sprockets/idlers/road-wheels created inside the wrapper; SCM terrain built below
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain === SCM Bekker-Wong soft soil initialized from a height map
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e7,    # Bekker_Kphi   — frictional modulus (Pa); stiff enough to bear the ~11 t tank
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — sinkage exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    20,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear deformation modulus (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m), > Kphi before plastic yield
    3e4,    # damping_R     — vertical damping (Pa*s/m)
)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0),
                       chrono.ChVector3d(5, 3, 1))     # follow the chassis, not spindles
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.Initialize(heightmap_file, terrain_length, terrain_width,
                   height_min, height_max, terrain_resolution)
terrain.SetMeshWireframe(False)
terrain.SetTexture(dirt_texture, 80, 80)               # prompt: dirt terrain texture

# === Visualization === tracked-vehicle Irrlicht window: sky + camera + lights
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 1.0)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === scripted open-loop control (hard-coded throttle, no steering/brake)
driver = veh.ChDataDriver(vehicle.GetVehicle(), veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, throttle_value, 0.0),
    veh.DataDriverEntry(sim_end, 0.0, throttle_value, 0.0),
]))
driver.Initialize()

# === Main loop === real-time stepping with the full tracked-vehicle subsystem stack
try:

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    frame = 0
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:           # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)       # 2-arg for tracked vehicles
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)                     # advances the wrapper-owned system
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:              # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
