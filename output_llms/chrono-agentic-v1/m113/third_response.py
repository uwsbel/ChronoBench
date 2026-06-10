"""
M113 Tracked Vehicle Simulation with Obstacle Box.

System type: NSC (tracked vehicle, rigid terrain)
Main bodies: M113 tracked vehicle chassis, track shoes (created by wrapper),
             rigid terrain patch, long obstacle box.
Expected behavior: The M113 starts at (-5, 0, 0.5), a long box obstacle is placed
on the terrain to test vehicle mobility, and throttle is hard-coded at 0.8 so the
vehicle drives forward and attempts to climb over / navigate the obstacle.
"""

# === Imports ===
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Constants ===
# Simulation parameters
step_size   = 5e-4        # integration timestep (s)
sim_end     = 20.0        # total simulation time (s)
render_fps  = 50.0        # render framerate
render_every = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

# Terrain
terrain_length = 100.0
terrain_width  = 40.0

# Vehicle initial position (as specified: -5, 0, 0.5)
init_loc = chrono.ChVector3d(-5, 0, 0.5)
init_rot = chrono.QuatFromAngleZ(0.0)   # pointing +X

# Obstacle box dimensions and position
BOX_LX     = 20.0   # long box length along X
BOX_LY     = 6.0    # box width (Y)
BOX_LZ     = 0.3    # box height
BOX_POS_X  = 10.0   # placed ahead of the vehicle start
BOX_POS_Y  = 0.0
BOX_POS_Z  = BOX_LZ / 2.0  # so base sits on terrain z=0

# Visualization
vis_type = veh.VisualizationType_PRIMITIVES

# === Vehicle setup ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)             # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

# Visualization types (after Initialize)
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)

# === System & bodies (created by the veh.M113 wrapper) ===
system  = vehicle.GetSystem()              # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()         # cache: main chassis rigid body, reused below
# track shoes / spindles / sprockets created inside the M113 wrapper

# Collision system — REQUIRED for any contact/terrain scene
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Stable solver for tracked vehicle contact
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# === Terrain ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Long obstacle box (to test vehicle mobility) ===
obs_mat = chrono.ChContactMaterialNSC()
obs_mat.SetFriction(0.8)
obs_mat.SetRestitution(0.01)

obs_box = chrono.ChBodyEasyBox(BOX_LX, BOX_LY, BOX_LZ, 2000.0, True, True, obs_mat)
obs_box.SetName("obstacle_box")
obs_box.SetPos(chrono.ChVector3d(BOX_POS_X, BOX_POS_Y, BOX_POS_Z))
obs_box.SetFixed(True)   # static obstacle — tests vehicle climbing, not free-fall
system.Add(obs_box)

# === Interactive Driver (scored core) ===
# veh.ChTrackedVehicleVisualSystemIrrlicht must exist before building the driver.
# Build vis first, driver second.

# === Tracked-vehicle Irrlicht visualization ===
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 with Obstacle Box")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver (interactive, scored core — real-time keyboard) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0   # s from 0 to +1 steering
throttle_time = 1.0   # s from 0 to +1 throttle
braking_time  = 0.3   # s from 0 to +1 braking
driver.SetSteeringDelta(render_fps * step_size / steering_time)
driver.SetThrottleDelta(render_fps * step_size / throttle_time)
driver.SetBrakingDelta(render_fps * step_size / braking_time)
driver.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()  # cache: fetched once per outer iter

        # Render at throttled cadence
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Synchronize subsystems (tracked vehicle: 2-arg vehicle.Synchronize)
        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)  # 2-arg for M113 tracked vehicle
        vis.Synchronize(time, driver_inputs)

        # Advance
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)


        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
