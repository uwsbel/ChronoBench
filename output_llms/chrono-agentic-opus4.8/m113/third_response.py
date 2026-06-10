"""M113 tracked-vehicle mobility test on rigid terrain (PyChrono, SMC system).

Models the M113 tracked armored vehicle (single-pin track shoes, shaft engine +
automatic-shaft transmission, BDS driveline) spawned at (-5, 0, 0.5) and driven
forward at a constant 0.8 throttle across a flat rigid-terrain patch. A long
rigid box is placed on the ground ahead of the vehicle as an obstacle to probe
tracked-vehicle mobility. Contact is handled by the Bullet collision system with
a Barzilai-Borwein solver (the stable choice for tracked contact). Expected
behavior: the vehicle accelerates forward under steady throttle and interacts
with the box, demonstrating mobility.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants, then derived positions
step_size = 1e-4                       # small step for stable tracked SMC contact
sim_end = 10.0                         # simulation horizon (s)
render_fps = 50.0                      # review-frame cadence
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / step_size)              # precomputed once

throttle_value = 0.8                   # hard-coded constant throttle
init_loc = chrono.ChVector3d(-5, 0, 0.5)   # vehicle spawn (prompt-specified)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

box_size = chrono.ChVector3d(8.0, 1.0, 0.5)   # long obstacle box (full extents)
box_pos = chrono.ChVector3d(6.0, 0, box_size.z / 2.0)  # ahead of the vehicle

terrain_length = 100.0
terrain_width = 100.0


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === M113 tracked vehicle (wrapper owns its ChSystem)
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)   # M113 truth uses SMC
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

vis_type = veh.VisualizationType_PRIMITIVES
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# === System & bodies (created by the veh.M113 wrapper) ===
system = vehicle.GetSystem()                       # ChSystemSMC owned by the wrapper
chassis = vehicle.GetChassisBody()                 # cache: main chassis rigid body, reused below
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)           # stable tracked-contact solver
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain === flat rigid patch under the vehicle (SMC material)
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Obstacle box === long rigid box added to test vehicle mobility
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.9)
box_mat.SetRestitution(0.01)
box_mat.SetYoungModulus(2e7)
box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000.0, True, True, box_mat)
box.SetPos(box_pos)
box.SetFixed(True)                                 # static obstacle on the ground
box.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.5, 0.6))
system.Add(box)

# === Driver === scripted constant-throttle control (tracked vehicle)
driver = veh.ChDataDriver(vehicle.GetVehicle(), veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.0, throttle_value, 0.0),
    veh.DataDriverEntry(sim_end, 0.0, throttle_value, 0.0),
]))
driver.Initialize()

# === Visualization === tracked-vehicle Irrlicht window (sky + camera + lights)
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Mobility Test")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Main loop === advance driver/terrain/vehicle/vis; constant 0.8 throttle
os.makedirs("cam", exist_ok=True)   # guard against missing output dir

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)   # 2-arg for tracked vehicles
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + physics plot (review only)
