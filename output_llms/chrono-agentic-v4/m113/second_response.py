"""
M113 Tracked Vehicle on SCM Deformable Terrain (Turn 2).

- M113 tracked vehicle (veh.M113 wrapper)
- SCM (Bekker-Wong) deformable soft-soil terrain
- Hard-coded throttle = 0.8 (scripted driver)
- Initial vehicle location: (-15, 0, 0.0)
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation constants ===
time_step = 5e-4
sim_end = 20.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_every = max(1, math.ceil(render_step_size / time_step))

# Vehicle init position
init_loc = chrono.ChVector3d(-15.0, 0.0, 1.0)
init_rot = chrono.QUNIT

# === M113 Tracked Vehicle ===
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

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSprocketVisualizationType(veh.VisualizationType_MESH)
vehicle.SetIdlerVisualizationType(veh.VisualizationType_MESH)
vehicle.SetIdlerWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)

system = vehicle.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

vehicle.GetVehicle().EnableRealtime(True)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === SCM Terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    40, 40, -1, 1, 0.02
)
terrain.SetTexture(
    veh.GetDataFile("terrain/textures/dirt.jpg"),
    6.0, 6.0,
)
terrain.AddMovingPatch(
    vehicle.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)

# === Visualization (Irrlicht — tracked vehicle visual system) ===
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 on SCM Terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.1), 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver (interactive — scored core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording scaffolding ===
import sim_recording as rec  # review-only
REC = bool(os.environ.get("SIMBENCH_RECORD"))  # review-only
irr_dir = rec.frame_dir("frames") if REC else None  # review-only

# CSV writer for review-only physics logging
csv_writer = None  # review-only
csv_f = None  # review-only
if REC:  # >>> review-only >>>
    csv_f = open("simulation_data.csv", "w", newline="")
    csv_writer = __import__('csv').writer(csv_f)
    csv_writer.writerow(["time", "chassis_x", "chassis_z", "speed"])
# <<< review-only <<<

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()

    # Hard-coded throttle = 0.8 (scripted, per input2.txt)
    driver.SetThrottle(0.8)

    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:  # review-only
            vis.WriteImageToFile(rec.frame_path(irr_dir, render_frame))
            render_frame += 1

    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(time_step)
    terrain.Advance(time_step)
    vehicle.Advance(time_step)
    vis.Advance(time_step)

    if REC and csv_writer:  # review-only
        t = system.GetChTime()
        pos = vehicle.GetChassisBody().GetPos()
        vel = vehicle.GetChassisBody().GetLinVel()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        csv_writer.writerow([t, pos.x, pos.z, speed])

    step_number += 1
    realtime_timer.Spin(time_step)

if REC:  # >>> review-only >>>
    if csv_f:
        csv_f.close()
    rec.assemble_all_videos(irr_dir, sensor_dirs=[])
    rec.plot_table("simulation_data.csv", "simulation_timeseries.png")
    rec.cleanup_frames(irr_dir)
# <<< review-only <<<
