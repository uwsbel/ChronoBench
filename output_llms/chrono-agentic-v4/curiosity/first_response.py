"""
Curiosity Mars Rover simulation on rigid terrain.

System type: NSC (ChSystemNSC) with Bullet collision.
A Curiosity rover navigates on a fixed rigid ground plane.
The rover uses a DC-motor steering driver for real-time steering inputs.
Visualized with Irrlicht: sky box, directional lights, shadows, grid, and logo.
"""

import os
import math
import csv as csvmod
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

import sim_recording as rec

# === System & gravity (NSC, Bullet collision) ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground — fixed rigid terrain ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 1.0, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))   # top surface at z=0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Curiosity rover — built-in catalog model ===
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.QUNIT
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Cache frequently-accessed handles
chassis_body = rover.GetChassis().GetBody()   # cache: chassis attach point

# === Visualization — full Irrlicht scene ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - Rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 2), chassis_body.GetPos())
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Simulation parameters ===
time_step = 1e-3
sim_end = 20.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))
max_steering = math.pi / 6

REC = bool(os.environ.get("SIMBENCH_RECORD"))

# === CSV setup ===
if REC:
    _csv_file = open("simulation_data.csv", "w", newline="")
    _csv_writer = csvmod.DictWriter(_csv_file, fieldnames=["time", "chassis_x", "chassis_y", "chassis_z", "steering"])
    _csv_writer.writeheader()

# === Main loop ===
frame = 0
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    if REC:
        vis.WriteImageToFile(f"frames/img_{frame:06d}.png"); frame += 1

    for _ in range(render_every):
        sim_time = system.GetChTime()

        # Ramp steering: straight, then a turn, then back
        steering = 0.0
        if 3.0 < sim_time < 8.0:
            steering = max_steering * (sim_time - 3.0) / 5.0
        elif 8.0 < sim_time < 13.0:
            steering = max_steering * (13.0 - sim_time) / 5.0
        driver.SetSteering(steering)
        rover.Update()

        if REC:
            pos = chassis_body.GetPos(); _csv_writer.writerow({"time": f"{sim_time:.4f}", "chassis_x": f"{pos.x:.4f}", "chassis_y": f"{pos.y:.4f}", "chassis_z": f"{pos.z:.4f}", "steering": f"{steering:.4f}"})

        system.DoStepDynamics(time_step)
        if system.GetChTime() >= sim_end:
            break

if REC:
    _csv_file.close()
    rec.assemble_all_videos("frames", sensor_dirs=[])
    rec.plot_table("simulation_data.csv", "simulation_timeseries.png")
    rec.cleanup_frames("frames")
