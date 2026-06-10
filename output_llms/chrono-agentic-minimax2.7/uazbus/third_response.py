"""
UAZBUS vehicle on flat rigid terrain with RIGID tires, driving forward
with a fixed box obstacle in front.

Changes from base uazbus turn 1:
  - Tire model: RIGID (changed from TMEASY per input3)
  - Added fixed box obstacle at (5, 0, 0.1)
  - Constant throttle = 0.5
"""

import os
import math
import csv

# === System & simulation parameters ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))


# === Chrono & vehicle data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === UAZBUS vehicle setup ===
# Tire model changed to RIGID per input3
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.QUNIT

uazbus = veh.UAZBUS()
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)
uazbus.SetChassisFixed(False)
uazbus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
uazbus.SetTireType(veh.TireModelType_RIGID)  # input3: changed from TMEASY to RIGID
uazbus.SetTireStepSize(time_step)
uazbus.Initialize()

system = uazbus.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", uazbus.GetVehicle().GetMass())

# === Rigid flat terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)
patch.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
terrain.Initialize()

# === Fixed box obstacle (per input3) ===
box_lx, box_ly, box_lz = 0.5, 5.0, 0.2
box_pos = chrono.ChVector3d(5.0, 0.0, 0.1)
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.6)
box_mat.SetRestitution(0.0)
obstacle = chrono.ChBodyEasyBox(box_lx, box_ly, box_lz, 1000.0, True, True, box_mat)
obstacle.SetName("box_obstacle")
obstacle.SetPos(box_pos)
obstacle.SetFixed(True)
system.AddBody(obstacle)

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS - RIGID tire, box obstacle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.5), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(uazbus.GetVehicle())

# === Scripted driver with constant throttle (per input3) ===
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.5, 0.0),   # constant throttle 0.5
    veh.DataDriverEntry(sim_end, 0.0, 0.5, 0.0),
])
driver = veh.ChDataDriver(uazbus.GetVehicle(), driver_data)
driver.Initialize()

# === Simulation loop ===
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:

    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            if REC:
                vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
                frame += 1

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        uazbus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        uazbus.Advance(time_step)
        vis.Advance(time_step)

        # Log vehicle state each step
            chassis_pos = uazbus.GetChassisBody().GetPos()
            veh_speed = uazbus.GetVehicle().GetSpeed()
            data_writer.writerow([
                f"{time:.4f}",
                f"{chassis_pos.x:.4f}",
                f"{chassis_pos.y:.4f}",
                f"{chassis_pos.z:.4f}",
                f"{veh_speed:.4f}",
            ])

        step_number += 1
        realtime_timer.Spin(time_step)

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise

finally:
    if REC:  # review-only block
        csv_file.close()
        rec.assemble_all_videos(irr_dir, sensor_dirs=[])
        rec.plot_table("simulation_data.csv", "simulation_timeseries.png")
        rec.cleanup_frames(irr_dir)
