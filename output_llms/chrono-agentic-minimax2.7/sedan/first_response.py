"""
Sedan (BMW E90) on rigid terrain with TMEASY tires and interactive driver.

Simulates a BMW E90 sedan driving on a flat rigid terrain surface using
the TMEASY tire model.  An interactive driver provides real-time steering,
throttle, and braking control via the Irrlicht window.
"""

import os
import math
import glob
import csv as csv_module
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
time_step = 1e-3
sim_end = 30.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Create sedan (BMW E90) ===
sedan = veh.BMW_E90()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)

# TMEASY tire model (prompt: TMEASY tire model)
sedan.SetTireType(veh.TireModelType_TMEASY)
sedan.SetTireStepSize(time_step)

init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT
sedan.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
sedan.Initialize()

# Set visualization types to MESH (BMW E90 meshes are available in the data path)
sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
sedan.SetSteeringVisualizationType(veh.VisualizationType_MESH)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

system = sedan.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", sedan.GetVehicle().GetMass())

# === Terrain (rigid flat) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain_length = 200.0
terrain_width = 200.0
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.7, 0.7, 0.7))
terrain.Initialize()

# === Visualization (Irrlicht, vehicle-aware) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 Sedan - Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()  # vehicle truths use directional light, not AddTypicalLights()
vis.AttachVehicle(sedan.GetVehicle())

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_every * time_step / steering_time)
driver.SetThrottleDelta(render_every * time_step / throttle_time)
driver.SetBrakingDelta(render_every * time_step / braking_time)
driver.Initialize()

# === Review-only setup ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
if REC:
    os.makedirs("frames", exist_ok=True)
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

# review-only CSV writer (scoped outside loop so data_writer is always visible when REC=True)
_data_writer = None
if REC:
    _csv_f = open("simulation_data.csv", "w", newline="")
    _data_writer = csv_module.DictWriter(
        _csv_f,
        fieldnames=[
            "time",
            "speed",
            "steering",
            "throttle",
            "braking",
            "pos_x",
            "pos_y",
            "pos_z",
        ],
    )
    _data_writer.writeheader()

try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        if REC:
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")
            frame += 1

        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            # review-only: scripted open-loop driving so the RUN video moves
            if REC:
                if sim_time < 0.5:
                    driver_inputs.m_throttle = 0.0
                    driver_inputs.m_braking = 1.0
                    driver_inputs.m_steering = 0.0
                elif sim_time < 2.0:
                    driver_inputs.m_throttle = 0.5
                    driver_inputs.m_braking = 0.0
                    driver_inputs.m_steering = 0.0
                else:
                    driver_inputs.m_throttle = 0.3
                    driver_inputs.m_braking = 0.0
                    driver_inputs.m_steering = 0.0

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            sedan.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(time_step)
            terrain.Advance(time_step)
            sedan.Advance(time_step)
            vis.Advance(time_step)

            if REC:
                chassis = sedan.GetChassisBody()
                speed = sedan.GetVehicle().GetSpeed()
                _data_writer.writerow(
                    {
                        "time": sim_time,
                        "speed": speed,
                        "steering": driver_inputs.m_steering,
                        "throttle": driver_inputs.m_throttle,
                        "braking": driver_inputs.m_braking,
                        "pos_x": chassis.GetPos().x,
                        "pos_y": chassis.GetPos().y,
                        "pos_z": chassis.GetPos().z,
                    }
                )

            step_number += 1
            realtime_timer.Spin(time_step)

            if system.GetChTime() >= sim_end:
                break

finally:
    if REC and _csv_f:
        _csv_f.close()

# === Post-processing (review-only) ===
