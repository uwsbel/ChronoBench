"""
PyChrono simulation featuring a UAZBUS vehicle on rigid terrain.

System type: NSC (Non-Smooth Contact) for rigid terrain contact.
Main bodies: UAZBUS wheeled vehicle, rigid ground patch.
Expected behavior: vehicle drives forward with scripted throttle input.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
time_step = 1e-3          # physics timestep [s]
sim_end = 10.0            # total simulation duration [s]
render_fps = 50.0         # render frame rate [fps]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === UAZBUS vehicle ===
bus = veh.UAZBUS()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.7), chrono.QUNIT))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(time_step)
bus.Initialize()

# Set visualization types AFTER Initialize
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)

system = bus.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# === Terrain (RigidTerrain) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.0)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    200.0,   # terrain length [m]
    200.0,   # terrain width [m]
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(-1.2, 0, 0.3), 8.0, 0.5)  # chase: trackPoint near vehicle COM
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()  # vehicle truths use directional light, not AddTypicalLights()
vis.AttachVehicle(bus.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0    # seconds to reach max steering
throttle_time = 1.0    # seconds to reach max throttle
braking_time = 0.3     # seconds to reach max braking
driver.SetSteeringDelta(render_every * time_step / steering_time)
driver.SetThrottleDelta(render_every * time_step / throttle_time)
driver.SetBrakingDelta(render_every * time_step / braking_time)
driver.Initialize()

# === Real-time step timer ===
realtime_timer = chrono.ChRealtimeStepTimer()

# === Review-only: CSV / frame capture / recording ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
if REC:
    import sim_recording as rec
    irr_dir = rec.frame_dir("frames")
    os.makedirs("frames", exist_ok=True)
    frame = 0
    csv_file = open("simulation_data.csv", "w", newline="")
    import csv
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["time", "speed", "pos_x", "pos_y", "pos_z", "steering", "throttle", "braking"])

# === Main simulation loop ===
step_number = 0
while vis.Run() and system.GetChTime() < sim_end:
    # --- throttled rendering ---
    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")
            frame += 1

    # --- scripted driving: keep throttle at 0.5 for the scripted demo ---
    driver.SetThrottle(0.5)   # uazbus truth scripts throttle at 0.5
    driver.SetSteering(0.0)
    driver.SetBraking(0.0)

    sim_time = system.GetChTime()
    driver_inputs = driver.GetInputs()

    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    bus.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    driver.Advance(time_step)
    terrain.Advance(time_step)
    bus.Advance(time_step)
    vis.Advance(time_step)

    if REC:
        chassis = bus.GetChassisBody()
        csv_writer.writerow([
            sim_time,
            bus.GetVehicle().GetSpeed(),
            chassis.GetPos().x,
            chassis.GetPos().y,
            chassis.GetPos().z,
            driver_inputs.m_steering,
            driver_inputs.m_throttle,
            driver_inputs.m_braking,
        ])

    step_number += 1
    realtime_timer.Spin(time_step)
