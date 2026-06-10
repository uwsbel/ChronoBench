"""
PyChrono 9.0 simulation: Gator vehicle with rigid terrain, interactive driver,
sensor manager, point lights, and a chassis-mounted camera.

Vehicle: Gator (veh.Gator wrapper)
Terrain: RigidTerrain (flat NSC patch)
Driver: ChInteractiveDriverIRR (interactive, keyboard)
Sensors: ChSensorManager + ChCameraSensor attached to chassis with point lights
"""

import os
import math

# === PyChrono imports ===
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Create Gator vehicle ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetTireType(veh.TireModelType_RIGID)
gator.Initialize()
system = gator.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === Rigid terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    200.0, 200.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Sensor manager + point lights ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-20, 2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)

# === Chassis camera sensor ===
# Attach to the real chassis body
chassis_body = gator.GetChassisBody()
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-3.0, 0.0, 1.0),  # behind and above chassis
    chrono.QuatFromAngleAxis(0.1, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis_body,
    30,  # physical update rate Hz
    offset_pose,
    1280, 720,
    1.408,
)
cam.SetName("Chassis Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Chassis Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/chassis_cam/"))
manager.AddSensor(cam)

# === Vehicle part visualization types (set BEFORE AttachVehicle) ===
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Irrlicht visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle - Sensor Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(gator.GetVehicle())

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_every * time_step / steering_time)
driver.SetThrottleDelta(render_every * time_step / throttle_time)
driver.SetBrakingDelta(render_every * time_step / braking_time)
driver.Initialize()

# === Review-only: CSV logging + frame capture ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))

if REC:
    os.makedirs("cam", exist_ok=True)
    import sim_recording as rec
    irr_dir = rec.frame_dir("frames")
    csv_path = "simulation_data.csv"
    csv_file = open(csv_path, "w", newline="")
    import csv as csv_mod
    csv_writer = csv_mod.DictWriter(
        csv_file,
        fieldnames=["time", "speed", "steering", "throttle", "braking"],
    )
    csv_writer.writeheader()

# === Main simulation loop ===
frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

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
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Sensor update every physics step
    manager.Update()

    driver.Advance(time_step)
    terrain.Advance(time_step)
    gator.Advance(time_step)
    vis.Advance(time_step)

    if REC:
        csv_writer.writerow({
            "time": time,
            "speed": gator.GetVehicle().GetSpeed(),
            "steering": driver_inputs.m_steering,
            "throttle": driver_inputs.m_throttle,
            "braking": driver_inputs.m_braking,
        })

    step_number += 1
    realtime_timer.Spin(time_step)

    if system.GetChTime() >= sim_end:
        break

# === Cleanup ===
if REC:
    csv_file.close()
    rec.assemble_all_videos(irr_dir, sensor_dirs=["cam/chassis_cam"])
    rec.cleanup_frames(irr_dir, "cam/chassis_cam")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if os.path.exists(csv_path):
        data = list(csv_mod.DictReader(open(csv_path)))
        if data:
            times = [float(r["time"]) for r in data]
            speeds = [float(r["speed"]) for r in data]
            plt.figure(figsize=(8, 4))
            plt.plot(times, speeds, label="Speed (m/s)")
            plt.xlabel("Time (s)")
            plt.ylabel("Speed (m/s)")
            plt.title("Gator Vehicle Speed vs Time")
            plt.legend()
            plt.savefig("simulation_timeseries.png", dpi=100)
            plt.close()
