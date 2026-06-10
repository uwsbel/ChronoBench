"""
FEDA Vehicle Simulation — Turn 3
Adds a grass-textured terrain, sensor manager with point lights,
and a first-person camera sensor on the vehicle chassis.

plan_type: mbs_in_scene (wheeled vehicle + terrain + sensor)
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Path setup for FEDA catalog vehicle ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
time_step = 1e-3
sim_end = 30.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Create FEDA vehicle ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(time_step)
feda.Initialize()
system = feda.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# === Rigid terrain with grass texture ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrainLength = 200.0
terrainWidth = 200.0
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrainLength,
    terrainWidth,
)
# Grass texture — use the available terrain texture
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 80, 80)
patch.SetColor(chrono.ChColor(0.4, 0.6, 0.2))
terrain.Initialize()

# === Sensor Manager and point lights ===
manager = sens.ChSensorManager(system)
# Add point lights for well-illuminated scene
manager.scene.AddPointLight(
    chrono.ChVector3f(10, 10, 50),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-10, -10, 50),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 60),
    chrono.ChColor(0.9, 0.9, 0.9),
    800.0,
)

# === Camera sensor — First Person View on chassis ===
# offset: in front of and above the chassis, facing forward
chassis_body = feda.GetChassisBody()
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-1.0, 0.0, 1.5),  # forward of chassis origin, elevated
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
fpv_cam = sens.ChCameraSensor(
    chassis_body,
    30,  # update_rate Hz (physical rate)
    offset_pose,
    1920, 1080,  # high resolution
    1.408,  # horizontal FOV (rad) — ~80 degrees
)
fpv_cam.SetName("FPV Camera Sensor")
fpv_cam.SetLag(0)
fpv_cam.SetCollectionWindow(0)
# Filter chain: visualize + save
fpv_cam.PushFilter(sens.ChFilterVisualize(1920, 1080, "FPV Camera"))
fpv_cam.PushFilter(sens.ChFilterRGBA8Access())
fpv_cam.PushFilter(sens.ChFilterSave("cam/fpv/"))
manager.AddSensor(fpv_cam)

# === Interactive driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA — Turn 3: FPV Camera + Grass Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(feda.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(time_step / steering_time)
driver.SetThrottleDelta(time_step / throttle_time)
driver.SetBrakingDelta(time_step / braking_time)
driver.Initialize()

# === Main simulation loop ===

# CSV logging setup — only open when recording

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()

    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        feda.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)
        manager.Update()
        driver.Advance(time_step)
        terrain.Advance(time_step)
        feda.Advance(time_step)
        vis.Advance(time_step)
        if system.GetChTime() >= sim_end:
            break

    step_number += 1
    realtime_timer.Spin(time_step)
