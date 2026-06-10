"""
HMMWV on SCM deformable terrain with randomly positioned boxes and a chassis-mounted camera sensor.

Objectives (from SimBench demo=scm turn=3):
  1. Add randomly positioned boxes to the scene (ChBodyEasyBox objects, not overlapping the vehicle).
  2. Integrate a sensor manager with point lights at various scene positions.
  3. Attach a camera sensor to the vehicle chassis and configure its settings.
  4. Include a filter to visualize the camera feed during the simulation.

Physics: SMC contact, SCM soft-soil terrain, TMEASY tire model, HMMWV_Full vehicle.
"""

import os
import math
import random
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Named constants ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

init_loc = chrono.ChVector3d(-8, 0, 0.6)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
step_size = 1e-3
tire_step_size = step_size
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_every = max(1, round(render_step_size / step_size))
sim_end = 20.0

# === Vehicle (wrapper owns its own ChSystemSMC internally) ===
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

sys = vehicle.GetSystem()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Randomly positioned boxes (ensuring no overlap with vehicle spawn area) ===
random.seed(42)
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)

VEH_X_MIN, VEH_X_MAX = -10, 2
VEH_Y_MIN, VEH_Y_MAX = -2, 2

for i in range(5):
    while True:
        bx = random.uniform(-5, 12)
        by = random.uniform(-10, -2)
        bz = 1.0
        if not (VEH_X_MIN <= bx <= VEH_X_MAX and VEH_Y_MIN <= by <= VEH_Y_MAX):
            break
    box = chrono.ChBodyEasyBox(2, 2, 2, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(bx, by, bz))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    vehicle.GetSystem().AddBody(box)

# === SCM terrain ===
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4
)
terrain.AddMovingPatch(
    vehicle.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)

# === Sensor manager + point lights ===
manager = sens.ChSensorManager(vehicle.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0
)
manager.scene.AddPointLight(
    chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0
)
manager.scene.AddPointLight(
    chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0
)
manager.scene.AddPointLight(
    chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0
)

# === Camera sensor attached to chassis ===
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.7, 0, 1.2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    vehicle.GetChassisBody(),
    30,
    cam_offset_pose,
    1280, 720,
    1.047,
)
cam.SetName("First Person POV")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Before Grayscale Filter"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.71), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0


while vis.Run() and sys.GetChTime() < sim_end:
    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


    manager.Update()
    driver_inputs = driver.GetInputs()
    driver.Synchronize(sys.GetChTime())
    terrain.Synchronize(sys.GetChTime())
    vehicle.Synchronize(sys.GetChTime(), driver_inputs, terrain)
    vis.Synchronize(sys.GetChTime(), driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)
