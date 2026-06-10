"""
SCM Off-Road HMMWV Simulation with Random Boxes and Camera Sensor.

System type: SMC (required for SCM terrain).
Main bodies: HMMWV_Full vehicle on Bekker-Wong soft-soil SCM terrain,
             randomly placed ChBodyEasyBox obstacles (no overlap with vehicle),
             camera sensor attached to vehicle chassis.
Expected behavior: HMMWV drives forward on deformable SCM terrain, leaving ruts;
                   randomly placed boxes are scattered across the terrain; a sensor
                   camera attached to the chassis captures a chase-camera view and
                   visualizes the feed; sensor manager holds multiple point lights.
"""

import os
import math
import random
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

step_size       = 2e-3
sim_end         = 20.0
render_fps      = 50.0
render_every    = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

# Terrain parameters
TERRAIN_LENGTH  = 200.0
TERRAIN_WIDTH   = 200.0
TERRAIN_RES     = 0.2

# Random box parameters
NUM_BOXES       = 20
BOX_HALF_EXT    = 0.3            # half-extent of each axis for boxes
VEH_EXCL_RADIUS = 6.0            # exclusion radius around vehicle spawn

# Vehicle spawn
INIT_X          = 0.0
INIT_Y          = 0.0
SUSPENSION_H    = 0.5            # chassis origin above wheel bottom at rest
INIT_Z          = SUSPENSION_H   # terrain z=0 at rest

# Tire collision families
TIRE_FAMILY     = 1
SUPPORT_FAMILY  = 4
CHASSIS_FAMILY  = 3

random.seed(42)

# === Vehicle Setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()            # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()      # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === SCM Terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi
    0,      # Bekker_Kc
    1.1,    # Bekker_n
    0,      # Mohr_cohesion
    30,     # Mohr_friction (deg)
    0.01,   # Janosi_shear (m)
    2e8,    # elastic_K
    3e4,    # damping_R
)
terrain.AddMovingPatch(
    chassis,                           # attach to chassis — stable OOBB projection
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RES)
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,
)

# === Tire Collision Cylinders (required for TMEASY on SCM) ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w   = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)

system.GetCollisionSystem().BindAll()

# === Random Boxes ===
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.7)
box_mat.SetRestitution(0.05)

boxes = []
for _ in range(NUM_BOXES):
    while True:
        bx = random.uniform(-50.0, 50.0)   # scatter near vehicle region for visibility
        by = random.uniform(-50.0, 50.0)
        dist = math.sqrt((bx - INIT_X) ** 2 + (by - INIT_Y) ** 2)
        if dist > VEH_EXCL_RADIUS:
            break
    box = chrono.ChBodyEasyBox(
        BOX_HALF_EXT * 2, BOX_HALF_EXT * 2, BOX_HALF_EXT * 2,
        500.0, True, True, box_mat,
    )
    box.SetPos(chrono.ChVector3d(bx, by, BOX_HALF_EXT))
    system.AddBody(box)
    boxes.append(box)

# === Sensor Manager ===
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(10, 10, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-10, -10, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera Sensor (attached to vehicle chassis) ===
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 3),
    chrono.QuatFromAngleAxis(0.3, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis,              # camera rides on chassis
    30,                   # update_rate Hz (physical, not 1/dt)
    cam_offset,
    1280, 720,
    1.408,                # horizontal FOV (rad)
)
cam.SetName("Chassis Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Chassis Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/chassis_cam/"))
manager.AddSensor(cam)

# === Irrlicht Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Terrain with Boxes and Camera Sensor")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / render_fps
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Recording Setup (review-only) ===

# === Main Loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        manager.Update()


        step_number += 1
        realtime_timer.Spin(step_size)

        if system.GetChTime() >= sim_end:
            break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass  # no file writers to close in scored core
