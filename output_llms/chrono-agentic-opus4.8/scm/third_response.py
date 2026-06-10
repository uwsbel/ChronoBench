"""HMMWV on Bekker-Wong (SCM) deformable terrain with scattered obstacles and an
onboard camera sensor.

System type: NSC vehicle system (owned by the veh.HMMWV_Full wrapper); SMC contact
method is used for the deformable SCM soil. The main bodies are the HMMWV chassis
and four wheel spindles riding on the SCM grid, plus a set of randomly positioned
ChBodyEasyBox obstacles scattered across the terrain (kept clear of the vehicle
spawn footprint so the vehicle is not initialized inside a box). A ChSensorManager
with point lights drives a ChCameraSensor mounted on the chassis; a ChFilterVisualize
streams the live camera feed during the run.

Expected behavior: the HMMWV (TMEASY tires) drives forward across the soft soil,
leaving ruts, while the onboard camera renders the scene ahead of the chassis.
"""

import math
import random

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Simulation constants ===
time_step = 2e-3                 # SCM soft-soil step size
sim_end = 10.0                   # total simulated seconds
tire_step_size = 1e-3            # TMEASY tire integration step

init_loc = chrono.ChVector3d(-5.0, 0.0, 0.6)   # chassis spawn (origin = geometric center)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Terrain extent (square patch centered at origin) and grid resolution.
terrain_size = 40.0
terrain_res = 0.1

# Random obstacle layout.
NUM_BOXES = 8
BOX_SIZE = 0.4                   # cube full edge length
BOX_DENSITY = 50.0
VEHICLE_CLEAR_R = 4.0            # keep boxes this far from the vehicle spawn XY


# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# === Vehicle (HMMWV_Full wrapper owns the ChSystem + chassis/spindles/joints) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)           # SMC for deformable SCM soil
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                 # MANDATORY — a fixed chassis never moves
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                  # SCM needs a non-rigid tire (RIGID won't drive)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                   # ChSystemSMC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain
chassis_body = hmmwv.GetChassisBody()                        # cache: main chassis rigid body, reused below
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === SCM deformable terrain (Bekker-Wong soft soil) ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi
    0,      # Bekker_Kc
    1.1,    # Bekker_n
    0,      # Mohr_cohesion
    30,     # Mohr_friction (deg)
    0.01,   # Janosi_shear
    2e8,    # elastic_K
    3e4,    # damping_R
)
terrain.AddMovingPatch(chassis_body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)
terrain.Initialize(terrain_size, terrain_size, terrain_res)

# === Tire collision cylinders (TMEASY needs explicit spindle collision for SCM) ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

TIRE_FAMILY = 1
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
system.GetCollisionSystem().BindAll()

# === Random obstacle boxes (kept clear of the vehicle spawn footprint) ===
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)

rng = random.Random(7)           # deterministic layout
half_span = terrain_size / 2 - 2.0
boxes = []
while len(boxes) < NUM_BOXES:
    bx = rng.uniform(-half_span, half_span)
    by = rng.uniform(-half_span, half_span)
    # reject any draw that lands within the vehicle clearance radius
    if math.hypot(bx - init_loc.x, by - init_loc.y) < VEHICLE_CLEAR_R:
        continue
    box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(bx, by, BOX_SIZE / 2))
    box.SetName("obstacle_box_%d" % len(boxes))
    system.AddBody(box)
    boxes.append(box)

# === Sensor system: manager + point lights + chassis camera ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-10, -10, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 50), chrono.ChColor(1, 1, 1), 500.0)

cam_offset = chrono.ChFramed(
    chrono.ChVector3d(1.5, 0, 1.2),                              # forward & above the chassis origin
    chrono.QuatFromAngleAxis(0.05, chrono.ChVector3d(0, 1, 0)),  # slight downward tilt
)
cam = sens.ChCameraSensor(chassis_body, 30, cam_offset, 1280, 720, 1.408)
cam.SetName("Chassis Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Chassis Camera"))   # live camera feed during the run
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/chassis_cam/"))                 # camera image stream -> mp4
manager.AddSensor(cam)

# === Driver (interactive, real-time — matches the catalog-vehicle truth shape) ===
render_step_size = 1.0 / 50.0
render_steps = math.ceil(render_step_size / time_step)

# === Visualization (vehicle-aware Irrlicht: window + camera + sky + light) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM terrain with obstacles and onboard camera")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop (real-time: Synchronize/Advance the full subsystem stack) ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run():
        time = system.GetChTime()
        if time >= sim_end:
            break

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)

        manager.Update()
        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
