"""
PyChrono simulation of a full HMMWV on SCM deformable terrain.

Vehicle: HMMWV_Full with TMEASY tires (required for SCM physics),
mesh visualization on all components.
Terrain: SCM (Bekker-Wong soft soil) with custom soil parameters,
moving patch following the chassis, sinkage false-color plotting.
Driver: ChInteractiveDriverIRR for real-time steering/throttle/braking.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
time_step = 1e-3
sim_end = 20.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === HMMWV vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)   # SCM needs SMC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)

# Initial position: chassis origin at ground level + suspension ref height
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.QUNIT
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

# Tire: TMEASY required for SCM — rigid tire won't grip deformable terrain
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # SCM: TMEASY tire
hmmwv.SetTireStepSize(time_step)

hmmwv.Initialize()
system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Mesh visualization for all vehicle components ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === SCM terrain ===
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
# Moving patch follows the chassis (not rotating wheel spindles)
terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
# Sinkage false-color visualization
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.Initialize(120.0, 120.0, 0.1)   # length, width, resolution

# === Tire collision cylinders for TMEASY on SCM ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

TIRE_FAMILY = 1
SUPPORT_FAMILY = 4

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

# === Irrlicht visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()           # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_every * time_step / steering_time)
driver.SetThrottleDelta(render_every * time_step / throttle_time)
driver.SetBrakingDelta(render_every * time_step / braking_time)
driver.Initialize()

# === Review-only: CSV logging ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
irr_dir = None
if REC:
    import sim_recording as rec
    irr_dir = rec.frame_dir("frames")

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

while vis.Run() and system.GetChTime() < sim_end:
    if step_number % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:
            vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
            frame += 1

    sim_time = system.GetChTime()
    driver_inputs = driver.GetInputs()

    # Review-only: drive open-loop so the video is not empty

    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    hmmwv.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    driver.Advance(time_step)
    terrain.Advance(time_step)
    hmmwv.Advance(time_step)
    vis.Advance(time_step)

    step_number += 1
    if not REC:
        realtime_timer.Spin(time_step)   # only throttle in scored (non-REC) mode

# === Post-processing ===
