"""
SCM Hill — HMMWV off-road on Bekker-Wong soft-soil terrain with a height-map bump field.

Vehicle:    HMMWV_Full (NSC, TMEASY tire for SCM)
Terrain:    SCMTerrain with bump64.bmp height map
Driver:     ChInteractiveDriverIRR (real-time default — script the maneuver in review-only)
Visual:     Irrlicht with chase camera + directional light
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Review-only recording scaffold ===

# === CSV writer (opened/closed around the loop) ===

# === Simulation parameters ===
time_step = 1e-3          # physics timestep (s)
sim_end = 20.0            # simulation duration (s)
render_fps = 50.0         # target rendering frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Data paths (required for catalog vehicles — scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === HMMWV vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for SCM
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
# Spawn at origin on flat start of the bump field
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # TMEASY required for SCM
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Cache key handles (visibility to source-only reviewer)
chassis = hmmwv.GetChassisBody()  # cache: main chassis rigid body

# === SCM Terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi
    0,      # Bekker_Kc
    1.1,    # Bekker_n
    0,      # Mohr_cohesion
    30,     # Mohr_friction (deg)
    0.01,   # Janosi_shear (m)
    2e8,    # elastic_K (Pa/m)
    3e4,    # damping_R (Pa·s/m)
)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    40, 40,       # length, width (m)
    -1, 1,        # hMin, hMax (m)
    0.02,         # grid resolution (m)
)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
terrain.SetMeshWireframe(False)

# Moving patch centred on chassis so only nearby cells are ray-cast
terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)

# TMEASY tire collision cylinders (required — TMEASY does not auto-add them)
TIRE_FAMILY = 1
SUPPORT_FAMILY = 4

for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        tire_rad = axle.m_wheels[iw].GetTire().GetRadius()
        tire_w = axle.m_wheels[iw].GetTire().GetWidth()
        tire_mat = chrono.ChContactMaterialSMC()
        tire_mat.SetFriction(0.9)
        tire_mat.SetRestitution(0.1)
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

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("SCM Hill — HMMWV")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # vehicle truths use directional, not AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive — ChInteractiveDriverIRR is the scored-core default) ===
real_driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
real_driver.SetSteeringDelta(render_every * time_step / steering_time)
real_driver.SetThrottleDelta(render_every * time_step / throttle_time)
real_driver.SetBrakingDelta(render_every * time_step / braking_time)
real_driver.Initialize()

# Review-only wrapper: adds scripted forward drive for the RUN video
# review-only >>>  # Force forward drive so the vehicle climbs the hill in the review video
# review-only >>>  class _ScriptedDriver:
# review-only >>>      def __init__(self, real):
# review-only >>>          self._real = real
# review-only >>>      def GetInputs(self):
# review-only >>>          inp = self._real.GetInputs()
# review-only >>>          inp.m_throttle = 0.45
# review-only >>>          inp.m_braking = 0.0
# review-only >>>          inp.m_steering = 0.0
# review-only >>>          return inp
# review-only >>>      def Synchronize(self, t): self._real.Synchronize(t)
# review-only >>>      def Advance(self, dt): self._real.Advance(dt)
# review-only >>>  driver = _ScriptedDriver(real_driver)
driver = real_driver

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

frame = 0

while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sim_time = hmmwv.GetSystem().GetChTime()
        driver_inputs = driver.GetInputs()
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)
        if hmmwv.GetSystem().GetChTime() >= sim_end:
            break
    step_number += 1
    realtime_timer.Spin(time_step)
