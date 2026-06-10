import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

# simulation parameters
step_size = 3e-3                                                       # 3 ms physics step
sim_end = 20.0                                                         # 20 s simulation
render_fps = 50.0                                                      # 50 fps rendering

# HMMWV initial pose — spawn above flat SCM terrain (z=0)
# spindle is ~0.026 m below chassis center; tire radius ~0.467 m → chassis_z ≈ 0.49 m
init_loc = chrono.ChVector3d(-5, 0, 0.5)                               # chassis origin ~0.5 m above ground
init_rot = chrono.QuatFromAngleZ(0)                                    # heading along +X

hmmwv = veh.HMMWV_Full()                                               # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                     # SMC required for SCM terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                  # no chassis mesh collision
hmmwv.SetChassisFixed(False)                                           # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_RIGID)                             # prompt: rigid tire model
hmmwv.SetTireStepSize(step_size)                                       # tire substep matches sim step
hmmwv.Initialize()

system = hmmwv.GetSystem()                                             # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # REQUIRED before SCM

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                  # diagnostic mass banner

# visualization types — mesh for all components per prompt
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# SCM deformable terrain
terrain = veh.SCMTerrain(system)                                       # Bekker-Wong soft-soil model
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear deformation coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)            # false color sinkage visualization
terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),                                            # attach to chassis (stable OOBB)
    chrono.ChVector3d(0, 0, 0),                                        # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),                                        # OOBB dimensions (m)
)
terrain.Initialize(300.0, 300.0, 0.1)                                  # 300×300 m patch, 0.1 m grid resolution
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    200, 200,                                                          # UV tiling scaled for larger terrain
)

# tire collision cylinders — REQUIRED for RIGID tires on SCM
TIRE_FAMILY = 1
SUPPORT_FAMILY = 4

for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        tire_rad = axle.m_wheels[iw].GetTire().GetRadius()
        tire_w   = axle.m_wheels[iw].GetTire().GetWidth()
        tire_mat = chrono.ChContactMaterialSMC()
        tire_mat.SetFriction(0.9)
        tire_mat.SetRestitution(0.1)
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

system.GetCollisionSystem().BindAll()                                  # rebuild collision models

# vehicle Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)           # chase cam behind chassis
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                              # vehicle truth uses directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# interactive driver — keyboard control
render_step_size = 1.0 / render_fps                                    # 0.02 s per frame
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                                                    # seconds to full steering
throttle_time = 1.0                                                    # seconds to full throttle
braking_time = 0.3                                                     # seconds to full brake
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

render_every = max(1, round(render_step_size / step_size))             # physics steps per render frame
realtime_timer = chrono.ChRealtimeStepTimer()                          # keep wall-clock ≈ sim time
step_number = 0                                                        # step counter for render cadence


while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
    sim_time = hmmwv.GetSystem().GetChTime()

    if step_number % render_every == 0:                                # throttled rendering every render_every steps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    hmmwv.Synchronize(sim_time, driver_inputs, terrain)                # 3-arg wheeled vehicle
    vis.Synchronize(sim_time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)                                           # advances wrapper-owned ChSystem
    vis.Advance(step_size)


    step_number += 1
    realtime_timer.Spin(step_size)                                     # real-time pacing
