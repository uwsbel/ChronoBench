import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# simulation parameters
step_size = 5e-4                    # physics timestep (s) — smaller for SCM stability
sim_end = 20.0                      # simulation end time (s)
render_fps = 50.0                   # render frame rate
render_every = max(1, round(1.0 / (render_fps * step_size)))  # render cadence (untagged)

# initial vehicle position and orientation
initLoc = chrono.ChVector3d(0, 0, 1.5)           # spawn high enough to clear bumps (Z up)
initRot = chrono.QuatFromAngleZ(0)                # facing forward (no rotation)

# HMMWV on SCM deformable terrain — use SMC contact method and TMEASY tires
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)            # SMC for SCM/deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)         # chassis collision off
hmmwv.SetChassisFixed(False)                                  # MANDATORY — chassis must move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                   # TMEASY required for SCM — RIGID won't drive
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# acquire the system after wrapper initializes
system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                # vehicle mass banner (scored)

# set visualization types
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# SCM deformable terrain initialized from bump heightmap
terrain = veh.SCMTerrain(system)                              # soft deformable terrain
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # sinkage heatmap overlay

# moving patch tracks vehicle so only cells near it are updated
terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),                  # must use chassis, not spindles
    chrono.ChVector3d(0, 0, 0),              # local OOBB center offset
    chrono.ChVector3d(5, 3, 1),              # OOBB dimensions (m)
)

# initialize terrain from bump heightmap — 200×200 m large patch, height range [-1, 1] m, 0.05 m resolution
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),
                   200, 200, -1, 1, 0.05)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 30.0, 30.0)

# tire collision cylinders required for TMEASY on SCM (non-rigid tires have no auto collision)
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
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),  # +0.04 for guaranteed sinkage
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)  # tires don't collide with each other
        # NOTE: do NOT DisallowCollisionsWith(0) — breaks SCM ray-casts

# rebuild collision models after post-init shape changes
system.GetCollisionSystem().BindAll()

# Irrlicht vehicle visual system — Initialize FIRST, then add scene elements AFTER
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Hill")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)  # chase camera above chassis
vis.Initialize()                                               # MUST be first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                      # directional light (vehicle truth style)
vis.AttachVehicle(hmmwv.GetVehicle())

# interactive driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_every * step_size / 1.0)       # steering response
driver.SetThrottleDelta(render_every * step_size / 1.0)       # throttle response
driver.SetBrakingDelta(render_every * step_size / 0.3)        # braking response
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()                  # real-time pacing
step_number = 0


while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
    time = hmmwv.GetSystem().GetChTime()

    if step_number % render_every == 0:                        # throttled rendering
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

    step_number += 1
    realtime_timer.Spin(step_size)                             # maintain real-time pacing
