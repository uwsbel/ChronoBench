"""HMMWV on an SCM deformable hill (heightmap-initialized soft soil).

System type: NSC vehicle wrapper system (HMMWV_Full owns its ChSystemNSC),
deformable terrain via veh.SCMTerrain initialized from a shipped bump heightmap.
Main bodies: the HMMWV chassis + four wheels/spindles (TMEASY tires for SCM),
and the deformable SCM terrain grid. The vehicle is driven interactively
(ChInteractiveDriverIRR) in a real-time Irrlicht window; wheels sink into the
soft soil and leave ruts as the vehicle traverses the bump terrain.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr   # noqa: F401  (Irrlicht visual module)

# === Parameters === geometry / physics constants (no bare literals downstream)
step_size = 2e-3                 # integration time step (s)
tire_step_size = 1e-3            # TMEASY tire substep (required on SCM)
sim_end = 12.0                   # bounded recording horizon (s)
render_fps = 50.0                # review render cadence

# SCM heightmap terrain extents (m) — bump64 hill, mapped to a small height range
SCM_LENGTH = 40.0
SCM_WIDTH = 40.0
SCM_HMIN = -1.0
SCM_HMAX = 1.0
SCM_RES = 0.02                   # terrain grid resolution (m)

# Vehicle spawn — above the terrain so the wheels settle onto the hill surface
INIT_X = -8.0
INIT_Y = 0.0
INIT_Z = 1.2                     # chassis origin height above the bump crest at spawn

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === HMMWV_Full wrapper owns its ChSystemNSC; TMEASY tires for SCM
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QUNIT

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)       # SCM/deformable terrain uses SMC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                             # fixed chassis would never move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)              # SCM needs a non-rigid tire model
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                               # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()                         # main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxles()[i].m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())   # report total vehicle mass

# === Terrain === SCM Bekker-Wong deformable soil, initialized from a bump heightmap
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa.s/m)
)
# Moving patch on the CHASSIS (level body) so the active SCM window tracks the car
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # sinkage heatmap overlay
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),
                   SCM_LENGTH, SCM_WIDTH, SCM_HMIN, SCM_HMAX, SCM_RES)
terrain.SetMeshWireframe(False)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# === Tire collision cylinders === TMEASY tires need explicit collision geometry for SCM
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)
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
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)        # tires must not self-collide
system.GetCollisionSystem().BindAll()                    # rebuild models so rays see cylinders

# === Visualization === vehicle-aware Irrlicht window: sky + camera + lights + terrain
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Hill")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive driver bound to the visual system (truth-faithful)
render_step_size = 1.0 / render_fps
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop === real-time Synchronize/Advance of driver, terrain, vehicle, vis
render_steps = math.ceil(render_step_size / step_size)   # precomputed once

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:              # throttled rendering
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
        hmmwv.Advance(step_size)                         # advances the wrapper-owned system
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)                   # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:                # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
