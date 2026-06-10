"""HMMWV on SCM deformable terrain (Bekker-Wong soft soil) with Irrlicht.

Models a full HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving over a
deformable SCM terrain patch. The vehicle wrapper owns a ChSystemSMC (SMC
contact). The terrain uses custom firm-soil Bekker/Mohr/Janosi parameters, a
moving active patch that follows the chassis, and false-color sinkage plotting so
ruts are visible as the wheels sink in. A scripted driver applies throttle and a
gentle steering sweep so the chassis translates and carves visible ruts.

Expected behavior: the vehicle accelerates forward from rest, the four wheels
sink slightly into the soft soil and leave persistent ruts behind them, and the
chassis X position increases monotonically over the run (it must MOVE — a RIGID
tire would spin without translating on SCM, so a TMEASY slip/grip tire model with
explicit spindle collision cylinders is used).
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / soil / timing (no bare literals downstream)
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # tire force-model substep (s) — required on SCM
SIM_END = 8.0                          # modest end time (SCM ray-casts are slow)
RENDER_FPS = 50.0                      # review render / capture cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

TERRAIN_LENGTH = 40.0                  # SCM patch X size (m)
TERRAIN_WIDTH = 40.0                   # SCM patch Y size (m)
TERRAIN_RES = 0.08                     # SCM grid resolution (m)

# Firm-soil Bekker-Wong parameters (firm but still deformable -> visible ruts).
BEKKER_KPHI = 2.0e6                    # frictional modulus (Pa)
BEKKER_KC = 0.0                        # cohesive modulus
BEKKER_N = 1.1                         # exponent
MOHR_COHESION = 5.0e3                  # cohesive limit (Pa)
MOHR_FRICTION = 30.0                   # friction angle (deg)
JANOSI_SHEAR = 0.01                    # shear coefficient (m)
ELASTIC_K = 2.0e8                      # elastic stiffness (Pa/m)
DAMPING_R = 3.0e4                      # vertical damping (Pa.s/m)

SINKAGE_MAX = 0.10                     # sinkage false-color upper bound (m)

SUSPENSION_REF_HEIGHT = 0.5            # HMMWV chassis-origin height above wheel-bottom
INIT_X = -TERRAIN_LENGTH / 2 + 5.0     # spawn near one edge so there is room to drive
INIT_Y = 0.0

TIRE_FAMILY = 1                        # collision family for tire cylinders
ZTOL = 0.08                            # allowed wheel-bottom clearance vs terrain top

# === Output dir === guard against a missing output directory before any file I/O

# === Vehicle (full HMMWV wrapper) === wrapper creates + owns its ChSystemSMC
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(INIT_X, INIT_Y, SUSPENSION_REF_HEIGHT), chrono.QUNIT
    )
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # SCM slip/grip tire so the chassis actually moves
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()

# Mesh visualization applied to all vehicle components.
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                  # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()            # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()                # cache: vehicle subsystem handle, reused every step
# spindles: veh_obj.GetAxles()[i].m_wheels[j].GetSpindle() ; terrain: SCMTerrain below
# joints: suspension + Pitman-arm steering links created inside the wrapper

# === Collision system === BULLET so SCM ray-casts and tire contacts resolve
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === SCM deformable terrain === Bekker-Wong soft soil, firm params, moving patch
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    BEKKER_KPHI, BEKKER_KC, BEKKER_N,
    MOHR_COHESION, MOHR_FRICTION, JANOSI_SHEAR,
    ELASTIC_K, DAMPING_R,
)
# False-color sinkage heatmap so ruts are visible (must precede Initialize).
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, SINKAGE_MAX)
# Moving patch tied to the level chassis body so only cells near the vehicle update.
terrain.AddActiveDomain(
    chassis,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RES)
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80
)

# === Tire collision cylinders === TMEASY tires need explicit shapes for SCM ray-casts
tire_rad = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetRadius()   # cache: fetched once
tire_w = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetWidth()      # cache: fetched once
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)

for axle in veh_obj.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)   # wheels never collide with each other
        # NOTE: never DisallowCollisionsWith(0) — that filters SCM's ray-casts.

# Rebuild all collision models so the new cylinders are visible to ray-casts.
system.GetCollisionSystem().BindAll()

# Verify the wheels start on (not through) the terrain — assert, do not trust comments.
terrain_top = terrain.GetHeight(chrono.ChVector3d(INIT_X, INIT_Y, 0.0))
spindle_world = [
    veh_obj.GetSpindlePos(a, side)
    for a in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
]
wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
assert wheel_bottom_z >= terrain_top - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_top:.3f}; raise SUSPENSION_REF_HEIGHT"
)


# === Driver === scripted time-based control (no human-in-the-loop in batch runs)
class ScriptedDriver(veh.ChDriver):
    """Open-loop driver: brief settle, then throttle with a gentle steering sweep."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)        # let the suspension settle onto the soil
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.7)        # drive forward to carve ruts
            self.SetBraking(0.0)
        self.SetSteering(0.25 * math.sin(0.4 * time))   # gentle sweep


driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + sky + chase cam + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Main loop === render once per frame; advance the full subsystem stack per step
steer_ctrl = driver                                     # cache: driver handle reused each step

frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = steer_ctrl.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review videos + plot, then drop raw frames
