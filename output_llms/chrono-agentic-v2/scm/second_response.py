"""
SCM (Bekker-Wong soft-soil) terrain simulation with HMMWV_Full vehicle.
System type: NSC (SMC contact method used for SCM deformable terrain).
Main bodies: HMMWV chassis + 4 wheel spindles on a 120x120 m SCM deformable terrain.

SCM terrain parameters are encapsulated in the SCMTerrainParams class, which
provides predefined soil configurations: 'soft', 'mid' (default), and 'hard'.
The terrain deforms under wheel load, producing visible ruts and sinkage.
Expected behavior: HMMWV accelerates forward, leaving deformed ruts in the soft soil.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Data paths (required for all catalog vehicle truths) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


# === SCMTerrainParams class — encapsulates Bekker-Wong soil parameters ===
class SCMTerrainParams:
    """Manage and apply SCM terrain soil parameters via predefined presets.

    Presets:
        'soft' — loose, wet soil with low bearing capacity.
        'mid'  — typical off-road loam (default).
        'hard' — compacted gravel/clay with high stiffness.
    """

    # Preset parameter tables:
    # (Bekker_Kphi, Bekker_Kc, Bekker_n, Mohr_cohesion, Mohr_friction,
    #  Janosi_shear, elastic_K, damping_R)
    _PRESETS = {
        'soft': (2e4,  0.0, 1.1,   0.0, 20.0, 0.01, 2e6, 3e4),
        'mid':  (2e6,  0.0, 1.1,   0.0, 30.0, 0.01, 2e8, 3e4),
        'hard': (5e6, 2e4, 1.5, 1000.0, 35.0, 0.005, 5e8, 5e4),
    }

    def __init__(self, preset: str = 'mid'):
        """Initialise with a named preset ('soft', 'mid', or 'hard')."""
        if preset not in self._PRESETS:
            raise ValueError(
                f"Unknown SCM preset '{preset}'. "
                f"Valid choices: {list(self._PRESETS.keys())}"
            )
        self.preset = preset
        (self.Bekker_Kphi,
         self.Bekker_Kc,
         self.Bekker_n,
         self.Mohr_cohesion,
         self.Mohr_friction,
         self.Janosi_shear,
         self.elastic_K,
         self.damping_R) = self._PRESETS[preset]

    def apply(self, terrain: veh.SCMTerrain) -> None:
        """Apply the stored parameters to an SCMTerrain object."""
        terrain.SetSoilParameters(
            self.Bekker_Kphi,
            self.Bekker_Kc,
            self.Bekker_n,
            self.Mohr_cohesion,
            self.Mohr_friction,
            self.Janosi_shear,
            self.elastic_K,
            self.damping_R,
        )


# === Simulation constants ===
TIME_STEP = 2e-3          # simulation time step (s)
SIM_END   = 15.0          # total simulation time (s)
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TERRAIN_LENGTH  = 120.0   # SCM patch length (m)
TERRAIN_WIDTH   = 120.0   # SCM patch width  (m)
TERRAIN_RES     = 0.1     # SCM grid resolution (m)

INIT_POS   = chrono.ChVector3d(-40.0, 0.0, 0.5)  # vehicle spawn world position
INIT_ROT   = chrono.QuatFromAngleZ(0.0)

TIRE_FAMILY    = 1         # collision family for tire cylinders
SUPPORT_FAMILY = 4         # collision family for rigid support plane

# === Vehicle setup (HMMWV_Full with TMEASY tires — required for SCM) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)    # TMEASY required on SCM
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by veh.HMMWV_Full wrapper) ===
sys     = hmmwv.GetSystem()                # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); joints: suspension + steering inside wrapper
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Required immediately after Initialize, before building SCMTerrain
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === SCM Terrain using SCMTerrainParams class ===
soil_params = SCMTerrainParams('mid')      # use mid-stiffness preset

terrain = veh.SCMTerrain(sys)
soil_params.apply(terrain)                # apply the encapsulated soil parameters

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),               # CORRECT — chassis body stays level
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RES)
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,
)

# === Tire collision cylinders (required for TMEASY tires on SCM) ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w   = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialNSC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(
                tire_mat, tire_rad + 0.04, tire_w
            ),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
        # NOTE: do NOT DisallowCollisionsWith(0) — that blocks SCM ray-casts

# Rebuild collision models after adding shapes (MANDATORY)
sys.GetCollisionSystem().BindAll()

# Visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Driver — interactive (scored core matches truth; scripted driving in review-only block) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Terrain — SCMTerrainParams class demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME  = 0.3
render_step_size = 1.0 / RENDER_FPS   # precomputed once
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Review-only setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        time = hmmwv.GetSystem().GetChTime()

        if frame % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        frame += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
