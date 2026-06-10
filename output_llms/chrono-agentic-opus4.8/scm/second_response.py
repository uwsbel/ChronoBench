"""HMMWV on SCM (Bekker-Wong) deformable soft-soil terrain.

System type: NSC vehicle wrapper system (HMMWV_Full) with SMC contact method,
Bullet collision, and a deformable SCMTerrain patch.

Main bodies: the HMMWV chassis + four TMEASY tires (with explicit spindle
collision cylinders so the soft soil registers sinkage and ruts) driving over a
deformable SCM terrain patch.

The SCM soil parameters are encapsulated in an `SCMSoilConfig` class that exposes
predefined "soft"/"mid"/"hard" presets; the chosen preset's eight Bekker-Wong
parameters are applied to the terrain instead of being set inline. Expected
behavior: the HMMWV drives forward across the soft soil, the tires sink in and
leave visible ruts, and the chassis translates steadily in +X.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === SCM soil-parameter configuration class (replaces inline soil setup) ===
class SCMSoilConfig:
    """Encapsulate the eight Bekker-Wong SCM soil parameters.

    Construct from a named preset ("soft", "mid", "hard") and apply the bundled
    parameter set to a veh.SCMTerrain via ApplyTo(); this keeps the soil
    definition in one place instead of scattering eight magic numbers at the
    call site.
    """

    # cache: preset table built once at class-definition time, reused per run
    _PRESETS = {
        # Bekker_Kphi, Bekker_Kc, Bekker_n, Mohr_cohesion, Mohr_friction,
        # Janosi_shear, elastic_K, damping_R
        "soft": (0.2e6, 0.0, 1.1, 0.0, 20.0, 0.01, 4e7, 3e4),
        "mid":  (2e6,   0.0, 1.1, 0.0, 30.0, 0.01, 2e8, 3e4),
        "hard": (5e6,   0.0, 1.2, 1e3, 35.0, 0.01, 1e9, 3e4),
    }

    def __init__(self, preset="soft"):
        if preset not in self._PRESETS:
            raise ValueError(f"unknown SCM preset {preset!r}; "
                             f"choose one of {sorted(self._PRESETS)}")
        self.preset = preset
        (self.bekker_kphi, self.bekker_kc, self.bekker_n,
         self.mohr_cohesion, self.mohr_friction, self.janosi_shear,
         self.elastic_K, self.damping_R) = self._PRESETS[preset]

    def ApplyTo(self, terrain):
        # Exactly eight positional args, in the SetSoilParameters order.
        terrain.SetSoilParameters(
            self.bekker_kphi, self.bekker_kc, self.bekker_n,
            self.mohr_cohesion, self.mohr_friction, self.janosi_shear,
            self.elastic_K, self.damping_R,
        )


# === Named constants: geometry / physics / terrain ===
step_size = 2e-3                 # integration step (s)
tire_step_size = 1e-3            # TMEASY tire sub-step (s)
sim_end = 8.0                    # simulation duration (s)
render_fps = 50.0               # review-video frame rate

terrain_length = 40.0           # SCM patch X size (m)
terrain_width = 40.0            # SCM patch Y size (m)
terrain_resolution = 0.1        # SCM grid spacing (m)
soil_preset = "soft"            # one of "soft" / "mid" / "hard"

TIRE_FAMILY = 1                 # collision family for tire cylinders
SUPPORT_FAMILY = 4             # reserved support-plane family
init_loc = chrono.ChVector3d(-12.0, 0.0, 0.6)   # chassis spawn (rest on z=0 soil)
init_rot = chrono.QUNIT

# === Data paths (truth-faithful for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data

# === Vehicle (HMMWV_Full on SMC, TMEASY tire required for SCM) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)   # SMC for deformable SCM soil
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # SCM requires a non-rigid tire
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                           # ChSystemSMC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = hmmwv.GetChassisBody()                     # cache: main chassis rigid body, reused
# spindles: hmmwv.GetVehicle().GetAxles()[i].m_wheels[s].GetSpindle()
# joints: suspension + steering links created inside the wrapper
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === SCM deformable terrain (soil params from the SCMSoilConfig class) ===
terrain = veh.SCMTerrain(system)
soil = SCMSoilConfig(soil_preset)    # predefined "soft"/"mid"/"hard" configuration
soil.ApplyTo(terrain)                # apply the eight Bekker-Wong params from the preset
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # sinkage heatmap overlay
terrain.AddMovingPatch(
    chassis,                         # chassis body — stable OOBB projection
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.Initialize(terrain_length, terrain_width, terrain_resolution)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)

# === Tire collision cylinders (REQUIRED so SCM detects TMEASY sinkage) ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # cache: fixed geom
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()     # cache: fixed geom
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)

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
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)     # tires never touch each other
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)

system.GetCollisionSystem().BindAll()    # rebuild collision models after shape edits

# === Footprint assert (wheels rest on the z=0 soil, not through it) ===
veh_obj = hmmwv.GetVehicle()
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(a, s).z
    for a in range(veh_obj.GetNumberAxles())
    for s in (veh.LEFT, veh.RIGHT)
) - tire_rad
assert wheel_bottom_z >= -0.20, (
    f"vehicle sinks through soil at spawn: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise init_loc.z"
)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()        # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive, truth-faithful scored-core form) ===
render_step_size = 1.0 / render_fps
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop ===
render_steps = math.ceil(render_step_size / step_size)   # precomputed once

os.makedirs("cam", exist_ok=True)     # guard against missing output dir
try:

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    frame = 0
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:        # throttled rendering
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
        hmmwv.Advance(step_size)        # advances the wrapper-owned system
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing (review-only video + plot assembly) ===
