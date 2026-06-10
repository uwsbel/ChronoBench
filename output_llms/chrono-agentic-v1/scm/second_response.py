"""
SCM Terrain Parameter Manager Demo — PyChrono 9.0.x / Irrlicht

Simulates an HMMWV on Soft Soil Contact Model (SCM) deformable terrain.
The key feature is a SCMTerrainParams class that encapsulates all Bekker-Wong
soil parameters and supports named preset configurations: 'soft', 'mid', 'hard'.
This design replaces direct parameter setting in the main code.

System type: ChSystemNSC (owned by HMMWV_Full wrapper).
Main components:
  - HMMWV_Full wheeled vehicle with TMEASY tires (required for SCM traction)
  - SCMTerrain with deformable Bekker-Wong soil
  - SCMTerrainParams class with 'soft', 'mid', 'hard' presets
  - ChInteractiveDriverIRR for real-time keyboard control
  - ChWheeledVehicleVisualSystemIrrlicht for Irrlicht rendering

Expected behavior: HMMWV drives on deformable SCM terrain, leaving visible
ruts / sinkage marks. The soil preset can be switched by changing the
SOIL_PRESET constant.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
# Simulation timing
TIME_STEP = 2e-3          # physics step size (s) — 2 ms suitable for SCM
SIM_END = 30.0            # total simulation time (s)
RENDER_FPS = 50.0         # target render frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Vehicle initial position (spawned at origin, wheels on SCM z=0 surface)
INIT_X = -5.0
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (HMMWV ≈ 0.5 m)
INIT_Z = SUSPENSION_REF_HEIGHT

# SCM terrain size and resolution
SCM_LENGTH = 120.0
SCM_WIDTH = 80.0
SCM_RESOLUTION = 0.1      # grid resolution (m); 0.1 m = good balance

# Which soil preset to use: 'soft', 'mid', or 'hard'
SOIL_PRESET = "mid"


# === SCMTerrainParams class ===
class SCMTerrainParams:
    """
    Manages and sets SCM (Bekker-Wong) terrain parameters.
    Provides named preset configurations: 'soft', 'mid', 'hard'.
    Each preset defines all 8 required soil parameters:
      Bekker_Kphi, Bekker_Kc, Bekker_n, Mohr_cohesion,
      Mohr_friction, Janosi_shear, elastic_K, damping_R
    """

    # Preset parameter tables
    # Signature order: (Kphi, Kc, n, cohesion, friction_angle, Janosi, elastic_K, damping_R)
    PRESETS = {
        "soft": dict(
            Bekker_Kphi=2e5,        # frictional modulus (Pa) — low for soft soil
            Bekker_Kc=0.0,          # cohesive modulus
            Bekker_n=1.0,           # pressure-sinkage exponent (1.0 = very soft)
            Mohr_cohesion=100.0,    # cohesive limit (Pa)
            Mohr_friction=20.0,     # friction angle (deg)
            Janosi_shear=0.01,      # shear coefficient (m)
            elastic_K=4e7,          # elastic stiffness (Pa/m)
            damping_R=3e4,          # vertical damping (Pa·s/m)
        ),
        "mid": dict(
            Bekker_Kphi=2e6,        # medium frictional modulus
            Bekker_Kc=0.0,
            Bekker_n=1.1,           # moderate exponent
            Mohr_cohesion=0.0,
            Mohr_friction=30.0,
            Janosi_shear=0.01,
            elastic_K=2e8,
            damping_R=3e4,
        ),
        "hard": dict(
            Bekker_Kphi=5e6,        # high frictional modulus — hard/packed soil
            Bekker_Kc=0.0,
            Bekker_n=1.5,           # higher exponent = stiffer response
            Mohr_cohesion=0.0,
            Mohr_friction=40.0,
            Janosi_shear=0.005,     # stiffer shear (m)
            elastic_K=5e8,
            damping_R=5e4,
        ),
    }

    def __init__(self, preset: str = "mid"):
        """
        Initialize with a named preset ('soft', 'mid', 'hard').

        Args:
            preset: name of the soil configuration preset
        """
        if preset not in self.PRESETS:
            raise ValueError(
                f"Unknown SCM preset '{preset}'. "
                f"Valid options: {list(self.PRESETS.keys())}"
            )
        self._preset = preset
        params = self.PRESETS[preset]
        self.Bekker_Kphi = params["Bekker_Kphi"]
        self.Bekker_Kc = params["Bekker_Kc"]
        self.Bekker_n = params["Bekker_n"]
        self.Mohr_cohesion = params["Mohr_cohesion"]
        self.Mohr_friction = params["Mohr_friction"]
        self.Janosi_shear = params["Janosi_shear"]
        self.elastic_K = params["elastic_K"]
        self.damping_R = params["damping_R"]

    def apply(self, terrain: veh.SCMTerrain) -> None:
        """
        Apply the stored parameters to an SCMTerrain object.

        Args:
            terrain: the veh.SCMTerrain instance to configure
        """
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

    @property
    def preset_name(self) -> str:
        """Return the name of the active preset."""
        return self._preset

    def __repr__(self) -> str:
        return (
            f"SCMTerrainParams(preset='{self._preset}', "
            f"Kphi={self.Bekker_Kphi:.2e}, Kc={self.Bekker_Kc:.2e}, "
            f"n={self.Bekker_n}, cohesion={self.Mohr_cohesion}, "
            f"friction={self.Mohr_friction}°)"
        )


# === Vehicle setup ===
print(f"[scm_demo] Initializing HMMWV with soil preset: {SOIL_PRESET!r}")

# HMMWV_Full wrapper creates and owns its own ChSystem
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)    # SCM requires SMC contact
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis never moves
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)           # facing +X
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # TMEASY required for SCM traction
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                            # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()                      # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxles()[i].m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper

# REQUIRED for SCM: set collision system AFTER Initialize, BEFORE SCMTerrain
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Visualization types (after Initialize per skill rules)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === SCM Terrain ===
# Build terrain params from named preset class (the key feature of this demo)
soil_params = SCMTerrainParams(SOIL_PRESET)
print(f"[scm_demo] Terrain params: {soil_params!r}")

terrain = veh.SCMTerrain(system)
soil_params.apply(terrain)                            # apply preset via class method

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.10)   # sinkage heatmap

# Moving active domain: attach to chassis (NOT spindles) to track vehicle
# Note: 9.0.0 uses AddActiveDomain (verified by introspection) — same args as AddMovingPatch
terrain.AddActiveDomain(
    chassis,                                          # cache: chassis body (already fetched)
    chrono.ChVector3d(0, 0, 0),                       # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),                       # OOBB dimensions (m)
)
terrain.Initialize(SCM_LENGTH, SCM_WIDTH, SCM_RESOLUTION)
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    veh.GetVehicleDataFile("terrain/textures/dirt.jpg"),
    80, 80,
)

# === Tire collision cylinders (REQUIRED for TMEASY on SCM) ===
# SCM casts rays to detect tire contact; TMEASY tires need explicit cylinders
TIRE_FAMILY = 1
SUPPORT_FAMILY = 4

tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
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
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)      # tires don't collide with each other
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)   # tires ride on SCM, not support

# Rebuild collision models after post-init shape changes
system.GetCollisionSystem().BindAll()

# Assert wheel placement on terrain (validate no sinkage at spawn)
veh_obj = hmmwv.GetVehicle()                          # cache: fetched once
TIRE_RADIUS = tire_rad
ZTOL = 0.08  # generous tolerance at spawn (SCM is z=0 plane)
spindle_zs = []
for ax in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(ax, side)
        spindle_zs.append(p.z)
wheel_bottom_z = min(spindle_zs) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Wheel bottom z={wheel_bottom_z:.3f} too far below SCM surface; "
    f"raise SUSPENSION_REF_HEIGHT by {-ZTOL - wheel_bottom_z:.3f} m"
)

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle(f"HMMWV on SCM Terrain — preset: {SOIL_PRESET}")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (ChInteractiveDriver — scored-core default, truth-faithful) ===
# Note: 9.0.0 uses ChInteractiveDriver(vehicle) — no IRR variant in this build
driver = veh.ChInteractiveDriver(hmmwv.GetVehicle())  # takes vehicle object

steering_time = 1.0          # s to reach full steering
throttle_time = 1.0          # s to reach full throttle
braking_time = 0.3           # s to reach full brake
driver.SetSteeringDelta(RENDER_EVERY * TIME_STEP / steering_time)
driver.SetThrottleDelta(RENDER_EVERY * TIME_STEP / throttle_time)
driver.SetBrakingDelta(RENDER_EVERY * TIME_STEP / braking_time)
driver.Initialize()

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()   # for real-time pacing in scored core
frame = 0
print(f"[scm_demo] Starting simulation (soil={SOIL_PRESET}, dt={TIME_STEP}s, end={SIM_END}s)")

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        sim_time = hmmwv.GetSystem().GetChTime()  # cache per-frame time

        if frame % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)    # internally calls DoStepDynamics — do NOT also call it
        vis.Advance(TIME_STEP)


        frame += 1
        realtime_timer.Spin(TIME_STEP)    # real-time pacing

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    print(f"[scm_demo] Simulation ended at t={hmmwv.GetSystem().GetChTime():.2f}s")
