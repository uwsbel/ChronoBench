"""HMMWV driving on deformable SCM (Bekker-Wong) soft-soil terrain.

System type: vehicle-owned ChSystemSMC (created by the veh.HMMWV_Full wrapper).
Main bodies: the HMMWV chassis + four spindle/wheel assemblies, riding on a
deformable SCMTerrain patch. Soil parameters are encapsulated in an
SCMSoilConfig class that ships predefined "soft" / "mid" / "hard" presets and
applies the chosen preset to the terrain, instead of setting the eight Bekker /
Mohr / Janosi coefficients inline.

Expected behavior: under a steady forward throttle the HMMWV (TMEASY tires with
explicit per-spindle collision cylinders) drives across the firm-soil patch in a
straight line, sinking slightly and leaving visible ruts behind the wheels while
the chassis clearly translates forward.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Tunable constants === geometry / timing / control (no bare literals downstream)
TIME_STEP = 2e-3                 # integration step (s)
TIRE_STEP = 1e-3                 # TMEASY tire substep (s) — required on SCM
SIM_END = 5.0                    # modest run kept within the patch footprint
RENDER_FPS = 50.0
THROTTLE = 0.7                   # steady forward throttle
STEERING = 0.0                   # straight line

PATCH_LEN = 24.0                 # SCM patch X size (m) — sized so the run stays on it
PATCH_WID = 12.0                 # SCM patch Y size (m)
PATCH_RES = 0.08                 # SCM grid resolution (m)

TIRE_FAMILY = 1                  # collision family for the tire cylinders
TIRE_RAD_PAD = 0.04              # extra radius so SCM detects sinkage (Rule 1)

SUSPENSION_REF_HEIGHT = 0.5      # HMMWV chassis-origin height above wheel-bottom
INIT_X = -PATCH_LEN / 2 + 4.0    # spawn near the back edge, room to drive forward
INIT_Y = 0.0
ZTOL = 0.10                      # allowed wheel-bottom clearance/overlap vs terrain


# === SCM soil configuration === encapsulates the 8 Bekker/Mohr/Janosi params
class SCMSoilConfig:
    """Manage SCM soil parameters via named presets.

    Replaces direct inline SetSoilParameters(...) calls: pick a preset
    ("soft" / "mid" / "hard") and call apply(terrain) to push the eight
    coefficients (Bekker_Kphi, Bekker_Kc, Bekker_n, Mohr_cohesion,
    Mohr_friction, Janosi_shear, elastic_K, damping_R) onto the terrain.
    """

    PRESETS = {
        # soft mud: low frictional modulus, some cohesion -> deep sinkage
        "soft": dict(Bekker_Kphi=2e5, Bekker_Kc=0.0, Bekker_n=1.0,
                     Mohr_cohesion=2e3, Mohr_friction=20.0, Janosi_shear=0.01,
                     elastic_K=1e8, damping_R=3e4),
        # mid: moderate firmness
        "mid": dict(Bekker_Kphi=8e5, Bekker_Kc=0.0, Bekker_n=1.1,
                    Mohr_cohesion=3e3, Mohr_friction=25.0, Janosi_shear=0.01,
                    elastic_K=2e8, damping_R=3e4),
        # firm soil: high frictional modulus, drives cleanly with shallow ruts
        "hard": dict(Bekker_Kphi=2e6, Bekker_Kc=0.0, Bekker_n=1.1,
                     Mohr_cohesion=5e3, Mohr_friction=30.0, Janosi_shear=0.01,
                     elastic_K=2e8, damping_R=3e4),
    }

    def __init__(self, preset="hard"):
        if preset not in self.PRESETS:
            raise ValueError(f"unknown soil preset {preset!r}; "
                             f"choose one of {sorted(self.PRESETS)}")
        self.preset = preset
        self.params = dict(self.PRESETS[preset])

    def apply(self, terrain):
        """Push the eight soil coefficients onto an SCMTerrain (positional order)."""
        p = self.params
        terrain.SetSoilParameters(
            p["Bekker_Kphi"], p["Bekker_Kc"], p["Bekker_n"],
            p["Mohr_cohesion"], p["Mohr_friction"], p["Janosi_shear"],
            p["elastic_K"], p["damping_R"],
        )


# === Scripted driver === steady forward throttle, no steering (time-based)
class StraightLineDriver(veh.ChDriver):
    """Time-based driver: brief settle, then constant forward throttle."""

    def __init__(self, vehicle, throttle, steering):
        super().__init__(vehicle)
        self._throttle = throttle
        self._steering = steering

    def Synchronize(self, time):
        if time < 0.5:                      # let the suspension settle on the soil
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(self._throttle)
            self.SetBraking(0.0)
        self.SetSteering(self._steering)


# === Output dirs (guard against missing dir) ===


def build_and_run():
    # === Vehicle (HMMWV_Full wrapper owns the ChSystemSMC + all bodies/joints) ===
    init_z = 0.0 + SUSPENSION_REF_HEIGHT   # precomputed once: terrain rest plane z=0
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, init_z)
    init_rot = chrono.QUNIT

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # SCM: never the default RIGID tire
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # --- Wrapper-created components made visible (system + bodies + collision) ---
    system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused
    veh_obj = hmmwv.GetVehicle()               # cache: vehicle sub-API, reused below
    # spindles: veh_obj.GetAxles()[i].m_wheels[side].GetSpindle() (looped below)
    # joints: suspension + steering links created inside the HMMWV_Full wrapper

    # Collision system: REQUIRED for the vehicle/terrain contact + SCM ray-casts.
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain (deformable SCM soft soil) ===
    terrain = veh.SCMTerrain(system)
    soil = SCMSoilConfig("hard")               # firm-soil preset -> clean drive, ruts
    soil.apply(terrain)
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # sinkage heatmap
    terrain.SetMeshWireframe(False)
    terrain.Initialize(PATCH_LEN, PATCH_WID, PATCH_RES)
    terrain.SetTexture(
        chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 40, 40,
    )

    # === Tire collision cylinders (TMEASY tires need explicit per-spindle shapes) ===
    tire0 = veh_obj.GetAxles()[0].m_wheels[0].GetTire()   # cache: tire geom, reused
    tire_rad = tire0.GetRadius()
    tire_w = tire0.GetWidth()
    tire_mat = chrono.ChContactMaterialSMC()
    tire_mat.SetFriction(0.9)
    tire_mat.SetRestitution(0.1)
    tire_mat.SetYoungModulus(1e7)

    cyl_rot = chrono.QuatFromAngleX(math.pi / 2)   # precomputed once: cylinder axis -> Y
    for axle in veh_obj.GetAxles():
        for iw in range(2):
            spindle = axle.m_wheels[iw].GetSpindle()
            spindle.AddCollisionShape(
                chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + TIRE_RAD_PAD, tire_w),
                chrono.ChFramed(chrono.VNULL, cyl_rot),
            )
            spindle.EnableCollision(True)
            sp_cm = spindle.GetCollisionModel()
            sp_cm.SetFamily(TIRE_FAMILY)
            sp_cm.DisallowCollisionsWith(TIRE_FAMILY)   # wheels never collide each other
            # NOTE: never DisallowCollisionsWith(0) — that filters SCM ray-casts.

    # Rebuild all collision models so the new cylinders are visible to ray-casts.
    system.GetCollisionSystem().BindAll()

    # Verify the wheels rest on (not through) the terrain at spawn.
    spindle_z = [veh_obj.GetSpindlePos(a, s).z
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
    wheel_bottom_z = min(spindle_z) - tire_rad
    assert wheel_bottom_z >= 0.0 - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z=0.0; raise SUSPENSION_REF_HEIGHT by "
        f"{0.0 - wheel_bottom_z:.3f} m"
    )

    # === Driver (scripted straight-line forward) ===
    driver = StraightLineDriver(veh_obj, THROTTLE, STEERING)
    driver.Initialize()

    # === Visualization === full vehicle-aware Irrlicht scene + chase camera
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV on SCM deformable terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.6)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddGrid(1.0, 1.0, int(PATCH_LEN), int(PATCH_WID),
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid on the patch
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)

    # === Main loop === render-cadence outer loop; Synchronize/Advance the stack
    render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(render_every):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()
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


# === Entry point ===
if __name__ == "__main__":
    build_and_run()
