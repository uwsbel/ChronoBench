"""HMMWV driving on deformable SCM (Bekker-Wong soft-soil) terrain.

Model
-----
A full HMMWV wheeled vehicle (veh.HMMWV_Full, NSC contact via the SMC contact
method selected on the wrapper) drives forward across a deformable SCMTerrain
patch and ploughs visible ruts into the soil. The vehicle uses TMEASY tires
(RIGID tires do not generate slip/grip on SCM and the chassis would not move);
explicit per-spindle collision cylinders are added so SCM's ray-casts detect the
wheels and deform the soil.

SCM soil parameters are NOT set with scattered literal calls. Instead they are
encapsulated in a small configuration class (SCMSoilConfig) with three named
presets — "soft", "mid", and "hard" — each carrying the full eight-parameter
Bekker-Wong set. A single preset object is selected by name and applied to the
terrain in one call. This keeps the soil definition in one auditable place and
makes switching soil regimes a one-line change.

System type: NSC system owned by the HMMWV_Full wrapper (SMC contact method).
Main bodies: HMMWV chassis + 4 spindles/wheels, deformable SCM terrain grid,
hidden rigid support plane (keeps the deformable surface from sagging at edges).
Expected behavior: the chassis accelerates forward from rest, the wheels sink
slightly into the firm soil, and continuous ruts form along the driven path.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants: timing, geometry, control ===
TIME_STEP = 2.0e-3                 # integration step (s) — SCM is stiff; keep modest
TIRE_STEP = 1.0e-3                 # TMEASY tire sub-step (s); required on SCM
SIM_END = 6.0                      # simulation horizon (s) — SCM is slow, keep modest
RENDER_FPS = 30.0                  # review-video frame rate

TERRAIN_LENGTH = 44.0              # SCM patch X extent (m): contains full travel
TERRAIN_WIDTH = 6.0                # SCM patch Y extent (m): narrow corridor
TERRAIN_RES = 0.10                 # SCM grid resolution (m): balance ruts vs cost

VEH_INIT_X = -18.0                 # spawn near the back of the patch, drive +X
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis-origin height above wheel-bottom
ZTOL = 0.08                        # allowed wheel-bottom vs terrain-top clearance (m)

TIRE_RADIUS_GUESS = 0.46          # HMMWV tire radius (m), refined from the wheel API
TIRE_COLLISION_PAD = 0.04          # extra radius so SCM ray-casts detect sinkage (m)

TIRE_FAMILY = 1                    # collision family for spindle cylinders
SUPPORT_FAMILY = 4                 # collision family for the hidden support plane

THROTTLE_RAMP_END = 1.0            # ramp throttle 0->target over this many seconds
TARGET_THROTTLE = 0.5              # cruising throttle: bounds travel within patch

SOIL_PRESET = "hard"               # firm soil so the HMMWV drives without bogging down


# === SCM soil configuration class (encapsulated presets) ===
class SCMSoilConfig:
    """Encapsulates the full eight-parameter Bekker-Wong SCM soil parameter set.

    Replaces ad-hoc inline SetSoilParameters(...) literals with a named, reusable
    configuration. Pick a preset by name with SCMSoilConfig.preset("soft"|"mid"|
    "hard"), then apply it to a veh.SCMTerrain with .apply(terrain). The eight
    fields map one-to-one onto SCMTerrain.SetSoilParameters in declaration order.
    """

    # Field order matches SCMTerrain.SetSoilParameters exactly.
    __slots__ = (
        "name",
        "bekker_kphi",     # frictional modulus (Pa)
        "bekker_kc",       # cohesive modulus (Pa)
        "bekker_n",        # sinkage exponent (1.0 soft -> 1.5 hard)
        "mohr_cohesion",   # cohesive limit (Pa)
        "mohr_friction",   # internal friction angle (deg)
        "janosi_shear",    # Janosi-Hanamoto shear coefficient (m)
        "elastic_k",       # elastic stiffness (Pa/m)
        "damping_r",       # vertical damping (Pa.s/m)
    )

    # Predefined soil regimes. "hard" is firm enough that the HMMWV drives across
    # it leaving shallow ruts rather than bogging down.
    PRESETS = {
        "soft": dict(
            bekker_kphi=0.2e6, bekker_kc=0.0, bekker_n=1.0,
            mohr_cohesion=1.0e3, mohr_friction=20.0, janosi_shear=0.01,
            elastic_k=2.0e8, damping_r=3.0e4,
        ),
        "mid": dict(
            bekker_kphi=1.0e6, bekker_kc=0.0, bekker_n=1.1,
            mohr_cohesion=3.0e3, mohr_friction=25.0, janosi_shear=0.01,
            elastic_k=2.0e8, damping_r=3.0e4,
        ),
        "hard": dict(
            bekker_kphi=2.0e6, bekker_kc=0.0, bekker_n=1.2,
            mohr_cohesion=5.0e3, mohr_friction=30.0, janosi_shear=0.01,
            elastic_k=2.0e8, damping_r=3.0e4,
        ),
    }

    def __init__(self, name, **params):
        self.name = name
        for field in self.__slots__:
            if field == "name":
                continue
            setattr(self, field, params[field])

    @classmethod
    def preset(cls, name):
        """Build a config from a predefined regime name ('soft', 'mid', 'hard')."""
        if name not in cls.PRESETS:
            raise KeyError(
                f"unknown SCM soil preset '{name}'; "
                f"choose one of {sorted(cls.PRESETS)}"
            )
        return cls(name, **cls.PRESETS[name])

    def apply(self, terrain):
        """Apply this eight-parameter set to a veh.SCMTerrain in one call."""
        terrain.SetSoilParameters(
            self.bekker_kphi,
            self.bekker_kc,
            self.bekker_n,
            self.mohr_cohesion,
            self.mohr_friction,
            self.janosi_shear,
            self.elastic_k,
            self.damping_r,
        )


# === Scripted driver (open-loop throttle ramp, straight ahead) ===
class RampDriver(veh.ChDriver):
    """Time-based scripted driver: ramp throttle up, hold straight steering.

    Subclasses veh.ChDriver and drives state through the Set* setters inside
    Synchronize (the headless pipeline has no keyboard, so an interactive driver
    would leave the throttle at zero and the vehicle would never move).
    """

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < THROTTLE_RAMP_END:
            self.SetThrottle(TARGET_THROTTLE * (time / THROTTLE_RAMP_END))
        else:
            self.SetThrottle(TARGET_THROTTLE)
        self.SetSteering(0.0)
        self.SetBraking(0.0)


def main():
    # Validation gate: a short, windowless physics check when SIMBENCH_VALIDATE
    # is set. The full Irrlicht block below is always present in the source.
    headless = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless check
    run_end = min(SIM_END, 1.0) if headless else SIM_END   # short physics check

    os.makedirs("frames", exist_ok=True)   # guard against missing frame output dir
    os.makedirs("cam", exist_ok=True)      # guard against missing cam output dir

    # === Soil config: pick a preset BEFORE building the terrain ===
    soil = SCMSoilConfig.preset(SOIL_PRESET)   # one named, auditable soil definition

    # === Vehicle (HMMWV_Full wrapper owns and creates its own system) ===
    # The wrapper builds the ChSystem (SMC contact), the chassis rigid body, four
    # spindle/wheel bodies, the suspension + steering joints, and the powertrain.
    init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 0.0)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # RIGID will not move on SCM
    hmmwv.SetTireStepSize(TIRE_STEP)              # required for TMEASY on SCM

    # Pre-set the spawn pose (drive across +X). SCM rest plane is z=0, so the
    # wheel-bottom target is ~0; place the chassis origin one suspension height up.
    init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, SUSPENSION_REF_HEIGHT)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()              # ChSystem owned by the wrapper (SMC)
    veh_obj = hmmwv.GetVehicle()            # cache: vehicle subsystem, reused below
    chassis = hmmwv.GetChassisBody()        # cache: main chassis body, reused/logged
    # wheels/spindles: veh_obj.GetAxles()[i].m_wheels[j].GetSpindle()
    # joints: suspension + steering links created inside the wrapper

    # Real tire radius/width from the wheel API (refine the constant guess).
    tire_obj = veh_obj.GetAxles()[0].m_wheels[0].GetTire()
    tire_rad = tire_obj.GetRadius()         # cache: constant tire radius (m)
    tire_w = tire_obj.GetWidth()            # cache: constant tire width (m)

    # === Deformable SCM terrain (Bekker-Wong soft soil) ===
    # The wrapper already created a collision system, so SCMTerrain can be built
    # directly. Apply the encapsulated soil preset, then initialize the grid.
    terrain = veh.SCMTerrain(system)
    soil.apply(terrain)                     # apply the named soil preset in one call
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.10)  # sinkage heatmap
    terrain.SetMeshWireframe(False)
    terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RES)
    terrain.SetTexture(
        chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80
    )

    # === Tire collision cylinders (REQUIRED for TMEASY on SCM) ===
    # TMEASY tires carry no collision geometry; SCM ray-casts find nothing without
    # explicit cylinders. Pad the radius so the cylinder dips below the rest plane.
    tire_mat = chrono.ChContactMaterialSMC()
    tire_mat.SetFriction(0.9)
    tire_mat.SetRestitution(0.1)
    tire_mat.SetYoungModulus(2e7)

    for axle in veh_obj.GetAxles():
        for iw in range(2):
            spindle = axle.m_wheels[iw].GetSpindle()
            spindle.AddCollisionShape(
                chrono.ChCollisionShapeCylinder(
                    tire_mat, tire_rad + TIRE_COLLISION_PAD, tire_w
                ),
                chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
            )
            spindle.EnableCollision(True)
            sp_cm = spindle.GetCollisionModel()
            sp_cm.SetFamily(TIRE_FAMILY)
            sp_cm.DisallowCollisionsWith(TIRE_FAMILY)     # tires never self-contact
            sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)  # tires ride SCM, not plane
            # NOTE: never DisallowCollisionsWith(0) — that filters SCM ray-casts.

    # === Hidden rigid support plane (stabilizes the SCM patch edges) ===
    support_mat = chrono.ChContactMaterialSMC()
    support_mat.SetFriction(0.9)
    support_mat.SetRestitution(0.01)
    support_mat.SetYoungModulus(2e7)
    support = chrono.ChBodyEasyBox(
        TERRAIN_LENGTH, TERRAIN_WIDTH, 0.2, 1000, False, True, support_mat
    )
    support.SetName("scm_support_ground")
    support.SetPos(chrono.ChVector3d(0, 0, -0.2))   # top below the SCM rest plane
    support.SetFixed(True)
    support.EnableCollision(True)
    support_cm = support.GetCollisionModel()
    support_cm.SetFamily(SUPPORT_FAMILY)
    support_cm.DisallowCollisionsWith(TIRE_FAMILY)  # tires ride SCM, not the plane
    system.AddBody(support)

    # MANDATORY after post-init collision-shape edits: rebuild Bullet models so
    # the new spindle cylinders are visible to SCM ray-casts.
    system.GetCollisionSystem().BindAll()

    # === Footprint assertion (wheels rest on, not through, the SCM surface) ===
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    terrain_top_z = terrain.GetHeight(init_loc)     # SCM surface height at spawn
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
    assert wheel_bottom_z >= terrain_top_z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={terrain_top_z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{terrain_top_z - wheel_bottom_z:.3f} m"
    )

    # === Driver (scripted throttle ramp, straight ahead) ===
    driver = RampDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not headless:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on deformable SCM terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.6)
        vis.Initialize()                                   # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                    # outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 8, -8, 4),
                      chrono.ChVector3d(VEH_INIT_X, 0, 0))  # AFTER Initialize
        vis.AddTypicalLights()                             # standard lighting
        vis.AddGrid(1.0, 1.0, int(TERRAIN_LENGTH), int(TERRAIN_WIDTH),
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))         # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Precomputed loop constants (computed once before the loop) ===
    render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

    # === Main loop (Synchronize/Advance; no DoStepDynamics for wrapper veh) ===
    data_f = None
    motion_f = None
    times, speeds, xs, sinkages = [], [], [], []
    try:
        data_f = open("simulation_data.csv", "w", newline="")
        motion_f = open("cam/motion_log.csv", "w", newline="")
        data_w = csv.writer(data_f)
        motion_w = csv.writer(motion_f)
        data_w.writerow(
            ["time", "chassis_x", "chassis_y", "chassis_z", "speed", "throttle"]
        )
        motion_w.writerow(
            ["time", "body", "x", "y", "z", "vx", "vy", "vz"]
        )

        frame = 0
        while (headless or vis.Run()) and system.GetChTime() < run_end:
            if not headless:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive
                frame += 1

            for _ in range(render_every):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                # --- log physics each step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_w.writerow([
                    f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                    f"{pos.z:.5f}", f"{speed:.5f}", f"{driver_inputs.m_throttle:.4f}"
                ])
                motion_w.writerow([
                    f"{sim_time:.5f}", "chassis", f"{pos.x:.5f}", f"{pos.y:.5f}",
                    f"{pos.z:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"
                ])
                times.append(sim_time)
                speeds.append(speed)
                xs.append(pos.x)
                sinkages.append(terrain.GetHeight(init_loc))

                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                if not headless:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned system
                if not headless:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= run_end:
                    break

    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:            # disk / permission on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing: timeseries plot from the logged data ===
    if times:
        t = np.array(times)
        fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax[0].plot(t, np.array(speeds), label="chassis speed (m/s)")
        ax[0].set_ylabel("speed (m/s)")
        ax[0].grid(True)
        ax[0].legend(loc="best")
        ax[1].plot(t, np.array(xs), color="tab:orange", label="chassis x (m)")
        ax[1].set_xlabel("time (s)")
        ax[1].set_ylabel("x position (m)")
        ax[1].grid(True)
        ax[1].legend(loc="best")
        fig.suptitle(f"HMMWV on SCM terrain (soil preset: {SOIL_PRESET})")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        # Console summary so the run is verifiable from stdout.
        print(f"[summary] soil={SOIL_PRESET} steps={len(times)} "
              f"x0={xs[0]:.3f} xT={xs[-1]:.3f} dx={xs[-1] - xs[0]:.3f} "
              f"max_speed={max(speeds):.3f}")


if __name__ == "__main__":
    main()
