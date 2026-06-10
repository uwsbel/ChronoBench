"""HMMWV full vehicle driving on SCM deformable (Bekker-Wong soft-soil) terrain.

Model
-----
- A full High-Mobility Multipurpose Wheeled Vehicle (`veh.HMMWV_Full`) is spawned
  on a deformable SCM soft-soil patch. The wrapper owns an SMC (`ChSystemSMC`)
  contact system; terrain, driver and visualization are all attached to that same
  owned system.
- The terrain is `veh.SCMTerrain` with custom Bekker / Mohr-Coulomb / Janosi soil
  parameters. A moving (active) patch is attached to the chassis body so only the
  cells near the vehicle are deformed each step (large patches are otherwise
  prohibitively expensive). Sinkage is rendered as a false-colour heatmap via
  `SetPlotType(PLOT_SINKAGE, ...)`.
- A scripted time-based driver (`veh.ChDriver` subclass) supplies steering /
  throttle / braking — open-loop, headless-safe (interactive keyboard input is
  always zero in a windowless run).
- Tire model: TMEASY. The vehicle must actually translate across the soft soil
  and cut visible ruts; an explicit slip/grip force model (TMEASY) with collision
  cylinders on each spindle is what lets SCM register the contact and lets the
  chassis drive. With non-rigid tires SCM needs explicit per-spindle collision
  cylinders, added below.

Expected behaviour
------------------
After a brief settle the HMMWV accelerates forward, its wheels sink slightly into
the soil and leave deepening ruts; the chassis X-position increases monotonically
while the soft soil deforms under the active patch.
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

# === Named constants: geometry / physics / soil / control ===
TIME_STEP = 2.0e-3              # integration step (s) — SCM is stiff, keep modest
TIRE_STEP_SIZE = 1.0e-3         # TMEASY tire substep (s)
SIM_END = 6.0                   # simulation duration (s); SCM is slow -> keep modest
RENDER_FPS = 50.0               # review video frame rate (prompt: 50 fps)

INIT_X = -4.0                   # chassis spawn X (m), drives toward +X
INIT_Y = 0.0                    # chassis spawn Y (m)
SUSPENSION_REF_HEIGHT = 0.55    # HMMWV chassis-origin height above wheel-bottom (m)
TERRAIN_REST_Z = 0.0            # SCM undeformed surface plane (m)
INIT_Z = TERRAIN_REST_Z + SUSPENSION_REF_HEIGHT  # derived chassis-origin Z

TERRAIN_LENGTH = 40.0           # SCM patch X size (m)
TERRAIN_WIDTH = 40.0            # SCM patch Y size (m)
TERRAIN_RES = 0.08              # SCM grid resolution (m) — balance ruts vs cost

# Bekker-Wong / Mohr-Coulomb / Janosi soft-soil parameters (8 required args).
SOIL_BEKKER_KPHI = 0.2e6        # frictional modulus (Pa)
SOIL_BEKKER_KC = 0.0            # cohesive modulus (Pa)
SOIL_BEKKER_N = 1.1             # sinkage exponent (-)
SOIL_MOHR_COHESION = 0.0        # cohesive limit (Pa)
SOIL_MOHR_FRICTION = 30.0       # internal friction angle (deg)
SOIL_JANOSI_SHEAR = 0.01        # shear-displacement coefficient (m)
SOIL_ELASTIC_K = 4.0e7          # vertical elastic stiffness (Pa/m)
SOIL_DAMPING_R = 3.0e4          # vertical damping (Pa*s/m)

ACTIVE_HALF = chrono.ChVector3d(5.0, 3.0, 1.0)  # moving-patch half-extents (m)

TIRE_FAMILY = 1                 # collision family for tire cylinders
CYL_MARGIN = 0.04               # extra radius so SCM ray-casts hit the cylinder (m)
ZTOL = 0.08                     # allowed wheel-bottom clearance vs soil top (m)

# Control schedule (open loop): settle, then accelerate, with a gentle weave.
SETTLE_TIME = 0.5               # brake-hold settle window (s)
DRIVE_THROTTLE = 0.7            # throttle after settle
STEER_AMPLITUDE = 0.15          # gentle sinusoidal steering amplitude (-)
STEER_RATE = 0.4                # steering angular rate (rad/s)

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))          # fast windowless gate


# === Scripted driver (open-loop, headless-safe) ===
class ScriptedDriver(veh.ChDriver):
    """Time-based steering/throttle/braking via the ChDriver Set* setters."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_RATE * time))


def build_simulation():
    """Build vehicle + SCM terrain + driver; return the handles the loop needs."""

    # === Vehicle (HMMWV_Full wrapper owns its ChSystemSMC + all sub-bodies) ===
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity, facing +X

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    # TMEASY tire: a deformable-soil-capable slip/grip model so the chassis
    # actually drives across SCM (a non-deforming tire spins without translating).
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    # Mesh visualization on every vehicle component (chassis, suspension, wheels, tires).
    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()              # ChSystemSMC owned by the wrapper
    chassis = hmmwv.GetChassisBody()        # main chassis rigid body
    veh_obj = hmmwv.GetVehicle()            # underlying ChWheeledVehicle
    # wheels/spindles: veh_obj.GetAxles()[i].m_wheels[side].GetSpindle()
    # joints: suspension + Pitman-arm steering links created inside the wrapper

    # === Terrain (SCM deformable soft soil) ===
    # SetPlotType BEFORE Initialize so the sinkage false-colour overlay is built.
    terrain = veh.SCMTerrain(system)
    terrain.SetSoilParameters(
        SOIL_BEKKER_KPHI,
        SOIL_BEKKER_KC,
        SOIL_BEKKER_N,
        SOIL_MOHR_COHESION,
        SOIL_MOHR_FRICTION,
        SOIL_JANOSI_SHEAR,
        SOIL_ELASTIC_K,
        SOIL_DAMPING_R,
    )
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.10)  # false-colour sinkage
    # Moving patch on the CHASSIS (a level body) — NOT a spinning spindle, whose
    # rotating OOBB would make the deformed range empty (rays=0, no ruts).
    terrain.AddActiveDomain(chassis, chrono.ChVector3d(0, 0, 0), ACTIVE_HALF)
    terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RES)
    terrain.SetMeshWireframe(False)
    terrain.SetTexture(
        chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80
    )

    # === Tire collision cylinders (required for TMEASY on SCM) ===
    # TMEASY tires carry no automatic collision geometry, so SCM ray-casts find
    # nothing and no ruts form. Add a slightly oversized cylinder per spindle.
    tire0 = veh_obj.GetAxles()[0].m_wheels[0].GetTire()
    tire_rad = tire0.GetRadius()
    tire_w = tire0.GetWidth()
    tire_mat = chrono.ChContactMaterialSMC()
    tire_mat.SetFriction(0.9)
    tire_mat.SetRestitution(0.1)
    tire_mat.SetYoungModulus(2.0e7)

    cyl_rot = chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2))
    for axle in veh_obj.GetAxles():
        for iw in range(2):
            spindle = axle.m_wheels[iw].GetSpindle()
            spindle.AddCollisionShape(
                chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + CYL_MARGIN, tire_w),
                cyl_rot,
            )
            spindle.EnableCollision(True)
            sp_cm = spindle.GetCollisionModel()
            sp_cm.SetFamily(TIRE_FAMILY)
            sp_cm.DisallowCollisionsWith(TIRE_FAMILY)   # wheels never collide each other
            # NEVER DisallowCollisionsWith(0): family 0 is SCM's ray-cast query family.
    system.GetCollisionSystem().BindAll()   # rebuild models so ray-casts see cylinders

    # === Footprint assert: wheels start on (not through) the soil ===
    spindle_world = []
    for axle_i in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle_i, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
    assert wheel_bottom_z >= TERRAIN_REST_Z - ZTOL, (
        f"vehicle sinks into soil at spawn: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs soil top z={TERRAIN_REST_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_REST_Z - wheel_bottom_z:.3f} m"
    )

    # === Driver (scripted, open-loop) ===
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    return hmmwv, system, chassis, terrain, driver, tire_rad


def main():
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    hmmwv, system, chassis, terrain, driver, tire_rad = build_simulation()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + chase
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.6)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()                                  # outdoor sky backdrop
        vis.AddCamera(chrono.ChVector3d(INIT_X - 6, -6, 3),
                      chrono.ChVector3d(INIT_X, 0, 0.5))  # AFTER Initialize
        vis.AddTypicalLights()                           # standard lighting
        vis.AddGrid(1.0, 1.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))       # ground reference grid
        vis.AttachVehicle(hmmwv.GetVehicle())
        vis.AttachDriver(driver)                         # steering/throttle HUD bars

    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating

    # Cached handles reused every step (avoid repeated getter calls in the hot loop).
    get_time = system.GetChTime           # cache: bound method, called per step
    veh_obj = hmmwv.GetVehicle()          # cache: fetched once, reused every step

    data_f = None
    motion_f = None
    try:
        data_f = open("simulation_data.csv", "w", newline="")          # guard close
        motion_f = open("cam/motion_log.csv", "w", newline="")
        data_w = csv.writer(data_f)
        motion_w = csv.writer(motion_f)
        data_w.writerow(["time", "chassis_x", "chassis_y", "chassis_z",
                         "speed", "throttle", "steering"])
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        # === Main loop (render-cadence outer; Synchronize/Advance inner) ===
        frame = 0
        keep_running = True
        while keep_running and get_time() < run_end:
            if not HEADLESS:
                if not vis.Run():
                    break
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1

            for _ in range(RENDER_EVERY):
                sim_time = get_time()
                driver_inputs = driver.GetInputs()

                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)     # advances the wrapper-owned ChSystemSMC
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                # --- log physics each step ---
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{speed:.5f}",
                                 f"{driver_inputs.m_throttle:.4f}",
                                 f"{driver_inputs.m_steering:.4f}"])
                motion_w.writerow([f"{sim_time:.5f}", "chassis",
                                   f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                   f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])

                if get_time() >= run_end:
                    keep_running = False
                    break
    except (OSError, IOError) as exc:        # disk / permission while writing CSV
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverges
        if data_f is not None:
            data_f.close()
        if motion_f is not None:
            motion_f.close()

    # === Post-processing: timeseries plot from the logged CSV ===
    try:
        with open("simulation_data.csv", "r", newline="") as f:
            rows = list(csv.reader(f))
    except (OSError, IOError) as exc:         # missing / unreadable CSV
        import traceback
        traceback.print_exc()
        return

    if len(rows) > 1:
        arr = np.array(rows[1:], dtype=float)
        t = arr[:, 0]
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
        ax1.plot(t, arr[:, 1], label="chassis x")
        ax1.plot(t, arr[:, 2], label="chassis y")
        ax1.plot(t, arr[:, 3], label="chassis z")
        ax1.set_ylabel("position (m)")
        ax1.legend(); ax1.grid(True)
        ax2.plot(t, arr[:, 4], label="speed (m/s)", color="tab:red")
        ax2.plot(t, arr[:, 5], label="throttle", color="tab:green")
        ax2.set_xlabel("time (s)"); ax2.set_ylabel("speed / throttle")
        ax2.legend(); ax2.grid(True)
        fig.suptitle("HMMWV on SCM deformable terrain")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
        print(f"final chassis x = {arr[-1, 1]:.3f} m, "
              f"max speed = {arr[:, 4].max():.3f} m/s, rows = {len(rows) - 1}")


if __name__ == "__main__":
    main()
