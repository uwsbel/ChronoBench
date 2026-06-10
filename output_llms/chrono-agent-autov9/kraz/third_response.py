"""Kraz tractor + semitrailer and a sedan on a predefined highway mesh terrain.

Model
-----
- System type: NSC (non-smooth contacts); the Kraz wrapper owns the ChSystem.
- Protagonist: a `veh.Kraz` articulated truck (tractor + semitrailer). The Kraz
  catalog model uses RIGID tires (no slip/grip force model) for both the tractor
  and the trailer axles, which is the desired rigid tire behaviour here.
- Secondary vehicle: a `veh.Sedan` sharing the SAME ChSystem, given its own
  initial pose, its own scripted driver, and a RIGID tire model.
- Terrain: a large flat `veh.RigidTerrain` highway patch (a predefined, paved
  driving surface) textured with the shipped concrete road texture.

Control / expected behaviour
----------------------------
- The truck holds a mild forward cruise (fixed throttle, no steering) so the
  articulated tractor + trailer roll forward and stay upright.
- The sedan is commanded forward with a fixed throttle and a fixed (small,
  constant) steering value by its own driver, so it advances and gently curves.
- Both vehicles should translate forward over the highway surface and remain
  upright (roll/pitch near zero, +X displacement growing with time).

Outputs
-------
- simulation_data.csv : per-step time / speeds / positions for both vehicles.
- cam/motion_log.csv   : per-frame pose of the tractor, the trailer and the sedan.
- frames/img_*.png     : review frames (assembled into cam/review.mp4 downstream).
- simulation_timeseries.png : plotted summary of the logged time series.
"""

import os
import csv
import math

import matplotlib
matplotlib.use("Agg")  # headless plotting backend (no GUI needed)
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants === geometry / physics / control inputs (no bare literals downstream)
TIME_STEP = 2.0e-3            # integration step (s)
TIRE_STEP = 1.0e-3            # tire substep (s)
SIM_END = 12.0               # simulated duration (s)
RENDER_FPS = 30.0            # review-video frame rate

# Truck (Kraz) initial pose — changed location and orientation.
TRUCK_CLEARANCE = 0.05                       # wheel-bottom clearance above terrain
KRAZ_REF_HEIGHT = 0.5588                     # chassis-origin height above ground at rest
TRUCK_INIT_X = -10.0                         # spawn behind the origin so it drives forward
TRUCK_INIT_Y = -2.5                          # right lane
TRUCK_INIT_Z = KRAZ_REF_HEIGHT + TRUCK_CLEARANCE
TRUCK_INIT_YAW = math.radians(10.0)          # changed orientation: slight +Z heading

# Sedan initial pose — its own location and orientation.
SEDAN_REF_HEIGHT = 0.45                      # sedan chassis origin above ground at rest
SEDAN_INIT_X = -6.0
SEDAN_INIT_Y = 2.5                           # left lane, clear of the truck
SEDAN_INIT_Z = SEDAN_REF_HEIGHT + TRUCK_CLEARANCE
SEDAN_INIT_YAW = math.radians(-5.0)          # its own orientation

# Driver command levels.
TRUCK_THROTTLE = 0.35        # mild forward cruise for the truck
SEDAN_THROTTLE = 0.50        # fixed forward throttle for the sedan
SEDAN_STEERING = 0.03        # fixed (constant, gentle) steering for the sedan

# Highway terrain — large flat rigid paved patch (the predefined driving surface).
HIGHWAY_LENGTH = 360.0       # X extent of the highway patch (m)
HIGHWAY_WIDTH = 200.0        # Y extent of the highway patch (m) — wide enough for the sedan arc
HIGHWAY_THICKNESS = 1.0      # patch slab thickness (m)
HIGHWAY_TEXTURE = "terrain/textures/concrete.jpg"
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Derived render cadence — precomputed once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))          # fast windowless validation gate
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short physics check when validating


# === Scripted sedan driver === fixed forward throttle + fixed steering control law
class SedanDriver(veh.ChDriver):
    """Open-loop driver: constant throttle and constant steering, no braking."""

    def __init__(self, vehicle, throttle, steering):
        super().__init__(vehicle)
        self._throttle = throttle      # cache: fixed command, reused every step
        self._steering = steering      # cache: fixed command, reused every step

    def Synchronize(self, time):
        self.SetThrottle(self._throttle)
        self.SetSteering(self._steering)
        self.SetBraking(0.0)


def build_truck():
    """Create + initialize the Kraz tractor/semitrailer; return the wrapper."""
    truck = veh.Kraz()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisCollisionType(veh.CollisionType_NONE)
    truck.SetChassisFixed(False)
    init_rot = chrono.QuatFromAngleZ(TRUCK_INIT_YAW)
    truck.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(TRUCK_INIT_X, TRUCK_INIT_Y, TRUCK_INIT_Z), init_rot
        )
    )
    truck.SetTireStepSize(TIRE_STEP)
    truck.Initialize()
    # Visualization setters take TWO args (tractor, trailer) except steering (one).
    truck.SetChassisVisualizationType(
        chrono.VisualizationType_MESH, chrono.VisualizationType_PRIMITIVES
    )
    truck.SetSuspensionVisualizationType(
        chrono.VisualizationType_PRIMITIVES, chrono.VisualizationType_PRIMITIVES
    )
    truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(
        chrono.VisualizationType_MESH, chrono.VisualizationType_MESH
    )
    truck.SetTireVisualizationType(
        chrono.VisualizationType_MESH, chrono.VisualizationType_MESH
    )
    return truck


def build_sedan(system):
    """Create + initialize the sedan on the SHARED system; return the wrapper."""
    sedan = veh.Sedan(system)  # share the truck-owned ChSystem (no second system)
    sedan.SetChassisCollisionType(veh.CollisionType_NONE)
    sedan.SetChassisFixed(False)
    sedan.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(SEDAN_INIT_X, SEDAN_INIT_Y, SEDAN_INIT_Z),
            chrono.QuatFromAngleZ(SEDAN_INIT_YAW),
        )
    )
    sedan.SetTireType(veh.TireModelType_RIGID)  # prompt: rigid tire model
    sedan.SetTireStepSize(TIRE_STEP)
    sedan.Initialize()
    sedan.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    sedan.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    sedan.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    sedan.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    sedan.SetTireVisualizationType(chrono.VisualizationType_MESH)
    return sedan


def main():
    os.makedirs("frames", exist_ok=True)  # guard against missing output dir
    os.makedirs("cam", exist_ok=True)     # guard against missing output dir

    # === System & bodies (Kraz wrapper owns the ChSystem) ===
    truck = build_truck()
    system = truck.GetSystem()                          # ChSystemNSC owned by the Kraz wrapper
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    tractor_chassis = truck.GetTractorChassisBody()     # cache: tractor chassis rigid body
    trailer_obj = truck.GetTrailer()                    # cache: semitrailer subsystem handle
    trailer_chassis = trailer_obj.GetChassis().GetBody()  # cache: trailer chassis rigid body

    sedan = build_sedan(system)                         # shares the same system
    sedan_chassis = sedan.GetChassisBody()              # cache: sedan chassis rigid body

    # === Terrain === large flat rigid highway patch (paved driving surface)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    terrain = veh.RigidTerrain(system)
    highway_patch = terrain.AddPatch(
        patch_mat, chrono.CSYSNORM,
        HIGHWAY_LENGTH, HIGHWAY_WIDTH, HIGHWAY_THICKNESS,
    )
    highway_patch.SetTexture(veh.GetVehicleDataFile(HIGHWAY_TEXTURE), 72, 40)
    highway_patch.SetColor(chrono.ChColor(0.55, 0.55, 0.55))
    terrain.Initialize()

    # === Drivers === truck cruise (built-in inputs) + scripted sedan driver
    truck_inputs = veh.DriverInputs()
    truck_inputs.m_throttle = TRUCK_THROTTLE
    truck_inputs.m_steering = 0.0
    truck_inputs.m_braking = 0.0
    sedan_driver = SedanDriver(sedan.GetVehicle(), SEDAN_THROTTLE, SEDAN_STEERING)
    sedan_driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Kraz Truck + Sedan on Highway Mesh")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 14.0, 1.0)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(-25, -15, 8), chrono.ChVector3d(0, 0, 0))
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 60, 60,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))
        vis.AttachVehicle(truck.GetTractor())

    # === CSV logging === open both writers before the loop (context-managed)
    data_file = None
    motion_file = None
    try:
        data_file = open("simulation_data.csv", "w", newline="")          # disk/permission guarded below
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:  # disk full / permission denied opening outputs
        print(f"Failed to open CSV outputs: {exc}")
        raise

    times, truck_x, sedan_x = [], [], []
    truck_speed_series, sedan_speed_series = [], []

    try:
        data_writer = csv.writer(data_file)
        data_writer.writerow([
            "time",
            "truck_x", "truck_y", "truck_z", "truck_speed",
            "trailer_x", "trailer_y", "trailer_z",
            "sedan_x", "sedan_y", "sedan_z", "sedan_speed",
        ])
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(["time", "body", "x", "y", "z", "qw", "qx", "qy", "qz"])

        # === Main loop === render-cadence outer loop, physics in inner batch
        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()

                # --- log physics every step ---
                t_pos = tractor_chassis.GetPos()
                tr_pos = trailer_chassis.GetPos()
                s_pos = sedan_chassis.GetPos()
                t_speed = truck.GetTractor().GetSpeed()
                s_speed = sedan.GetVehicle().GetSpeed()
                data_writer.writerow([
                    f"{time:.5f}",
                    f"{t_pos.x:.5f}", f"{t_pos.y:.5f}", f"{t_pos.z:.5f}", f"{t_speed:.5f}",
                    f"{tr_pos.x:.5f}", f"{tr_pos.y:.5f}", f"{tr_pos.z:.5f}",
                    f"{s_pos.x:.5f}", f"{s_pos.y:.5f}", f"{s_pos.z:.5f}", f"{s_speed:.5f}",
                ])
                for name, body in (
                    ("tractor", tractor_chassis),
                    ("trailer", trailer_chassis),
                    ("sedan", sedan_chassis),
                ):
                    p = body.GetPos()
                    q = body.GetRot()
                    motion_writer.writerow([
                        f"{time:.5f}", name,
                        f"{p.x:.5f}", f"{p.y:.5f}", f"{p.z:.5f}",
                        f"{q.e0:.6f}", f"{q.e1:.6f}", f"{q.e2:.6f}", f"{q.e3:.6f}",
                    ])
                times.append(time)
                truck_x.append(t_pos.x)
                sedan_x.append(s_pos.x)
                truck_speed_series.append(t_speed)
                sedan_speed_series.append(s_speed)

                # --- synchronize subsystems, then advance one step ---
                sedan_inputs = sedan_driver.GetInputs()
                sedan_driver.Synchronize(time)
                terrain.Synchronize(time)
                truck.Synchronize(time, truck_inputs, terrain)
                sedan.Synchronize(time, sedan_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(time, truck_inputs)

                sedan_driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                truck.Advance(TIME_STEP)   # advances the shared wrapper-owned system
                sedan.Advance(TIME_STEP)
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= RUN_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state mid-run
        import traceback
        traceback.print_exc()
        print(f"Simulation aborted: {exc}")
        raise
    finally:
        # flush + close partial CSV even if a step diverged
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot the logged time series
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    ax1.plot(times, truck_x, label="truck x")
    ax1.plot(times, sedan_x, label="sedan x")
    ax1.set_ylabel("forward position (m)")
    ax1.legend()
    ax1.grid(True)
    ax2.plot(times, truck_speed_series, label="truck speed")
    ax2.plot(times, sedan_speed_series, label="sedan speed")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("speed (m/s)")
    ax2.legend()
    ax2.grid(True)
    fig.suptitle("Kraz truck + sedan on highway mesh")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

    print(f"Done. frames={'(headless)' if HEADLESS else 'written'} "
          f"truck_dx={truck_x[-1] - truck_x[0]:.3f} m "
          f"sedan_dx={sedan_x[-1] - sedan_x[0]:.3f} m")


if __name__ == "__main__":
    main()
