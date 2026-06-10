"""CityBus on a flat rigid dirt road — straight-line accelerate then gentle turn.

Model
-----
A full CityBus wheeled-vehicle wrapper (veh.CityBus) driving on a large flat
RigidTerrain patch textured as a dirt road. The wrapper owns its ChSystemSMC
(SMC contact). The bus uses the Pacejka 1989 tire force model (Pac89Tire),
attached explicitly to every wheel, integrated at a fine tire step size for
numerical stability. A scripted ChDriver subclass ramps the throttle up and
applies a steady steering input so the bus accelerates from rest and then turns,
which is why a large terrain patch is used (the bus sweeps a curved path).

System type: SMC (penalty contact), gravity -Z, Z-up world.
Main bodies: bus chassis + 6 wheels/spindles on 3 axles (created by the wrapper),
one fixed RigidTerrain patch body (the dirt road).
Expected behavior: the bus starts at rest, accelerates forward under throttle,
then follows a left curve while staying upright on the dirt patch.
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

# === Named constants (geometry / physics) — no bare literals downstream ===
STEP_SIZE = 5e-4              # simulation integration step (s)
TIRE_STEP_SIZE = 5e-4        # tire force-model step (s) — matched for stability
SIM_END = 12.0               # total simulated time (s)
RENDER_FPS = 30.0            # review-video frame rate

TERRAIN_LENGTH = 200.0       # X extent of the dirt road patch (m) — large: the bus turns
TERRAIN_WIDTH = 200.0        # Y extent of the dirt road patch (m)
TERRAIN_FRICTION = 0.9       # tire-road friction coefficient
TERRAIN_RESTITUTION = 0.01   # near-inelastic road contact
TERRAIN_YOUNG = 2e7          # SMC contact stiffness for the road (Pa)

INIT_X = -90.0               # spawn near one end so the bus has room to drive
INIT_Y = 0.0
INIT_Z = 0.0                 # chassis-origin height: wheel bottoms rest just on z=0
TIRE_RADIUS = 0.464          # Pac89 tire radius (m) for the support assert
Z_TOL = 0.20                 # allowed wheel-bottom clearance vs road top (m)

THROTTLE_RAMP_END = 3.0      # seconds over which throttle ramps 0 -> cruise
CRUISE_THROTTLE = 0.6        # steady throttle after the ramp
STEER_START = 4.0            # start steering (s) after reaching speed
STEER_VALUE = 0.35           # steady steering command (-1..+1), gentle left turn

# === Derived constants (precomputed once, never recomputed in the loop) ===
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))   # precomputed once
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                    # identity, +X heading

# Headless validation gate: skip the window + run a short bounded physics check.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run


# === Scripted driver (open-loop accelerate-then-turn control law) ===
class BusDriver(veh.ChDriver):
    """Ramp throttle to cruise, hold, then apply a steady left steering input."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Throttle: linear ramp 0 -> CRUISE_THROTTLE, then hold.
        if time < THROTTLE_RAMP_END:
            self.SetThrottle(CRUISE_THROTTLE * (time / THROTTLE_RAMP_END))
        else:
            self.SetThrottle(CRUISE_THROTTLE)
        self.SetBraking(0.0)
        # Steering: straight until STEER_START, then a steady gentle left turn.
        self.SetSteering(STEER_VALUE if time >= STEER_START else 0.0)


def main():
    # === Vehicle wrapper + system & bodies (created by the veh.CityBus wrapper) ===
    # The CityBus wrapper creates and OWNS its ChSystemSMC plus the chassis,
    # six wheels/spindles (3 axles), suspension + steering joints, engine and
    # driveline. The tire model is NOT set via SetTireType here: the CityBus
    # catalog only ships Rigid/TMeasy templates and silently substitutes TMeasy
    # for any Pacejka request, so the Pacejka 1989 tires are attached explicitly
    # below with InitializeTire after Initialize().
    bus = veh.CityBus()
    bus.SetContactMethod(chrono.ChContactMethod_SMC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    bus.Initialize()

    sys = bus.GetSystem()                 # ChSystemSMC owned by the wrapper
    chassis = bus.GetChassisBody()        # cache: main chassis rigid body, reused every step
    veh_obj = bus.GetVehicle()            # cache: ChWheeledVehicle handle, reused every step
    # spindles/wheels: veh_obj.GetSpindlePos(axle, side); joints: suspension +
    # steering links live inside the wrapper; terrain patch body added below.

    # === Tires (Pacejka 1989 force model, attached to every wheel) ===
    # Pac89Tire is the Pacejka '89 magic-formula tire; one fresh instance per
    # wheel (a tire object binds to a single wheel).
    for ia, axle in enumerate(veh_obj.GetAxles()):
        for iw, wheel in enumerate(axle.GetWheels()):
            tire = veh.HMMWV_Pac89Tire(f"pac89_tire_{ia}_{iw}")  # Pacejka 89 magic-formula tire
            tire.SetStepsize(TIRE_STEP_SIZE)                     # fine tire step for stability
            veh_obj.InitializeTire(tire, wheel, chrono.VisualizationType_MESH)

    bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # Assert the requested tire model actually took effect (no silent fallback).
    tire0 = veh_obj.GetAxles()[0].GetWheels()[0].GetTire()
    assert tire0.GetTemplateName() == "Pac89Tire", (
        f"expected Pac89Tire, got {tire0.GetTemplateName()}"
    )

    # === Terrain (flat rigid dirt road) ===
    # A single large flat patch so the bus can accelerate and sweep a curve.
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch_mat.SetYoungModulus(TERRAIN_YOUNG)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.7, 0.6, 0.45))
    terrain.Initialize()
    road_top_z = 0.0                      # flat patch sits at z=0

    # === Footprint assert (wheels start resting on the road, not above/through it) ===
    # Read real spindle Z after Initialize; the wheel bottom must sit within
    # Z_TOL of the road top so the bus neither floats (hard drop -> tire-model
    # divergence) nor spawns through the surface.
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert road_top_z - Z_TOL <= wheel_bottom_z <= road_top_z + Z_TOL, (
        f"bus not seated on road: wheel bottom z={wheel_bottom_z:.3f} vs road top "
        f"z={road_top_z:.3f}; nudge INIT_Z by {road_top_z - wheel_bottom_z:.3f} m"
    )

    # === Driver (scripted accelerate-then-turn) ===
    driver = BusDriver(veh_obj)
    driver.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    # Guarded by the headless gate so validation runs windowless; the committed
    # block is complete so the source reads as a full visualization setup.
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("CityBus on dirt road — PAC89 tires")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 14.0, 1.0)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddTypicalLights()
        vis.AddGrid(2.0, 2.0, 50, 50,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.45, 0.42, 0.38))   # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Output dirs + main loop ===
    os.makedirs("frames", exist_ok=True)   # guard against missing review-frame dir
    os.makedirs("cam", exist_ok=True)      # guard against missing motion-log dir

    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

    data_file = None
    motion_file = None
    times, speeds, xs, ys, zs = [], [], [], [], []
    try:
        # Open both CSV writers with context managers so they always flush/close.
        with open("simulation_data.csv", "w", newline="") as data_file, \
             open("cam/motion_log.csv", "w", newline="") as motion_file:
            data_writer = csv.writer(data_file)
            data_writer.writerow(
                ["time", "x", "y", "z", "speed", "throttle", "steering", "roll_deg"]
            )
            motion_writer = csv.writer(motion_file)
            motion_writer.writerow(
                ["time", "body", "x", "y", "z", "vx", "vy", "vz"]
            )

            frame = 0
            step = 0
            while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
                if not HEADLESS:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()
                    vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                    frame += 1

                for _ in range(render_every):
                    sim_time = sys.GetChTime()
                    driver_inputs = driver.GetInputs()

                    driver.Synchronize(sim_time)
                    terrain.Synchronize(sim_time)
                    bus.Synchronize(sim_time, driver_inputs, terrain)
                    if not HEADLESS:
                        vis.Synchronize(sim_time, driver_inputs)

                    driver.Advance(STEP_SIZE)
                    terrain.Advance(STEP_SIZE)
                    bus.Advance(STEP_SIZE)        # advances the wrapper-owned system
                    if not HEADLESS:
                        vis.Advance(STEP_SIZE)

                    # --- log physics every step ---
                    pos = chassis.GetPos()
                    vel = chassis.GetPosDt()
                    speed = veh_obj.GetSpeed()
                    # roll angle from chassis rotation (rotation about forward X axis)
                    rot = chassis.GetRot()
                    roll = math.atan2(
                        2.0 * (rot.e0 * rot.e1 + rot.e2 * rot.e3),
                        1.0 - 2.0 * (rot.e1 * rot.e1 + rot.e2 * rot.e2),
                    )
                    roll_deg = math.degrees(roll)
                    data_writer.writerow(
                        [f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                         f"{pos.z:.5f}", f"{speed:.5f}",
                         f"{driver_inputs.m_throttle:.4f}",
                         f"{driver_inputs.m_steering:.4f}", f"{roll_deg:.4f}"]
                    )
                    motion_writer.writerow(
                        [f"{sim_time:.5f}", "chassis", f"{pos.x:.5f}", f"{pos.y:.5f}",
                         f"{pos.z:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"]
                    )
                    times.append(sim_time)
                    speeds.append(speed)
                    xs.append(pos.x)
                    ys.append(pos.y)
                    zs.append(pos.z)

                    step += 1
                    if sys.GetChTime() >= run_end:
                        break
    except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:                # disk / permission on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Context managers above already flushed/closed the CSV writers; nothing
        # to close here, but report how much motion data was captured.
        print(f"logged {len(times)} samples; final time "
              f"{times[-1] if times else 0.0:.3f} s")

    # === Post-processing (timeseries plot from the captured data) ===
    if times:
        t = np.array(times)
        fig, axes = plt.subplots(2, 2, figsize=(11, 7))
        axes[0, 0].plot(t, speeds, "b-")
        axes[0, 0].set_xlabel("time (s)"); axes[0, 0].set_ylabel("speed (m/s)")
        axes[0, 0].set_title("Bus forward speed"); axes[0, 0].grid(True)

        axes[0, 1].plot(xs, ys, "g-")
        axes[0, 1].set_xlabel("x (m)"); axes[0, 1].set_ylabel("y (m)")
        axes[0, 1].set_title("Ground-plane trajectory"); axes[0, 1].grid(True)
        axes[0, 1].axis("equal")

        axes[1, 0].plot(t, zs, "m-")
        axes[1, 0].set_xlabel("time (s)"); axes[1, 0].set_ylabel("chassis z (m)")
        axes[1, 0].set_title("Chassis height (upright check)"); axes[1, 0].grid(True)

        axes[1, 1].plot(t, xs, "c-", label="x")
        axes[1, 1].plot(t, ys, "r-", label="y")
        axes[1, 1].set_xlabel("time (s)"); axes[1, 1].set_ylabel("position (m)")
        axes[1, 1].set_title("Chassis x / y vs time"); axes[1, 1].legend(); axes[1, 1].grid(True)

        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)


if __name__ == "__main__":
    # SetVehicleDataPath so veh.GetVehicleDataFile resolves the dirt texture.
    veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")
    main()
