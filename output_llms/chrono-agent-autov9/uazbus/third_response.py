"""UAZBUS wheeled-vehicle mobility test on flat rigid terrain (PyChrono 9.0.1, Irrlicht).

Model
-----
- A UAZBUS catalog wheeled vehicle (NSC contact, internally owned ChSystemNSC)
  driving forward under a constant 0.5 throttle command.
- Tire model: RIGID (TireModelType_RIGID) — a rigid contact tire, chosen so the
  wheels grip the rigid road directly.
- A flat RigidTerrain patch large enough for the vehicle to translate across.
- A fixed box obstacle (full extents 0.5 x 5.0 x 0.2 m) standing across the path
  at world (5, 0, 0.1) to probe the vehicle's mobility against an obstruction.

Expected behavior
------------------
The bus accelerates forward (+X) from rest under constant throttle, rolls along
the rigid road, and interacts with the fixed box barrier in its path. The chassis
should translate a meaningful distance in +X and remain upright (no rollover).
CSV logs capture chassis pose / speed each step for physics verification.
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

# === Named constants (geometry / physics) ===
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # tire substep (s)
SIM_END = 8.0                      # simulated duration (s)
RENDER_FPS = 30.0                  # review-video frame rate
THROTTLE = 0.5                     # constant forward throttle command (prompt)

TERRAIN_LENGTH = 100.0             # rigid road extent in X (m)
TERRAIN_WIDTH = 100.0             # rigid road extent in Y (m)
TERRAIN_FRICTION = 0.9             # road friction coefficient
TERRAIN_RESTITUTION = 0.01         # road restitution

# Fixed box obstacle (prompt: dims 0.5,5,0.2 at (5,0,0.1), fixed)
BOX_SIZE = chrono.ChVector3d(0.5, 5.0, 0.2)
BOX_POS = chrono.ChVector3d(5.0, 0.0, 0.1)
BOX_DENSITY = 1000.0

# Vehicle spawn — start at the origin facing +X, slightly above the road so the
# wheels settle onto the patch rather than spawning interpenetrating.
VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = 0.45                  # chassis-origin height above flat road at rest
ROAD_TOP_Z = 0.0                   # flat RigidTerrain patch top is at z=0

# Derived constants (precomputed once)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating

os.makedirs("frames", exist_ok=True)   # guard against missing output dir
os.makedirs("cam", exist_ok=True)       # review-video frame/CSV destination

data_writer = None
data_file = None
motion_file = None
motion_writer = None

try:
    # === Vehicle (UAZBUS wrapper creates + owns its ChSystemNSC) ===
    init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity: chassis faces +X

    bus = veh.UAZBUS()
    bus.SetContactMethod(chrono.ChContactMethod_NSC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    bus.SetTireType(veh.TireModelType_RIGID)   # prompt: rigid tire model
    bus.SetTireStepSize(TIRE_STEP)
    bus.Initialize()

    bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.UAZBUS wrapper) ===
    sys = bus.GetSystem()                 # cache: ChSystemNSC owned by the wrapper
    chassis = bus.GetChassisBody()        # cache: main chassis rigid body, reused every step
    veh_obj = bus.GetVehicle()            # cache: ChWheeledVehicle handle for spindle queries
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension +
    # steering links created internally by the wrapper; terrain patch added below.

    # === Terrain (flat rigid road) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Footprint check after Initialize (flat road: wheel-bottom vs road top) ===
    TIRE_RADIUS = 0.42   # UAZBUS tire radius (m), used only for the resting check
    ZTOL = 0.10          # allowed wheel-bottom overlap/clearance vs road top
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= ROAD_TOP_Z - ZTOL, (
        f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs road top z={ROAD_TOP_Z:.3f}; raise VEH_INIT_Z by "
        f"{ROAD_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Box obstacle (fixed barrier across the path) ===
    box_mat = chrono.ChContactMaterialNSC()
    box_mat.SetFriction(TERRAIN_FRICTION)
    box_mat.SetRestitution(TERRAIN_RESTITUTION)
    box = chrono.ChBodyEasyBox(
        BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, BOX_DENSITY, True, True, box_mat
    )
    box.SetPos(BOX_POS)
    box.SetFixed(True)             # prompt: fixed in place
    box.SetName("box_obstacle")
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.7, 0.2, 0.2))
    sys.Add(box)
    sys.GetCollisionSystem().BindAll()   # rebuild collision models after post-init add

    # No spawn overlap: front of vehicle near x=0, box at x=5 -> AABBs clear.
    assert BOX_POS.x - BOX_SIZE.x / 2.0 > VEH_INIT_X + 2.5, (
        "box overlaps the vehicle spawn footprint; move the obstacle in +X"
    )

    # === Driver (constant forward throttle) ===
    class ConstantThrottleDriver(veh.ChDriver):
        """Open-loop driver: constant throttle, zero steering / braking."""

        def __init__(self, vehicle, throttle):
            super().__init__(vehicle)
            self._throttle = throttle   # cache: fixed command, reused every Synchronize

        def Synchronize(self, time):
            self.SetThrottle(self._throttle)
            self.SetSteering(0.0)
            self.SetBraking(0.0)

    driver = ConstantThrottleDriver(veh_obj, THROTTLE)
    driver.Initialize()

    # === Visualization (full Irrlicht vehicle scene: window + sky + camera + lights + grid) ===
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("UAZBUS mobility test (rigid tires)")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.6)
        vis.Initialize()                                                  # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()                                                   # sky backdrop
        vis.AddTypicalLights()                                            # standard lights
        vis.AddGrid(1.0, 1.0, 100, 100,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))                        # ground reference grid
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Logging setup (open CSV writers with context managers) ===
    data_file = open("simulation_data.csv", "w", newline="")
    motion_file = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
    data_writer = csv.writer(data_file)
    motion_writer = csv.writer(motion_file)
    data_writer.writerow(["time", "x", "y", "z", "speed", "roll_deg", "throttle"])
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    # === Main loop (render-cadence outer loop; vehicle Synchronize/Advance inner) ===
    frame = 0
    step = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
            frame += 1

        for _ in range(RENDER_EVERY):
            time = sys.GetChTime()
            driver_inputs = driver.GetInputs()

            # Log chassis pose / speed each physics step.
            pos = chassis.GetPos()
            vel = chassis.GetPosDt()
            speed = veh_obj.GetSpeed()
            rot = chassis.GetRot()
            roll_deg = math.degrees(rot.GetCardanAnglesXYZ().x)
            data_writer.writerow([f"{time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                  f"{pos.z:.5f}", f"{speed:.5f}", f"{roll_deg:.5f}",
                                  f"{driver_inputs.m_throttle:.3f}"])
            motion_writer.writerow([f"{time:.5f}", "chassis", f"{pos.x:.5f}",
                                    f"{pos.y:.5f}", f"{pos.z:.5f}", f"{vel.x:.5f}",
                                    f"{vel.y:.5f}", f"{vel.z:.5f}"])

            driver.Synchronize(time)
            terrain.Synchronize(time)
            bus.Synchronize(time, driver_inputs, terrain)
            if not HEADLESS:
                vis.Synchronize(time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            bus.Advance(TIME_STEP)        # advances the wrapper-owned system
            if not HEADLESS:
                vis.Advance(TIME_STEP)
            step += 1
            if sys.GetChTime() >= RUN_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:           # disk / permission failure on CSV I/O
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close partial CSV even if a step diverges mid-run.
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing (plot logged time series) ===
try:
    t, x, y, z, spd, roll = [], [], [], [], [], []
    with open("simulation_data.csv", "r", newline="") as f:
        reader = csv.reader(f)
        next(reader, None)   # skip header
        for row in reader:
            if not row:
                continue
            t.append(float(row[0])); x.append(float(row[1]))
            y.append(float(row[2])); z.append(float(row[3]))
            spd.append(float(row[4])); roll.append(float(row[5]))

    if t:
        t = np.array(t)
        fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axs[0].plot(t, x, label="x"); axs[0].plot(t, y, label="y"); axs[0].plot(t, z, label="z")
        axs[0].set_ylabel("position (m)"); axs[0].legend(); axs[0].grid(True)
        axs[1].plot(t, spd, color="tab:green"); axs[1].set_ylabel("speed (m/s)"); axs[1].grid(True)
        axs[2].plot(t, roll, color="tab:red"); axs[2].set_ylabel("roll (deg)")
        axs[2].set_xlabel("time (s)"); axs[2].grid(True)
        fig.suptitle("UAZBUS mobility test — constant throttle on rigid terrain")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)
except (OSError, IOError, ValueError) as exc:   # missing CSV / parse failure
    import traceback
    traceback.print_exc()
