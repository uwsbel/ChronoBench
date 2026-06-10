"""HMMWV on deformable SCM (Bekker-Wong) soft soil with scene boxes and a chassis-mounted camera sensor.

Model
-----
- An HMMWV_Full wheeled vehicle (SMC contact, TMEASY tires) drives forward across a
  deformable SCM terrain patch, sinking into the soil and leaving ruts.
- Randomly positioned ChBodyEasyBox obstacles are scattered on the terrain. Each box
  is rejection-sampled so it is NOT initialized inside the vehicle footprint.
- A sensor system (ChSensorManager) is added with several point lights and a single
  RGB camera sensor rigidly attached to the vehicle chassis. The camera carries a
  ChFilterVisualize filter so its feed is shown live during the simulation, plus a
  ChFilterSave (PNG frames) and a ChFilterRGBA8Access (frame-buffer read).

System type
-----------
SMC (penalty) contact, owned by the HMMWV_Full wrapper. Z-up world.

Expected behavior
-----------------
The chassis translates forward (positive X) on the soft soil, the wheels leave
visible ruts, the scattered boxes remain in view, and the onboard chassis camera
records the forward driving view. Logged CSV columns (chassis pose / speed) show a
monotonically increasing X position.
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
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants: geometry / physics / scene ===
TIME_STEP = 2e-3                    # integration step (s)
TIRE_STEP = 1e-3                    # TMEASY tire substep (s) — required on SCM
SIM_END = 8.0                       # modest sim duration (s)
RENDER_FPS = 30.0                   # review-video frame rate

GRAVITY = chrono.ChVector3d(0, 0, -9.81)

# Terrain (deformable SCM) — firm soil so the vehicle drives without bogging down.
SCM_LENGTH = 40.0                   # terrain X extent (m)
SCM_WIDTH = 20.0                    # terrain Y extent (m)
SCM_RES = 0.08                      # grid resolution (m)
SCM_REST_Z = 0.0                    # SCM rest plane height (m)
BEKKER_KPHI = 2.0e6                 # frictional modulus (Pa) — firm soil
BEKKER_KC = 0.0                     # cohesive modulus
BEKKER_N = 1.1                      # sinkage exponent
MOHR_COHESION = 5.0e3               # cohesive limit (Pa)
MOHR_FRICTION = 30.0                # internal friction angle (deg)
JANOSI_SHEAR = 0.01                 # shear deformation modulus (m)
ELASTIC_K = 2.0e8                   # elastic stiffness (Pa/m)
DAMPING_R = 3.0e4                   # vertical damping (Pa.s/m)

# Vehicle spawn — origin near the -X end so it has room to drive forward in +X.
VEH_INIT_X = -12.0
VEH_INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5         # HMMWV chassis origin above wheel-bottom at rest
ZTOL = 0.10                         # allowed wheel-bottom clearance/overlap vs soil top

# Collision families (keep tires from filtering SCM ray-casts; never disallow family 0).
TIRE_FAMILY = 1

# Scene boxes — randomly positioned obstacles that must NOT spawn inside the vehicle.
NUM_BOXES = 8
BOX_SIZE = 0.5                      # cube edge length (m)
BOX_DENSITY = 50.0                  # light so a contact does not wreck the run
BOX_AREA_X = (-6.0, 14.0)          # sampling band in X (ahead of / around the vehicle)
BOX_AREA_Y = (-7.0, 7.0)           # sampling band in Y
VEH_CLEAR_RADIUS = 4.0              # keep box centers this far from the spawn (m)
RANDOM_SEED = 7                     # deterministic placement

# Onboard camera sensor (rides on the chassis, looks forward).
CAM_W = 1280
CAM_H = 720
CAM_FOV = 1.408                     # horizontal FOV (rad)
CAM_OFFSET = chrono.ChVector3d(0.8, 0.0, 1.4)   # local pose on chassis (forward+up)


def main():
    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    # The wrapper internally creates the ChSystemSMC, the chassis rigid body, four
    # spindle/wheel bodies, and the suspension + steering links. We enumerate the
    # real handles below so the system + bodies are explicit.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, SCM_REST_Z + SUSPENSION_REF_HEIGHT),
            chrono.QUNIT,
        )
    )
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)   # SCM requires a slip-curve tire, not RIGID
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

    system = hmmwv.GetSystem()              # ChSystemSMC owned by the wrapper
    system.SetGravitationalAcceleration(GRAVITY)
    veh_obj = hmmwv.GetVehicle()            # cache: vehicle handle, reused every step
    chassis = hmmwv.GetChassisBody()        # cache: main chassis rigid body, reused every step
    # spindles: veh_obj.GetAxles()[i].m_wheels[j].GetSpindle(); links: suspension/steering inside wrapper

    # === Terrain (deformable SCM soft soil) ===
    # SCMTerrain needs the wrapper's collision system, which Initialize() already set up.
    terrain = veh.SCMTerrain(system)
    terrain.SetSoilParameters(
        BEKKER_KPHI, BEKKER_KC, BEKKER_N,
        MOHR_COHESION, MOHR_FRICTION, JANOSI_SHEAR,
        ELASTIC_K, DAMPING_R,
    )
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # colored sinkage overlay
    terrain.SetMeshWireframe(False)
    terrain.Initialize(SCM_LENGTH, SCM_WIDTH, SCM_RES)
    terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)

    # === Tire collision cylinders (REQUIRED for TMEASY tires on SCM) ===
    # SCM detects tire contact via ray-casts against collision shapes. TMEASY tires
    # carry no collision geometry, so add a slightly oversized cylinder per spindle.
    tire0 = veh_obj.GetAxles()[0].m_wheels[0].GetTire()
    tire_rad = tire0.GetRadius()            # precomputed once
    tire_w = tire0.GetWidth()               # precomputed once
    tire_mat = chrono.ChContactMaterialSMC()
    tire_mat.SetFriction(0.9)
    tire_mat.SetRestitution(0.1)
    tire_mat.SetYoungModulus(1e7)

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
            sp_cm.DisallowCollisionsWith(TIRE_FAMILY)   # tires never contact each other
    system.GetCollisionSystem().BindAll()   # rebuild collision models so ray-casts see cylinders

    # === Validate footprint after Initialize (wheel bottoms rest on the soil) ===
    spindle_world = [
        veh_obj.GetSpindlePos(a, side)
        for a in range(veh_obj.GetNumberAxles())
        for side in (veh.LEFT, veh.RIGHT)
    ]
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
    assert wheel_bottom_z >= SCM_REST_Z - ZTOL, (
        f"vehicle sinks into soil: wheel bottom z={wheel_bottom_z:.3f} vs soil top "
        f"z={SCM_REST_Z:.3f}; raise SUSPENSION_REF_HEIGHT by {SCM_REST_Z - wheel_bottom_z:.3f} m"
    )

    # === Scene boxes (randomly positioned, none inside the vehicle) ===
    # Rejection-sample box centers so each stays clear of the vehicle spawn.
    rng = np.random.default_rng(RANDOM_SEED)
    box_mat = chrono.ChContactMaterialSMC()
    box_mat.SetFriction(0.8)
    box_mat.SetRestitution(0.0)
    box_mat.SetYoungModulus(1e7)

    placed = 0
    attempts = 0
    while placed < NUM_BOXES and attempts < NUM_BOXES * 50:
        attempts += 1
        bx = float(rng.uniform(*BOX_AREA_X))
        by = float(rng.uniform(*BOX_AREA_Y))
        # Reject any candidate that lands within the vehicle clearance radius.
        if math.hypot(bx - VEH_INIT_X, by - VEH_INIT_Y) < VEH_CLEAR_RADIUS:
            continue
        box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY, True, True, box_mat)
        box.SetPos(chrono.ChVector3d(bx, by, SCM_REST_Z + BOX_SIZE / 2.0))
        box.SetName(f"scene_box_{placed}")
        system.AddBody(box)
        placed += 1
    assert placed == NUM_BOXES, f"only placed {placed}/{NUM_BOXES} boxes"

    # === Driver (scripted forward-driving control) ===
    class ForwardDriver(veh.ChDriver):
        def __init__(self, vehicle):
            super().__init__(vehicle)

        def Synchronize(self, time):
            # Brief settle, then a steady forward throttle with no steering.
            if time < 0.5:
                self.SetThrottle(0.0)
                self.SetBraking(1.0)
            else:
                self.SetThrottle(0.6)
                self.SetBraking(0.0)
            self.SetSteering(0.0)

    driver = ForwardDriver(veh_obj)
    driver.Initialize()

    # === Sensor system (manager + point lights + chassis camera) ===
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 60), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(-20, 20, 60), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(0, -20, 60), chrono.ChColor(0.9, 0.9, 0.9), 500.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

    # Camera rigidly mounted on the chassis, looking forward (+X local).
    cam = sens.ChCameraSensor(
        chassis,                                        # rides on the chassis body
        1.0 / TIME_STEP,                                # update rate (Hz)
        chrono.ChFramed(CAM_OFFSET, chrono.QUNIT),      # local offset pose on chassis
        CAM_W, CAM_H, CAM_FOV,
    )
    cam.SetName("chassis_cam")
    cam.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H))   # visualize the camera feed live
    cam.PushFilter(sens.ChFilterSave("cam/chassis_cam/"))  # PNG frames -> mp4 by RUN stage
    cam.PushFilter(sens.ChFilterRGBA8Access())             # frame-buffer access
    manager.AddSensor(cam)

    # === Visualization (full Irrlicht vehicle scene) ===
    HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation run
    vis = None
    if not HEADLESS:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV on SCM soft soil with scene boxes and chassis camera")
        vis.SetWindowSize(1280, 720)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.6)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 6, -8, 4),
                      chrono.ChVector3d(VEH_INIT_X, 0, 0.5))
        vis.AddTypicalLights()
        vis.AddGrid(1.0, 1.0, int(SCM_LENGTH), int(SCM_WIDTH),
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))
        vis.AttachVehicle(veh_obj)
        vis.AttachDriver(driver)

    # === Main loop (render-cadence outer loop; physics in inner batch) ===
    render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating
    os.makedirs("frames", exist_ok=True)                            # guard against missing output dir

    data_file = None
    motion_file = None
    times, xs, ys, speeds = [], [], [], []
    frame = 0
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_w = csv.writer(data_file)
        motion_w = csv.writer(motion_file)
        data_w.writerow(["time", "chassis_x", "chassis_y", "chassis_z", "speed", "throttle"])
        motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1
            for _ in range(render_every):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                hmmwv.Synchronize(sim_time, driver_inputs, terrain)
                if not HEADLESS:
                    vis.Synchronize(sim_time, driver_inputs)

                manager.Update()    # pump sensors every physics step

                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = veh_obj.GetSpeed()
                data_w.writerow([f"{sim_time:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                 f"{pos.z:.5f}", f"{speed:.5f}", f"{driver_inputs.m_throttle:.3f}"])
                motion_w.writerow([f"{sim_time:.5f}", "chassis", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                   f"{pos.z:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])
                times.append(sim_time)
                xs.append(pos.x)
                ys.append(pos.y)
                speeds.append(speed)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned system
                if not HEADLESS:
                    vis.Advance(TIME_STEP)

                if system.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state mid-run
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:           # disk / permission errors on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush partial CSV even if a step diverged.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing (timeseries plot) ===
    if times:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        ax1.plot(times, xs, label="chassis x")
        ax1.plot(times, ys, label="chassis y")
        ax1.set_ylabel("position (m)")
        ax1.legend()
        ax1.grid(True)
        ax2.plot(times, speeds, color="tab:red", label="speed")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("speed (m/s)")
        ax2.legend()
        ax2.grid(True)
        fig.suptitle("HMMWV on SCM soft soil — chassis motion")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

        x_travel = xs[-1] - xs[0]
        print(f"[result] frames={frame} steps={len(times)} "
              f"x_travel={x_travel:.3f} m final_speed={speeds[-1]:.3f} m/s")


if __name__ == "__main__":
    os.makedirs("cam", exist_ok=True)   # guard: ensure cam/ exists for CSV + sensor frames
    main()
