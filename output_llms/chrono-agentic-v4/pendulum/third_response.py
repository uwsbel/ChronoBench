"""
Double pendulum simulation — pure MBS with two independent pendulum arms.

System: ChSystemNSC (non-smooth contact, no collision shapes).
Bodies: ground anchor, arm1, bob1, arm2, bob2.
Joints:  arm1<->ground revolute,  arm2<->bob1 revolute.
Both pendulums swing freely under gravity and can move independently.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Review-only: sim_recording import and REC flag ===
import sim_recording as rec  # review-only
REC = bool(os.environ.get("SIMBENCH_RECORD"))  # review-only

# === Named constants ===
L1 = 0.8          # length of arm1 [m]
L2 = 0.6          # length of arm2 [m]
R = 0.12          # bob radius [m]
density = 1800.0  # kg/m^3 (plastic-like)
time_step = 1e-3  # s
sim_end = 8.0     # s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===
# Ground anchor (fixed pivot post)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground_vis = chrono.ChVisualShapeCylinder(0.04, 0.3)
ground_vis.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(ground_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))
sys.AddBody(ground)

# Arm1: slender rod, body origin at its center.
# Body-local X points along the arm toward bob1 (world -Y).
# Cylinder axis is body-local Z; rotate by -90 deg around Z so Z→(-Y).
arm1 = chrono.ChBody()
arm1.SetMass(0.5)
arm1.SetInertiaXX(chrono.ChVector3d(0.005, 0.005, 0.005))
arm1.SetPos(chrono.ChVector3d(0, -L1 / 2, 0))
arm1.SetRot(chrono.QuatFromAngleZ(-chrono.CH_PI_2))  # body-local X → world -Y
arm1_vis = chrono.ChVisualShapeCylinder(R * 0.3, L1)
arm1_vis.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
arm1.AddVisualShape(arm1_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.AddBody(arm1)

# Bob1: sphere at end of arm1
bob1 = chrono.ChBodyEasySphere(R, density, True, False, None)
bob1.SetPos(chrono.ChVector3d(0, -L1, 0))
bob1.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.3, 0.2))
sys.AddBody(bob1)

# Arm2: body origin at its center, body-local X points toward bob2 (world -Y from bob1).
arm2 = chrono.ChBody()
arm2.SetMass(0.4)
arm2.SetInertiaXX(chrono.ChVector3d(0.004, 0.004, 0.004))
arm2.SetPos(chrono.ChVector3d(0, -L1 - L2 / 2, 0))
arm2.SetRot(chrono.QuatFromAngleZ(-chrono.CH_PI_2))  # body-local X → world -Y
arm2_vis = chrono.ChVisualShapeCylinder(R * 0.3, L2)
arm2_vis.SetColor(chrono.ChColor(0.2, 0.8, 0.4))
arm2.AddVisualShape(arm2_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.AddBody(arm2)

# Bob2: sphere at end of arm2
bob2 = chrono.ChBodyEasySphere(R, density, True, False, None)
bob2.SetPos(chrono.ChVector3d(0, -L1 - L2, 0))
bob2.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.7, 0.1))
sys.AddBody(bob2)

# === Joints ===
# Hinge1: arm1 <-> ground  (arm1 near end = pivot = world origin)
hinge1 = chrono.ChLinkLockRevolute()
hinge1.Initialize(
    arm1, ground, True,
    chrono.ChFramed(chrono.ChVector3d(-L1 / 2, 0, 0), chrono.QUNIT),  # arm1 LOCAL near end
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)         # ground LOCAL origin
)
sys.AddLink(hinge1)

# Hinge2: arm2 <-> bob1  (arm2 near end at bob1 center)
hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(
    arm2, bob1, True,
    chrono.ChFramed(chrono.ChVector3d(-L2 / 2, 0, 0), chrono.QUNIT),  # arm2 LOCAL near end
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)          # bob1 LOCAL center
)
sys.AddLink(hinge2)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Double Pendulum")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, -2.5, 2.0), chrono.ChVector3d(0, -1.5, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging setup (review-only) ===
if REC:  # >>> review-only >>>
    os.makedirs("frames", exist_ok=True)
    csv_f = open("simulation_data.csv", "w", newline="")
    writer = csv.writer(csv_f)
    writer.writerow(["time", "arm1_angle", "arm2_angle",
                     "bob1_x", "bob1_y", "bob2_x", "bob2_y"])
# <<< review-only <<<

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:  # review-only
            irr_dir = rec.frame_dir("frames") if REC else None
            vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
        frame += 1
        for _ in range(render_every):
            t = sys.GetChTime()
            # Angles via bob positions relative to pivot
            arm1_angle = math.atan2(-bob1.GetPos().x, -bob1.GetPos().y)
            arm2_angle = math.atan2(-(bob2.GetPos().x - bob1.GetPos().x),
                                    -(bob2.GetPos().y - bob1.GetPos().y))
            if REC:
                writer.writerow([t, arm1_angle, arm2_angle,
                                bob1.GetPos().x, bob1.GetPos().y,
                                bob2.GetPos().x, bob2.GetPos().y])
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
finally:
    if REC:  # >>> review-only >>>
        csv_f.close()
        rec.assemble_all_videos(rec.frame_dir("frames"), sensor_dirs=[])
        rec.cleanup_frames(rec.frame_dir("frames"))
    # <<< review-only <<<

# === Post-processing (review-only) ===
if REC:  # >>> review-only >>>
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        with open("simulation_data.csv", newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        if rows:
            t = [float(r["time"]) for r in rows]
            a1 = [float(r["arm1_angle"]) for r in rows]
            a2 = [float(r["arm2_angle"]) for r in rows]
            fig, ax = plt.subplots(1, 1)
            ax.plot(t, a1, label="arm1")
            ax.plot(t, a2, label="arm2")
            ax.set_xlabel("time [s]")
            ax.set_ylabel("angle [rad]")
            ax.legend()
            ax.grid()
            plt.savefig("simulation_timeseries.png")
            plt.close()
    except Exception:
        import traceback; traceback.print_exc()
# <<< review-only <<<
