"""
Slider-crank mechanism with spherical joints and planar constraint.
Turn 3 modifications: crank-rod and rod-piston changed from revolute to spherical;
piston-floor prismatic replaced with a plane-plane (planar) joint.

System: ChSystemNSC (Non-Smooth Contact)
Bodies: floor (truss), crank, connecting rod, piston
Joints: motor (crank-floor), spherical (crank-rod, rod-piston), planar (piston-floor)
Expected: crank rotates at pi rad/s, piston slides/rotates freely in XY plane.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# review-only import for recording

# === Simulation parameters ===
time_step = 1e-3
sim_end = 20.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Mechanism geometry
crank_center = chrono.ChVector3d(-1.0, 0.5, 0.0)  # crank pivot on floor
crank_rad = 0.4        # crank throw radius [m]
crank_thick = 0.1      # crank disk thickness [m]
rod_length = 1.5       # connecting rod length [m]
piston_rad = 0.2       # piston radius [m]
piston_thick = 0.3     # piston height [m]

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

# === Bodies ===

# Floor / truss (fixed)
mfloor = chrono.ChBodyEasyBox(3.0, 1.0, 3.0, 1000.0, True, True)
mfloor.SetPos(chrono.ChVector3d(0.0, -0.5, 0.0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Crank disk — cylinder along Y initially, then rotated to Z-axis alignment
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000.0)
mcrank.SetPos(crank_center + chrono.ChVector3d(0.0, 0.0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Connecting rod — box aligned along X
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000.0)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2.0, 0.0, 0.0))
sys.Add(mrod)

# Piston — cylinder along X
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, piston_rad, piston_thick, 1000.0)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0.0, 0.0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# === Joints ===

# Motor — drives crank rotation about floor
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))
sys.Add(my_motor)

# Spherical joint — crank to rod (ball-and-socket at crank pin)
mjoint_crank_rod = chrono.ChLinkLockSpherical()
mjoint_crank_rod.Initialize(
    mrod, mcrank,
    chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0.0, 0.0)),
)
sys.Add(mjoint_crank_rod)

# Spherical joint — rod to piston (wrist pin)
mjoint_rod_piston = chrono.ChLinkLockSpherical()
mjoint_rod_piston.Initialize(
    mpiston, mrod,
    chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0.0, 0.0)),
)
sys.Add(mjoint_rod_piston)

# Planar joint — piston constrained to XY plane (replaces prismatic)
plane_joint = chrono.ChLinkLockPlanar()
plane_joint.Initialize(
    mfloor, mpiston,
    chrono.ChFramed(
        crank_center + chrono.ChVector3d(crank_rad + rod_length, 0.0, 0.0),
        chrono.Q_ROTATE_Y_TO_X,
    ),
)
sys.Add(plane_joint)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Slider-crank — spherical joints + planar constraint")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 1.0, 3.0), chrono.ChVector3d(0.0, 1.0, 0.0))
vis.AddTypicalLights()

# === CSV logging (review-only setup — stripped; here only for reference) ===
# data_path = "simulation_data.csv"
# csv_file = None
# try:
#     csv_file = open(data_path, "w", newline="")
#     fieldnames = ["time", "motor_angle", "piston_x", "piston_y", "piston_vx"]
#     data_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
#     data_writer.writeheader()
# except (OSError, IOError):
#     pass
# finally:
#     if csv_file is not None:
#         csv_file.close()

# === Main loop ===
frame = 0


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
