"""Slider-crank mechanism with motion data collection and Matplotlib analysis.

Models a planar crank-rod-piston slider-crank driven by a prescribed-speed
rotational motor (NSC rigid-body system, Y-up gravity). Bodies: a fixed truss,
a crank spun at constant angular speed, a connecting rod, and a piston that
slides on a fixed horizontal guide. Topology: crank-truss motor (full motor-link),
crank-rod revolute, rod-piston revolute, piston-truss prismatic.

During the run the crank angle, piston position, and piston speed are sampled
each step into arrays. The run stops after 20 s, then Matplotlib renders two
subplots — piston position vs crank angle and piston speed vs crank angle — with
the crank-angle axis ticked at pi-based intervals (0, pi/2, pi, 3pi/2, 2pi).
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")                       # headless plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / drive / timing constants (no bare literals below)
time_step = 1e-3            # integration step [s]
sim_end = 20.0             # stop the run after 20 s (objective)
render_fps = 50.0          # review-video frame cadence
crank_radius = 0.1         # crank pin offset from rotation axis [m]
rod_length = 0.3           # connecting-rod length [m]
crank_speed = chrono.CH_PI  # prescribed crank angular speed [rad/s]
piston_x0 = crank_radius + rod_length   # piston start x (crank at angle 0)

render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity === NSC rigid-body world, standard Y-up gravity
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies === fixed truss + crank + connecting rod + piston (pure jointed MBS)
truss = chrono.ChBody()
truss.SetFixed(True)
sys.AddBody(truss)

crank = chrono.ChBody()
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.005, 0.005, 0.005))
crank.SetPos(chrono.ChVector3d(crank_radius * 0.5, 0, 0))
crank.SetRot(chrono.QUNIT)
sys.AddBody(crank)
crank_vis = chrono.ChVisualShapeCylinder(0.02, crank_radius)
crank.AddVisualShape(crank_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

rod = chrono.ChBody()
rod.SetMass(0.5)
rod.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
rod.SetPos(chrono.ChVector3d(crank_radius + rod_length * 0.5, 0, 0))
rod.SetRot(chrono.QUNIT)
sys.AddBody(rod)
rod_vis = chrono.ChVisualShapeCylinder(0.015, rod_length)
rod.AddVisualShape(rod_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

piston = chrono.ChBody()
piston.SetMass(0.8)
piston.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
piston.SetPos(chrono.ChVector3d(piston_x0, 0, 0))
sys.AddBody(piston)
piston_vis = chrono.ChVisualShapeCylinder(0.05, 0.08)
piston.AddVisualShape(piston_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# === Joints / constraints === motor + two revolutes + prismatic guide
# crank <-> truss : prescribed-speed motor (FULL motor-link, no separate revolute)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(crank_speed))
sys.AddLink(motor)

# crank <-> rod : revolute at the crank pin (hinge about world Z, planar XY motion)
crank_rod = chrono.ChLinkLockRevolute()
crank_rod.Initialize(crank, rod, chrono.ChFramed(chrono.ChVector3d(crank_radius, 0, 0), chrono.QUNIT))
sys.AddLink(crank_rod)

# rod <-> piston : revolute at the wrist pin
rod_piston = chrono.ChLinkLockRevolute()
rod_piston.Initialize(rod, piston, chrono.ChFramed(chrono.ChVector3d(piston_x0, 0, 0), chrono.QUNIT))
sys.AddLink(rod_piston)

# piston <-> truss : prismatic guide along world X (frame local +Z -> world X)
piston_truss = chrono.ChLinkLockPrismatic()
piston_truss.Initialize(piston, truss, chrono.ChFramed(chrono.ChVector3d(piston_x0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(piston_truss)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.2, 0.4, -0.9), chrono.ChVector3d(0.2, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.05, 0.05, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0.2, -0.15, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Data arrays === per-step samples for the post-run Matplotlib analysis
array_time = []
array_angle = []
array_pos = []
array_speed = []

# cache: motor handle fetched once, reused every step for the crank angle readout
motor_cache = motor
piston_cache = piston   # cache: piston handle reused for pos/speed each step


# === Main loop === render at frame cadence, batch physics steps between frames
os.makedirs("cam", exist_ok=True)          # guard against missing output dir
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            array_time.append(t)
            array_angle.append(motor_cache.GetMotorAngle())
            array_pos.append(piston_cache.GetPos().x)
            array_speed.append(piston_cache.GetPosDt().x)
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === Matplotlib: piston position & speed vs crank angle
angle = np.array(array_angle)
pos = np.array(array_pos)
speed = np.array(array_speed)

max_angle = float(angle[-1]) if angle.size else 2.0 * math.pi
tick_vals = np.arange(0.0, max_angle + math.pi / 2.0, math.pi / 2.0)
tick_labels = [r"$0$", r"$\pi/2$", r"$\pi$", r"$3\pi/2$", r"$2\pi$"]
labels = [tick_labels[i % 4] if i % 4 != 0 else (r"$0$" if i == 0 else rf"${i//2}\pi$")
          for i in range(len(tick_vals))]

fig, (ax_pos, ax_speed) = plt.subplots(2, 1, figsize=(9, 7))
ax_pos.plot(angle, pos, color="tab:blue")
ax_pos.set_xlabel("crank angle [rad]")
ax_pos.set_ylabel("position [m]")
ax_pos.set_title("Piston position vs crank angle")
ax_pos.set_xticks(tick_vals)
ax_pos.set_xticklabels(labels)
ax_pos.grid(True)

ax_speed.plot(angle, speed, color="tab:red")
ax_speed.set_xlabel("crank angle [rad]")
ax_speed.set_ylabel("speed [m/s]")
ax_speed.set_title("Piston speed vs crank angle")
ax_speed.set_xticks(tick_vals)
ax_speed.set_xticklabels(labels)
ax_speed.grid(True)

fig.tight_layout()
fig.savefig("simulation_timeseries.png", dpi=120)
plt.close(fig)
