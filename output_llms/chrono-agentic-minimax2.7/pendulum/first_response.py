"""
Simple pendulum simulation using PyChrono with Irrlicht visualization.

System: ChSystemNSC (Non-Smooth Contact, rigid-body pendulum)
Bodies: fixed ground body + pendulum bob connected via ChLinkLockRevolute
Expected behavior: pendulum swings under gravity with periodic logging of position and velocity.
"""
import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# review-only: sim_recording for video assembly

# === Physical constants ===
pendulum_mass = 10.0          # kg
pendulum_radius = 0.1         # m (sphere radius)
pendulum_inertia = 0.04       # kg·m² (Ixx = Iyy = Izz for solid sphere: 2/5 * m * r²)
pendulum_initial_angle = 0.6  # rad (~34 degrees from vertical)
arm_length = 1.5              # m (distance from pivot to bob center)
time_step = 1e-3             # s (high-precision for MBS)
sim_end = 10.0               # s
render_fps = 50.0             # frames per second for review video
render_every = max(1, round(1.0 / (render_fps * time_step)))


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up gravity

# === Bodies ===

# --- Ground (fixed anchor body) ---
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(ground)

# --- Pendulum bob (sphere body) ---
# Pivot is at (0, 0, 0); arm_length is from pivot to bob center.
# Initial position at angle = pendulum_initial_angle from vertical (Y-axis).
bob_x = arm_length * math.sin(pendulum_initial_angle)
bob_y = -arm_length * math.cos(pendulum_initial_angle)

pendulum_bob = chrono.ChBody()
pendulum_bob.SetMass(pendulum_mass)
pendulum_bob.SetInertiaXX(chrono.ChVector3d(pendulum_inertia, pendulum_inertia, pendulum_inertia))
pendulum_bob.SetPos(chrono.ChVector3d(bob_x, bob_y, 0))
# Initial angular velocity so the pendulum starts swinging
pendulum_bob.SetAngVelParent(chrono.ChVector3d(0, 0, 0))  # starts from rest
sys.AddBody(pendulum_bob)

# Visual shape for pendulum bob (sphere)
bob_sphere = chrono.ChVisualShapeSphere(pendulum_radius)
bob_sphere.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
pendulum_bob.AddVisualShape(bob_sphere)

# --- Revolute joint (hinge at origin, pendulum swings in XY plane) ---
# Hinge axis = world +Z; local +Z of joint frame is already +Z, so QUNIT suffices.
joint_revolute = chrono.ChLinkLockRevolute()
joint_revolute.Initialize(
    pendulum_bob,  # body A
    ground,        # body B (fixed)
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)  # pivot at world origin
)
sys.AddLink(joint_revolute)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Simple Pendulum")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.0, -2.0, 3.0), chrono.ChVector3d(0, -1.0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# review-only: prepare frame capture directories

# CSV logging setup (review-only)
csv_path = "simulation_data.csv"
csv_file = None   # assigned inside try below
csv_writer = None


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        if REC:  # review-only: capture frame
            vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
            frame += 1

        # Inner physics batch: advance multiple steps per rendered frame
        for _ in range(render_every):
            t = sys.GetChTime()

            # Compute pendulum angle (from Y-axis vertical, positive = swung toward +X)
            angle = math.atan2(pendulum_bob.GetPos().x, -pendulum_bob.GetPos().y)


            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:

# Post-processing: generate timeseries plot (review-only)
