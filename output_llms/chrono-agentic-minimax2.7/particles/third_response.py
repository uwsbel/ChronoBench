"""
Three-Body Particle Simulation

Models three spheres interacting via contact dynamics in a three-body gravitational
scenario. Spheres have specified initial positions and velocities:
  Sphere 1: origin, velocity (0.5, 0, 0.1)
  Sphere 2: (-10, -10, 0), velocity (-0.5, 0, -0.1)
  Sphere 3: (0, 20, 0), velocity (0, -0.5, 0.2)

System: ChSystemNSC (Non-Smooth Contact for rigid impacts)
"""

import os
import csv
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
SPHERE_RADIUS = 0.5          # sphere radius [m]
SPHERE_DENSITY = 1000.0     # density [kg/m^3]
SPHERE_MASS = (4/3) * math.pi * SPHERE_RADIUS**3 * SPHERE_DENSITY

time_step = 1e-3            # physics timestep [s]
sim_end = 20.0             # simulation duration [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Sphere 1: origin
S1_POS = chrono.ChVector3d(0, 0, 0)
S1_VEL = chrono.ChVector3d(0.5, 0, 0.1)

# Sphere 2: (-10, -10, 0)
S2_POS = chrono.ChVector3d(-10, -10, 0)
S2_VEL = chrono.ChVector3d(-0.5, 0, -0.1)

# Sphere 3: (0, 20, 0)
S3_POS = chrono.ChVector3d(0, 20, 0)
S3_VEL = chrono.ChVector3d(0, -0.5, 0.2)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)
mat.SetRestitution(0.3)

# === Bodies ===
# Sphere 1
sphere1 = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, True, mat)
sphere1.SetPos(S1_POS)
sphere1.SetLinVel(S1_VEL)
sys.AddBody(sphere1)

# Sphere 2
sphere2 = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, True, mat)
sphere2.SetPos(S2_POS)
sphere2.SetLinVel(S2_VEL)
sys.AddBody(sphere2)

# Sphere 3
sphere3 = chrono.ChBodyEasySphere(SPHERE_RADIUS, SPHERE_DENSITY, True, True, mat)
sphere3.SetPos(S3_POS)
sphere3.SetLinVel(S3_VEL)
sys.AddBody(sphere3)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Body Particle Simulation")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -30, 30), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))


# === CSV logging setup ===
csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None

try:
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["time",
                          "s1_x", "s1_y", "s1_z", "s1_vx", "s1_vy", "s1_vz",
                          "s2_x", "s2_y", "s2_z", "s2_vx", "s2_vy", "s2_vz",
                          "s3_x", "s3_y", "s3_z", "s3_vx", "s3_vy", "s3_vz"])

    # === Main loop ===
    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame += 1
        for _ in range(render_every):
            t = sys.GetChTime()
            s1p = sphere1.GetPos()
            s1v = sphere1.GetLinVel()
            s2p = sphere2.GetPos()
            s2v = sphere2.GetLinVel()
            s3p = sphere3.GetPos()
            s3v = sphere3.GetLinVel()

            csv_writer.writerow([t,
                                  s1p.x, s1p.y, s1p.z, s1v.x, s1v.y, s1v.z,
                                  s2p.x, s2p.y, s2p.z, s2v.x, s2v.y, s2v.z,
                                  s3p.x, s3p.y, s3p.z, s3v.x, s3v.y, s3v.z])

            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

        frame += 1

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    if csv_file:
        csv_file.close()


# === Post-processing ===
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    times, s1x, s1y, s1z, s2x, s2y, s2z, s3x, s3y, s3z = [], [], [], [], [], [], [], [], [], []
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            times.append(float(row[0]))
            s1x.append(float(row[1]))
            s1y.append(float(row[2]))
            s1z.append(float(row[3]))
            s2x.append(float(row[7]))
            s2y.append(float(row[8]))
            s2z.append(float(row[9]))
            s3x.append(float(row[13]))
            s3y.append(float(row[14]))
            s3z.append(float(row[15]))

    fig, axes = plt.subplots(3, 1, sharex=True)
    axes[0].plot(times, s1x, times, s1y, times, s1z)
    axes[0].set_ylabel("Sphere 1 [m]")
    axes[0].grid()
    axes[1].plot(times, s2x, times, s2y, times, s2z)
    axes[1].set_ylabel("Sphere 2 [m]")
    axes[1].grid()
    axes[2].plot(times, s3x, times, s3y, times, s3z)
    axes[2].set_ylabel("Sphere 3 [m]")
    axes[2].set_xlabel("Time [s]")
    axes[2].grid()
    plt.tight_layout()
    plt.savefig("simulation_timeseries.png")
    plt.close()
except Exception as exc:
    import traceback
    traceback.print_exc()
