"""
Particle gravitational attraction simulation.

Models N particles with mutual gravitational attraction using a particle emitter
that generates random shapes, positions, velocities, and orientations. Visualized
with Irrlicht. Uses ChSystemNSC with collision detection.
"""

import os
import math
import random
import numpy as np

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
time_step = 1e-3           # physics time step [s]
sim_end = 10.0             # simulation duration [s]
render_fps = 50.0          # frames per second for rendering
render_every = max(1, round(1.0 / (render_fps * time_step)))
G_constant = 6.674e-6      # gravitational constant (scaled for visual effect)
emission_interval = 0.3     # seconds between particle emissions
max_particles = 80          # maximum number of active particles
spawn_height = 3.0         # height above ground to spawn particles
spawn_radius = 3.0         # horizontal spawn radius

# === System setup ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)
mat.SetRestitution(0.3)

# === Ground plane (FIXED: raised so top surface is at y=0, collision works) ===
# Ground box: center at y=-5, height=10 -> top surface at y=0
ground = chrono.ChBodyEasyBox(20.0, 10.0, 20.0, 1000.0, True, True, mat)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0.0, -5.0, 0.0))
sys.AddBody(ground)

# === Particle emitter ===
class ParticleEmitter:
    """Generates particles with random shapes, positions, velocities, and orientations."""

    def __init__(self, system, material):
        self.sys = system
        self.mat = material
        self.particles = []
        self.emission_timer = 0.0
        self.emitted_count = 0

    def update(self, dt, current_time):
        """Emit new particles on schedule."""
        self.emission_timer += dt
        if self.emission_timer >= emission_interval and len(self.particles) < max_particles:
            self._emit_particle(current_time)
            self.emission_timer = 0.0

    def _emit_particle(self, current_time):
        """Create a single particle with random properties."""
        # Random shape type
        shape_type = random.choice(["sphere", "box", "cylinder"])

        # Random mass (affects gravitational force)
        mass = random.uniform(0.5, 3.0)

        if shape_type == "sphere":
            radius = random.uniform(0.1, 0.3)
            density = mass / (4.0/3.0 * math.pi * radius**3)
            body = chrono.ChBodyEasySphere(radius, density, True, True, self.mat)
            inertia = 0.4 * mass * radius**2

        elif shape_type == "box":
            sx = random.uniform(0.15, 0.35)
            sy = random.uniform(0.15, 0.35)
            sz = random.uniform(0.15, 0.35)
            volume = sx * sy * sz
            density = mass / volume
            body = chrono.ChBodyEasyBox(sx, sy, sz, density, True, True, self.mat)
            inertia = (1.0/12.0) * mass * (sy**2 + sz**2)

        else:  # cylinder
            radius = random.uniform(0.08, 0.2)
            height = random.uniform(0.2, 0.4)
            volume = math.pi * radius**2 * height
            density = mass / volume
            axis_choice = random.choice([chrono.ChAxis_X, chrono.ChAxis_Y, chrono.ChAxis_Z])
            body = chrono.ChBodyEasyCylinder(axis_choice, radius, height, density, True, True, self.mat)
            inertia = 0.5 * mass * radius**2

        # Random spawn position (well above ground top surface at y=0)
        angle = random.uniform(0.0, 2.0 * math.pi)
        r = random.uniform(0.0, spawn_radius)
        x = r * math.cos(angle)
        z = r * math.sin(angle)
        y = spawn_height + random.uniform(0.0, 2.0)

        body.SetPos(chrono.ChVector3d(x, y, z))
        body.SetMass(mass)
        body.SetInertiaXX(chrono.ChVector3d(inertia, inertia, inertia))

        # Random initial velocity (slight outward + downward bias)
        vx = random.uniform(-1.5, 1.5)
        vy = random.uniform(-2.0, 0.0)
        vz = random.uniform(-1.5, 1.5)
        body.SetPosDt(chrono.ChVector3d(vx, vy, vz))

        # Random initial angular velocity
        ang_vel = chrono.ChVector3d(
            random.uniform(-3.0, 3.0),
            random.uniform(-3.0, 3.0),
            random.uniform(-3.0, 3.0),
        )
        body.SetAngVelParent(ang_vel)

        self.sys.AddBody(body)
        self.particles.append(body)
        self.emitted_count += 1

    def apply_gravitational_forces(self):
        """Apply mutual gravitational attraction between all particle pairs.

        Uses direct velocity modification for particle dynamics (simplified gravity).
        """
        n = len(self.particles)
        for i in range(n):
            for j in range(i + 1, n):
                pi = self.particles[i]
                pj = self.particles[j]

                # Vector from i to j
                dx = pj.GetPos().x - pi.GetPos().x
                dy = pj.GetPos().y - pi.GetPos().y
                dz = pj.GetPos().z - pi.GetPos().z
                dist_sq = dx*dx + dy*dy + dz*dz
                dist = math.sqrt(dist_sq)

                if dist < 0.05:
                    dist = 0.05  # prevent singularity

                # Gravitational acceleration magnitude: a = G * m2 / r^2
                # (We use acceleration directly on particle i due to particle j)
                m2 = pj.GetMass()
                a_mag = G_constant * m2 / dist_sq

                # Direction (normalized)
                ax = a_mag * dx / dist
                ay = a_mag * dy / dist
                az = a_mag * dz / dist

                # Apply acceleration as velocity change (simplified, timestep-scaled)
                dt_capped = min(time_step, 0.02)  # stability
                pi.SetPosDt(pi.GetPosDt() + chrono.ChVector3d(ax * dt_capped, ay * dt_capped, az * dt_capped))

                # Equal and opposite for particle j
                m1 = pi.GetMass()
                a_mag_j = G_constant * m1 / dist_sq
                pj.SetPosDt(pj.GetPosDt() + chrono.ChVector3d(-ax * dt_capped * m1 / m2,
                                                               -ay * dt_capped * m1 / m2,
                                                               -az * dt_capped * m1 / m2))

    def get_particle_count(self):
        return len(self.particles)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle Gravitational Attraction")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, -8, 6), chrono.ChVector3d(0, 2, 0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Particle emitter setup ===
emitter = ParticleEmitter(sys, mat)

# === CSV logging setup ===
os.makedirs("frames", exist_ok=True)
import csv

csv_path = "simulation_data.csv"
motion_csv_path = "motion_log.csv"

# review-only CSV writer block
REC = bool(os.environ.get("SIMBENCH_RECORD"))
irr_dir = "frames" if REC else None

# === Main simulation loop ===
frame = 0
try:
    with open(csv_path, "w", newline="") as cf, \
         open(motion_csv_path, "w", newline="") as mf:
        data_writer = csv.DictWriter(cf, fieldnames=["time", "particle_count", "total_ke"])
        data_writer.writeheader()
        motion_writer = csv.DictWriter(mf, fieldnames=["time", "body_id", "x", "y", "z", "vx", "vy", "vz"])
        motion_writer.writeheader()

        while vis.Run() and sys.GetChTime() < sim_end:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            if REC and frame % render_every == 0:
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")

            frame += 1

            for _ in range(render_every):
                t = sys.GetChTime()

                # Emit new particles
                emitter.update(time_step, t)

                # Apply gravitational forces between particles
                emitter.apply_gravitational_forces()

                # Log particle data
                total_ke = 0.0
                for idx, p in enumerate(emitter.particles):
                    vel = p.GetPosDt()
                    total_ke += 0.5 * p.GetMass() * (vel.x**2 + vel.y**2 + vel.z**2)
                    if REC:
                        motion_writer.writerow({
                            "time": t,
                            "body_id": idx,
                            "x": p.GetPos().x, "y": p.GetPos().y, "z": p.GetPos().z,
                            "vx": vel.x, "vy": vel.y, "vz": vel.z,
                        })

                if REC:
                    data_writer.writerow({
                        "time": t,
                        "particle_count": emitter.get_particle_count(),
                        "total_ke": total_ke,
                    })

                sys.DoStepDynamics(time_step)

                if sys.GetChTime() >= sim_end:
                    break

except (RuntimeError, ValueError, OSError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # CSV writers closed by context managers

# review-only: assemble videos and plot
if REC:
    try:
        import sim_recording as rec
        rec.assemble_all_videos("frames", sensor_dirs=[])
        rec.plot_table(csv_path, "simulation_timeseries.png")
        rec.cleanup_frames("frames", [])
    except Exception:
        pass
