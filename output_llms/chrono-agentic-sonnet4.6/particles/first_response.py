"""
Particle gravitational attraction simulation using PyChrono.

Models a set of particles (spheres, boxes, cylinders) emitted from a central
emitter with random shapes, positions, velocities, and orientations. Custom
gravitational forces are applied every physics step to simulate mutual
gravitational attraction between all particles. Uses ChSystemNSC with Bullet
collision, Irrlicht visualization. Expected behavior: particles drift toward
each other, clustering over time under mutual gravitational pull.
"""

import os
import math
import random
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
GRAVITY_CONST   = 6.674e-3   # artificial gravitational constant (scaled for visibility)
PARTICLE_MASS   = 1.0        # mass per particle [kg]
PARTICLE_DENSITY = 500.0     # density used for easy-factory bodies [kg/m³]
EMIT_COUNT      = 40         # number of particles to emit
EMIT_RADIUS     = 4.0        # emitter spawn radius [m]
MAX_VEL         = 0.5        # max initial speed [m/m]
TIME_STEP       = 1e-3       # physics time step [s]
SIM_END         = 10.0       # simulation end time [s]
RENDER_FPS      = 50.0
RENDER_EVERY    = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

random.seed(42)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))   # no global gravity; custom forces
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(50)

# === Contact material (NSC) ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.3)
mat.SetRestitution(0.2)

# === Particle emitter — random shapes, positions, velocities, orientations ===
particles = []

SHAPE_TYPES = ["sphere", "box", "cylinder"]

for i in range(EMIT_COUNT):
    # Random position on sphere surface of EMIT_RADIUS
    theta = random.uniform(0, 2 * math.pi)
    phi   = random.uniform(0, math.pi)
    px = EMIT_RADIUS * math.sin(phi) * math.cos(theta)
    py = EMIT_RADIUS * math.sin(phi) * math.sin(theta)
    pz = EMIT_RADIUS * math.cos(phi)

    # Random initial velocity directed roughly inward
    vx = random.uniform(-MAX_VEL, MAX_VEL)
    vy = random.uniform(-MAX_VEL, MAX_VEL)
    vz = random.uniform(-MAX_VEL, MAX_VEL)

    # Random orientation quaternion
    ax = random.uniform(-1, 1); ay = random.uniform(-1, 1); az = random.uniform(-1, 1)
    anorm = math.sqrt(ax*ax + ay*ay + az*az) + 1e-9
    angle = random.uniform(0, 2 * math.pi)
    rot = chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(ax/anorm, ay/anorm, az/anorm))

    shape_type = SHAPE_TYPES[i % len(SHAPE_TYPES)]
    size = random.uniform(0.1, 0.25)

    if shape_type == "sphere":
        body = chrono.ChBodyEasySphere(size, PARTICLE_DENSITY, True, True, mat)
    elif shape_type == "box":
        body = chrono.ChBodyEasyBox(size, size, size, PARTICLE_DENSITY, True, True, mat)
    else:  # cylinder
        body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, size * 0.5, size, PARTICLE_DENSITY, True, True, mat)

    body.SetPos(chrono.ChVector3d(px, py, pz))
    body.SetRot(rot)
    body.SetPosDt(chrono.ChVector3d(vx, vy, vz))
    sys.Add(body)
    particles.append(body)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle Gravitational Attraction")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, -12, 6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# === Review-only setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()  # cache: fetched once per physics step

            # Apply custom pairwise gravitational attraction
            n = len(particles)
            forces = [chrono.ChVector3d(0, 0, 0)] * n
            for a in range(n):
                pa = particles[a].GetPos()  # cache: position of particle a
                ma = particles[a].GetMass()  # cache: mass a
                for b in range(a + 1, n):
                    pb = particles[b].GetPos()  # cache: position of particle b
                    mb = particles[b].GetMass()  # cache: mass b
                    dx = pb.x - pa.x
                    dy = pb.y - pa.y
                    dz = pb.z - pa.z
                    dist2 = dx*dx + dy*dy + dz*dz + 1e-4  # softened to avoid singularity
                    dist = math.sqrt(dist2)
                    fmag = GRAVITY_CONST * ma * mb / dist2
                    fx = fmag * dx / dist
                    fy = fmag * dy / dist
                    fz = fmag * dz / dist
                    forces[a] = chrono.ChVector3d(
                        forces[a].x + fx, forces[a].y + fy, forces[a].z + fz)
                    forces[b] = chrono.ChVector3d(
                        forces[b].x - fx, forces[b].y - fy, forces[b].z - fz)

            for idx, body in enumerate(particles):
                body.EmptyAccumulators()
                body.AccumulateForce(forces[idx], body.GetPos(), False)


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad particle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
