"""
Gravitational particle attraction simulation using PyChrono (NSC system).

Models a set of particles emitted with random shapes (sphere/box/cylinder),
random initial positions, velocities, and orientations. Custom gravitational
forces are applied each step to simulate mutual attraction between particles,
causing them to cluster and orbit around a common center of mass. Uses
Irrlicht for real-time 3D visualization.
"""

import os
import math
import random
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
time_step    = 1e-3       # physics time step [s]
sim_end      = 15.0       # simulation duration [s]
render_fps   = 50.0       # render frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

GRAV_CONST    = 6.674e-4   # scaled gravitational constant for visible effect
NUM_PARTICLES = 20         # number of emitted particles
EMIT_RADIUS   = 3.0        # spawn radius [m]
PARTICLE_DENSITY = 800.0   # density [kg/m³]
MIN_DIST      = 0.1        # softening distance to avoid singularity [m]

random.seed(42)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # no global gravity; custom forces only
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(50)

# === Contact material (NSC matches ChSystemNSC) ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.3)
mat.SetRestitution(0.2)

# === Particle emitter — random shapes, positions, velocities, orientations ===
particles = []  # list of ChBody references for force application

for i in range(NUM_PARTICLES):
    # Random position on sphere shell
    theta = random.uniform(0, 2 * math.pi)
    phi   = random.uniform(0, math.pi)
    r     = random.uniform(0.5 * EMIT_RADIUS, EMIT_RADIUS)
    px = r * math.sin(phi) * math.cos(theta)
    py = r * math.sin(phi) * math.sin(theta)
    pz = r * math.cos(phi)

    # Random initial velocity (small)
    vx = random.uniform(-0.5, 0.5)
    vy = random.uniform(-0.5, 0.5)
    vz = random.uniform(-0.5, 0.5)

    # Random orientation (angle-axis)
    ax = random.uniform(-1, 1)
    ay = random.uniform(-1, 1)
    az = random.uniform(-1, 1)
    axis_len = math.sqrt(ax*ax + ay*ay + az*az) + 1e-9
    ax /= axis_len; ay /= axis_len; az /= axis_len
    angle = random.uniform(0, 2 * math.pi)
    rot = chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(ax, ay, az))

    shape_type = i % 3  # cycle: sphere, box, cylinder

    if shape_type == 0:
        radius = random.uniform(0.08, 0.18)
        body = chrono.ChBodyEasySphere(radius, PARTICLE_DENSITY, True, True, mat)
    elif shape_type == 1:
        sx = random.uniform(0.1, 0.25)
        sy = random.uniform(0.1, 0.25)
        sz = random.uniform(0.1, 0.25)
        body = chrono.ChBodyEasyBox(sx, sy, sz, PARTICLE_DENSITY, True, True, mat)
    else:
        cr = random.uniform(0.06, 0.14)
        ch = random.uniform(0.1, 0.25)
        body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, cr, ch, PARTICLE_DENSITY, True, True, mat)

    body.SetPos(chrono.ChVector3d(px, py, pz))
    body.SetRot(rot)
    body.SetPosDt(chrono.ChVector3d(vx, vy, vz))
    sys.Add(body)
    particles.append(body)

# === Visualization — Irrlicht (Initialize FIRST, scene elements AFTER) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gravitational Particle Attraction")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -12, 6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -EMIT_RADIUS - 0.5), chrono.QUNIT),
            chrono.ChColor(0.35, 0.35, 0.35))


# === Main loop ===
n = len(particles)  # cache: particle count, constant
frame = 0


try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            # Apply pairwise gravitational attraction between all particles
            for i in range(n):
                pi = particles[i]       # cache: body i
                pos_i = pi.GetPos()     # cache: position of i this step
                mass_i = pi.GetMass()   # cache: mass of i
                fx = fy = fz = 0.0
                for j in range(n):
                    if j == i:
                        continue
                    pj = particles[j]   # cache: body j
                    pos_j = pj.GetPos()
                    dx = pos_j.x - pos_i.x
                    dy = pos_j.y - pos_i.y
                    dz = pos_j.z - pos_i.z
                    dist2 = dx*dx + dy*dy + dz*dz + MIN_DIST*MIN_DIST
                    dist = math.sqrt(dist2)
                    f = GRAV_CONST * mass_i * pj.GetMass() / dist2
                    fx += f * dx / dist
                    fy += f * dy / dist
                    fz += f * dz / dist
                pi.AccumulateForce(chrono.ChVector3d(fx, fy, fz), pos_i, False)


            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad particle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
