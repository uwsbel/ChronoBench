"""
Gravitational particle attraction simulation using PyChrono 9.0.x with Irrlicht.

System type: ChSystemNSC with Bullet collision.
Bodies: A particle emitter spawns rigid-body particles with random shapes
        (sphere, box, cylinder), random positions, velocities, and orientations.
Custom gravitational forces are applied each timestep between all particle pairs
to simulate inter-particle gravitational attraction (N-body gravitational dynamics).

Expected behavior: Particles are emitted over time and attracted to each other
gravitationally, exhibiting clustering or orbital-like behavior near the scene center.
"""

import math
import os
import random
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Simulation constants ===
time_step = 5e-3           # physics timestep [s]  # precomputed once
sim_end = 15.0             # simulation duration [s]
render_fps = 30.0          # render cadence [fps]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Particle emitter parameters
EMIT_INTERVAL = 0.8        # seconds between new particle emissions
MAX_PARTICLES = 25         # maximum number of particles
PARTICLE_DENSITY = 800.0   # kg/m3
GRAV_CONST = 0.5           # "gravitational constant" (scaled for visualization)
SOFTENING = 0.8            # softening factor to avoid singularities [m]
MAX_SPEED = 8.0            # velocity cap to prevent numerical blowup [m/s]

# Spawn region bounds (centered at origin)
SPAWN_RADIUS = 3.0         # particles spawn within this sphere [m]
VEL_SCALE = 0.3            # max random initial speed [m/s]

random.seed(42)  # reproducible emission sequence

# === System & gravity ===
# NSC system; disable default gravity - custom N-body gravity is applied manually
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # no global gravity; N-body handled manually
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(50)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)
mat.SetRestitution(0.2)

# === Visualization (initialized before particles so BindAll works) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle Gravitational Attraction Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 8, 0), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 30, 30,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -5.0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

# === Particle list (tracked for N-body gravity) ===
particles = []


def emit_particle():
    """Emit one particle with a random shape, position, velocity, and orientation."""
    shape_type = random.choice(["sphere", "box", "cylinder"])

    # Random position within spawn sphere (uniform distribution)
    theta = random.uniform(0, 2 * math.pi)
    phi = random.uniform(0, math.pi)
    r = random.uniform(0.5, SPAWN_RADIUS)
    px = r * math.sin(phi) * math.cos(theta)
    py = r * math.sin(phi) * math.sin(theta)
    pz = r * math.cos(phi)

    # Random initial velocity (small, tangential-like)
    vx = random.uniform(-VEL_SCALE, VEL_SCALE)
    vy = random.uniform(-VEL_SCALE, VEL_SCALE)
    vz = random.uniform(-VEL_SCALE, VEL_SCALE)

    # Random orientation quaternion (via angle-axis)
    ax = random.uniform(-1, 1)
    ay = random.uniform(-1, 1)
    az = random.uniform(-1, 1)
    norm = math.sqrt(ax*ax + ay*ay + az*az) + 1e-9
    angle = random.uniform(0, 2 * math.pi)
    rot = chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(ax/norm, ay/norm, az/norm))

    if shape_type == "sphere":
        radius = random.uniform(0.15, 0.30)
        body = chrono.ChBodyEasySphere(radius, PARTICLE_DENSITY, True, True, mat)
    elif shape_type == "box":
        sx = random.uniform(0.2, 0.4)
        sy = random.uniform(0.2, 0.4)
        sz = random.uniform(0.2, 0.4)
        body = chrono.ChBodyEasyBox(sx, sy, sz, PARTICLE_DENSITY, True, True, mat)
    else:  # cylinder
        cr = random.uniform(0.1, 0.20)
        ch = random.uniform(0.2, 0.45)
        body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, cr, ch, PARTICLE_DENSITY, True, True, mat)

    body.SetPos(chrono.ChVector3d(px, py, pz))
    body.SetPosDt(chrono.ChVector3d(vx, vy, vz))
    body.SetRot(rot)

    # Random color for visibility
    r_col = random.uniform(0.3, 1.0)
    g_col = random.uniform(0.3, 1.0)
    b_col = random.uniform(0.3, 1.0)
    body.GetVisualShape(0).SetColor(chrono.ChColor(r_col, g_col, b_col))

    sys.Add(body)
    particles.append(body)
    # Bind new body's visual assets so Irrlicht can render it immediately
    vis.BindItem(body)


def apply_gravitational_forces():
    """Apply pairwise gravitational attraction forces between all particles (N-body)."""
    n = len(particles)
    for i in range(n):
        pi = particles[i]
        pos_i = pi.GetPos()  # cache: position fetched once per particle per step
        mass_i = pi.GetMass()  # cache: mass fetched once per particle per step
        for j in range(i + 1, n):
            pj = particles[j]
            pos_j = pj.GetPos()  # cache: position fetched once per pair
            mass_j = pj.GetMass()  # cache: mass fetched once per pair
            # Direction vector from i to j
            dx = pos_j.x - pos_i.x
            dy = pos_j.y - pos_i.y
            dz = pos_j.z - pos_i.z
            dist2 = dx*dx + dy*dy + dz*dz + SOFTENING * SOFTENING  # softened distance^2
            dist = math.sqrt(dist2)
            # Gravitational force magnitude: F = G * m_i * m_j / dist^2 (softened)
            F_mag = GRAV_CONST * mass_i * mass_j / dist2
            # Force components along the line joining the two particles
            inv_dist = 1.0 / dist
            fx = F_mag * dx * inv_dist
            fy = F_mag * dy * inv_dist
            fz = F_mag * dz * inv_dist
            # Apply equal and opposite forces (Newton's 3rd law)
            pi.AccumulateForce(chrono.ChVector3d(fx, fy, fz), pos_i, False)
            pj.AccumulateForce(chrono.ChVector3d(-fx, -fy, -fz), pos_j, False)


def clamp_particle_velocities():
    """Cap particle speeds to prevent numerical divergence."""
    for p in particles:
        vel = p.GetPosDt()  # cache: velocity vector
        speed = vel.Length()
        if speed > MAX_SPEED:
            scale = MAX_SPEED / speed
            p.SetPosDt(chrono.ChVector3d(vel.x * scale, vel.y * scale, vel.z * scale))


# === Review-only setup ===


# === Main loop ===
next_emit_time = 0.0
frame = 0  # precomputed once (consecutive review frame counter)

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()  # cache: sim time fetched once per physics step

            # Emit a new particle if interval elapsed and under particle cap
            if t >= next_emit_time and len(particles) < MAX_PARTICLES:
                emit_particle()
                next_emit_time = t + EMIT_INTERVAL

            # Apply custom N-body gravitational attraction between all particles
            apply_gravitational_forces()

            # Clamp velocities to prevent numerical divergence
            clamp_particle_velocities()


            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad particle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
