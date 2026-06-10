"""Particle attraction simulation using PyChrono NSC rigid bodies.

This script emits random sphere, box, and cylinder particles into a zero-global-
gravity system, then applies custom pairwise attraction forces so the particles
cluster dynamically while still colliding through Bullet contact. Irrlicht shows
the evolving cloud and the review layer records a short video plus physics data.
"""

import math
import random
import sys
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === reproducible particle field and bounded validation runtime
TIME_STEP = 0.002
SIM_END = 5.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
PARTICLE_COUNT = 24
EMIT_PERIOD = 0.045
PARTICLE_DENSITY = 850.0
ATTRACTION_G = 0.18
SOFTENING = 0.12
SPAWN_RADIUS = 1.4
RNG_SEED = 17


def random_unit_vector(rng):
    """Return a nonzero random unit vector for orientations and velocities."""
    x = rng.uniform(-1.0, 1.0)
    y = rng.uniform(-1.0, 1.0)
    z = rng.uniform(-1.0, 1.0)
    length = math.sqrt(x * x + y * y + z * z)
    if length < 1e-9:
        return chrono.ChVector3d(1.0, 0.0, 0.0)
    inv_length = 1.0 / length
    return chrono.ChVector3d(x * inv_length, y * inv_length, z * inv_length)


def make_particle(system, material, rng, index):
    """Emit one randomly shaped particle with pose, velocity, and spin."""
    shape_kind = rng.choice(("sphere", "box", "cylinder"))
    radius = rng.uniform(0.07, 0.13)
    if shape_kind == "sphere":
        body = chrono.ChBodyEasySphere(radius, PARTICLE_DENSITY, True, True, material)
    elif shape_kind == "box":
        sx = rng.uniform(0.12, 0.24)
        sy = rng.uniform(0.12, 0.22)
        sz = rng.uniform(0.12, 0.22)
        body = chrono.ChBodyEasyBox(sx, sy, sz, PARTICLE_DENSITY, True, True, material)
    else:
        cyl_radius = rng.uniform(0.06, 0.11)
        cyl_height = rng.uniform(0.15, 0.28)
        body = chrono.ChBodyEasyCylinder(
            chrono.ChAxis_Y, cyl_radius, cyl_height, PARTICLE_DENSITY, True, True, material
        )

    angle = rng.uniform(0.0, 2.0 * math.pi)
    body.SetRot(chrono.QuatFromAngleAxis(angle, random_unit_vector(rng)))
    body.SetPos(
        chrono.ChVector3d(
            rng.uniform(-SPAWN_RADIUS, SPAWN_RADIUS),
            rng.uniform(-SPAWN_RADIUS, SPAWN_RADIUS),
            rng.uniform(-SPAWN_RADIUS, SPAWN_RADIUS),
        )
    )
    body.SetLinVel(
        chrono.ChVector3d(
            rng.uniform(-0.35, 0.35),
            rng.uniform(-0.35, 0.35),
            rng.uniform(-0.35, 0.35),
        )
    )
    body.SetAngVelParent(random_unit_vector(rng) * rng.uniform(0.5, 3.0))
    body.SetName(f"attracting_particle_{index:02d}")
    body.GetVisualShape(0).SetColor(
        chrono.ChColor(rng.uniform(0.2, 1.0), rng.uniform(0.2, 1.0), rng.uniform(0.2, 1.0))
    )
    system.Add(body)
    return body


def apply_pairwise_attraction(particles):
    """Apply softened inverse-square attraction between all emitted particles."""
    for body in particles:
        body.EmptyAccumulators()

    for i, body_a in enumerate(particles):
        pos_a = body_a.GetPos()  # cache: reused for all partners of body_a
        mass_a = body_a.GetMass()  # cache: reused for force scaling
        for body_b in particles[i + 1 :]:
            pos_b = body_b.GetPos()  # cache: used for symmetric force application
            delta = pos_b - pos_a
            dist_sq = delta.Dot(delta) + SOFTENING * SOFTENING
            dist = math.sqrt(dist_sq)
            direction = delta * (1.0 / dist)
            force_mag = ATTRACTION_G * mass_a * body_b.GetMass() / dist_sq
            force = direction * force_mag
            body_a.AccumulateForce(force, pos_a, False)
            body_b.AccumulateForce(force * -1.0, pos_b, False)


# === System & particles === NSC bodies with Bullet collision and custom attraction
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.GetSolver().AsIterative().SetMaxIterations(80)

contact_material = chrono.ChContactMaterialNSC()
contact_material.SetFriction(0.35)
contact_material.SetRestitution(0.15)

rng = random.Random(RNG_SEED)
particles = [make_particle(system, contact_material, rng, i) for i in range(8)]  # cache: initial emitted bodies
next_emit_time = EMIT_PERIOD


# === Visualization === Irrlicht window with camera, sky, lights, and reference grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle gravitational attraction")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, -5.0, 3.0), chrono.ChVector3d(0.0, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, -1.6), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === emit particles, apply attraction, render, and step dynamics
frame = 0

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()  # cache: used for emission, logging, and stop checks
            if len(particles) < PARTICLE_COUNT and sim_time >= next_emit_time:
                particles.append(make_particle(system, contact_material, rng, len(particles)))
                vis.BindAll()
                next_emit_time += EMIT_PERIOD

            apply_pairwise_attraction(particles)
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:  # file creation/write failures during review recording
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # Chrono stepping or invalid numeric state
    traceback.print_exc()
    raise
finally:
    pass
