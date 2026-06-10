"""Particle attraction demo using PyChrono NSC rigid bodies.

This simulation emits a deterministic cloud of mixed rigid particles, including
spheres, boxes, and cylinders, then applies pairwise softened gravitational
attraction between every particle. The particles begin with random positions,
velocities, and orientations, collide through Bullet contact, and are visualized
in an Irrlicht window while the attraction pulls them into a dynamic cluster.
"""

import math
import random
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants and cached setup values ===
PARTICLE_COUNT = 14
TIME_STEP = 0.003
SIM_END = 8.0
RENDER_FPS = 30.0
SOFTENING = 0.18
ATTRACTION_G = 0.018
SPAWN_RADIUS = 1.8


def vec_length(vec):
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)


def random_unit_vector(rng):
    theta = rng.uniform(0.0, 2.0 * math.pi)
    z = rng.uniform(-1.0, 1.0)
    r = math.sqrt(max(0.0, 1.0 - z * z))
    return chrono.ChVector3d(r * math.cos(theta), r * math.sin(theta), z)


def set_visual_color(body, color):
    shape = body.GetVisualShape(0)  # cache: first factory visual reused once
    shape.SetColor(color)


def make_particle(system, material, rng, index):
    shape_kind = index % 3
    mass = rng.uniform(0.8, 2.2)
    radius = rng.uniform(0.08, 0.16)

    if shape_kind == 0:
        volume = (4.0 / 3.0) * math.pi * radius**3
        body = chrono.ChBodyEasySphere(radius, mass / volume, True, True, material)
    elif shape_kind == 1:
        sx = rng.uniform(0.14, 0.26)
        sy = rng.uniform(0.12, 0.24)
        sz = rng.uniform(0.12, 0.24)
        body = chrono.ChBodyEasyBox(sx, sy, sz, mass / (sx * sy * sz), True, True, material)
    else:
        height = rng.uniform(0.18, 0.34)
        volume = math.pi * radius**2 * height
        body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radius, height, mass / volume, True, True, material)

    position = random_unit_vector(rng)
    radial = rng.uniform(0.25, SPAWN_RADIUS)
    body.SetPos(chrono.ChVector3d(position.x * radial, position.y * radial, position.z * radial))

    axis = random_unit_vector(rng)
    body.SetRot(chrono.QuatFromAngleAxis(rng.uniform(0.0, 2.0 * math.pi), axis))
    body.SetLinVel(chrono.ChVector3d(rng.uniform(-0.18, 0.18), rng.uniform(-0.18, 0.18), rng.uniform(-0.18, 0.18)))
    body.SetAngVelParent(chrono.ChVector3d(rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)))
    body.EnableCollision(True)

    color = chrono.ChColor(rng.uniform(0.25, 0.95), rng.uniform(0.25, 0.95), rng.uniform(0.25, 0.95))
    set_visual_color(body, color)

    system.Add(body)
    return {"body": body, "mass": mass}


def update_attraction_forces(particles):
    net_forces = [chrono.ChVector3d(0, 0, 0) for _ in particles]

    for i in range(len(particles)):
        body_i = particles[i]["body"]  # cache: reused for pair state
        pos_i = body_i.GetPos()
        mass_i = particles[i]["mass"]
        for j in range(i + 1, len(particles)):
            body_j = particles[j]["body"]  # cache: reused for pair state
            pos_j = body_j.GetPos()
            dx = pos_j.x - pos_i.x
            dy = pos_j.y - pos_i.y
            dz = pos_j.z - pos_i.z
            distance_squared = dx * dx + dy * dy + dz * dz + SOFTENING * SOFTENING
            distance = math.sqrt(distance_squared)
            magnitude = ATTRACTION_G * mass_i * particles[j]["mass"] / distance_squared
            fx = magnitude * dx / distance
            fy = magnitude * dy / distance
            fz = magnitude * dz / distance
            net_forces[i] = chrono.ChVector3d(net_forces[i].x + fx, net_forces[i].y + fy, net_forces[i].z + fz)
            net_forces[j] = chrono.ChVector3d(net_forces[j].x - fx, net_forces[j].y - fy, net_forces[j].z - fz)

    for particle, force_vec in zip(particles, net_forces):
        body = particle["body"]  # cache: accumulator target reused for this step
        body.EmptyAccumulators()
        body.AccumulateForce(force_vec, body.GetPos(), False)


# === System and particle bodies ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)

contact_material = chrono.ChContactMaterialNSC()
contact_material.SetFriction(0.35)
contact_material.SetRestitution(0.25)

rng = random.Random(17)
particles = [make_particle(sys, contact_material, rng, i) for i in range(PARTICLE_COUNT)]


# === Irrlicht visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle gravitational attraction")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.8, -5.2, 3.1), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 16, 16, chrono.ChCoordsysd(), chrono.ChColor(0.35, 0.35, 0.35))


# === Main loop with custom gravitational attraction ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        update_attraction_forces(particles)
        sys.DoStepDynamics(TIME_STEP)

except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
