import math
import random

import pychrono as chrono
import pychrono.irrlicht as irr


# ---------------------------------------------------------------------
# Compatibility helpers for different PyChrono versions
# ---------------------------------------------------------------------

Vec = chrono.ChVector3d if hasattr(chrono, "ChVector3d") else chrono.ChVectorD
Quat = chrono.ChQuaterniond if hasattr(chrono, "ChQuaterniond") else chrono.ChQuaternionD


def set_body_fixed(body, fixed):
    if hasattr(body, "SetFixed"):
        body.SetFixed(fixed)
    else:
        body.SetBodyFixed(fixed)


def set_body_velocity(body, vel):
    if hasattr(body, "SetPosDt"):
        body.SetPosDt(vel)
    else:
        body.SetPos_dt(vel)


def get_body_velocity(body):
    if hasattr(body, "GetPosDt"):
        return body.GetPosDt()
    else:
        return body.GetPos_dt()


def set_body_angular_velocity(body, omega):
    if hasattr(body, "SetAngVelParent"):
        body.SetAngVelParent(omega)
    elif hasattr(body, "SetWvel_par"):
        body.SetWvel_par(omega)


def enable_collision(body, enabled):
    if hasattr(body, "EnableCollision"):
        body.EnableCollision(enabled)
    elif hasattr(body, "SetCollide"):
        body.SetCollide(enabled)


def set_force(body, force):
    if hasattr(body, "SetForce"):
        body.SetForce(force)
    else:
        body.EmptyAccumulators()
        body.AccumulateForce(force, body.GetPos(), False)


def set_torque(body, torque):
    if hasattr(body, "SetTorque"):
        body.SetTorque(torque)


def random_unit_quaternion():
    """
    Generate a uniformly random unit quaternion.
    """
    u1 = random.random()
    u2 = random.random()
    u3 = random.random()

    qx = math.sqrt(1.0 - u1) * math.sin(2.0 * math.pi * u2)
    qy = math.sqrt(1.0 - u1) * math.cos(2.0 * math.pi * u2)
    qz = math.sqrt(u1) * math.sin(2.0 * math.pi * u3)
    qw = math.sqrt(u1) * math.cos(2.0 * math.pi * u3)

    return Quat(qw, qx, qy, qz)


def random_point_in_sphere(radius):
    """
    Return a random point inside a sphere of given radius.
    """
    while True:
        x = random.uniform(-radius, radius)
        y = random.uniform(-radius, radius)
        z = random.uniform(-radius, radius)
        if x * x + y * y + z * z <= radius * radius:
            return x, y, z


# ---------------------------------------------------------------------
# Particle emitter
# ---------------------------------------------------------------------

class ParticleEmitter:
    def __init__(
        self,
        system,
        particle_list,
        emission_rate=15.0,
        max_particles=120,
        emitter_radius=3.0,
        density=120.0,
    ):
        self.system = system
        self.particles = particle_list
        self.emission_rate = emission_rate
        self.max_particles = max_particles
        self.emitter_radius = emitter_radius
        self.density = density
        self.emit_accumulator = 0.0

    def update(self, dt):
        if len(self.particles) >= self.max_particles:
            return

        self.emit_accumulator += self.emission_rate * dt

        while self.emit_accumulator >= 1.0 and len(self.particles) < self.max_particles:
            self.emit_one()
            self.emit_accumulator -= 1.0

    def emit_one(self):
        shape_type = random.choice(["sphere", "box", "cylinder"])

        body = chrono.ChBody()
        set_body_fixed(body, False)
        enable_collision(body, False)

        # Random position inside the emitter sphere
        x, y, z = random_point_in_sphere(self.emitter_radius)
        body.SetPos(Vec(x, y, z))

        # Random orientation
        body.SetRot(random_unit_quaternion())

        # Random velocity with a weak swirl component around the Z axis
        r_xy = math.sqrt(x * x + y * y) + 1e-9
        tangent_x = -y / r_xy
        tangent_y = x / r_xy

        swirl_speed = random.uniform(0.2, 0.9)
        random_speed = 0.35

        vx = swirl_speed * tangent_x + random.uniform(-random_speed, random_speed)
        vy = swirl_speed * tangent_y + random.uniform(-random_speed, random_speed)
        vz = random.uniform(-random_speed, random_speed)

        set_body_velocity(body, Vec(vx, vy, vz))

        # Random angular velocity
        set_body_angular_velocity(
            body,
            Vec(
                random.uniform(-2.0, 2.0),
                random.uniform(-2.0, 2.0),
                random.uniform(-2.0, 2.0),
            ),
        )

        # Random visual color
        color = chrono.ChColor(
            random.uniform(0.3, 1.0),
            random.uniform(0.3, 1.0),
            random.uniform(0.3, 1.0),
        )

        # Create a random shape and assign approximate mass/inertia
        if shape_type == "sphere":
            radius = random.uniform(0.08, 0.18)
            volume = 4.0 / 3.0 * math.pi * radius ** 3
            mass = self.density * volume
            inertia = 2.0 / 5.0 * mass * radius ** 2

            body.SetMass(mass)
            body.SetInertiaXX(Vec(inertia, inertia, inertia))

            visual = chrono.ChVisualShapeSphere(radius)
            visual.SetColor(color)
            body.AddVisualShape(visual)

        elif shape_type == "box":
            sx = random.uniform(0.12, 0.30)
            sy = random.uniform(0.12, 0.30)
            sz = random.uniform(0.12, 0.30)

            volume = sx * sy * sz
            mass = self.density * volume

            ixx = mass / 12.0 * (sy * sy + sz * sz)
            iyy = mass / 12.0 * (sx * sx + sz * sz)
            izz = mass / 12.0 * (sx * sx + sy * sy)

            body.SetMass(mass)
            body.SetInertiaXX(Vec(ixx, iyy, izz))

            try:
                visual = chrono.ChVisualShapeBox(Vec(sx, sy, sz))
            except TypeError:
                visual = chrono.ChVisualShapeBox(sx, sy, sz)

            visual.SetColor(color)
            body.AddVisualShape(visual)

        else:
            radius = random.uniform(0.07, 0.14)
            height = random.uniform(0.16, 0.35)

            volume = math.pi * radius ** 2 * height
            mass = self.density * volume

            # Approximate inertia for a cylinder aligned with its local Y axis
            ixx = mass / 12.0 * (3.0 * radius ** 2 + height ** 2)
            iyy = 0.5 * mass * radius ** 2
            izz = ixx

            body.SetMass(mass)
            body.SetInertiaXX(Vec(ixx, iyy, izz))

            visual = chrono.ChVisualShapeCylinder(radius, height)
            visual.SetColor(color)
            body.AddVisualShape(visual)

        self.system.Add(body)
        self.particles.append(body)


# ---------------------------------------------------------------------
# Custom pairwise gravity
# ---------------------------------------------------------------------

def apply_pairwise_gravity(
    particles,
    gravitational_constant=2.5,
    softening_length=0.15,
    velocity_damping=0.015,
):
    """
    Apply custom Newtonian-like pairwise attraction.

    F_ij = G * m_i * m_j * r_ij / (|r_ij|^2 + eps^2)^(3/2)

    softening_length prevents singular forces when particles get very close.
    velocity_damping adds mild numerical damping to keep the particle cloud stable.
    """
    n = len(particles)

    forces = [[0.0, 0.0, 0.0] for _ in range(n)]
    soft2 = softening_length * softening_length

    for i in range(n):
        bi = particles[i]
        pi = bi.GetPos()
        mi = bi.GetMass()

        for j in range(i + 1, n):
            bj = particles[j]
            pj = bj.GetPos()
            mj = bj.GetMass()

            dx = pj.x - pi.x
            dy = pj.y - pi.y
            dz = pj.z - pi.z

            dist2 = dx * dx + dy * dy + dz * dz + soft2
            inv_dist = 1.0 / math.sqrt(dist2)
            inv_dist3 = inv_dist * inv_dist * inv_dist

            fscale = gravitational_constant * mi * mj * inv_dist3

            fx = fscale * dx
            fy = fscale * dy
            fz = fscale * dz

            forces[i][0] += fx
            forces[i][1] += fy
            forces[i][2] += fz

            forces[j][0] -= fx
            forces[j][1] -= fy
            forces[j][2] -= fz

    for i, body in enumerate(particles):
        vel = get_body_velocity(body)
        mass = body.GetMass()

        # Mild linear damping
        forces[i][0] += -velocity_damping * mass * vel.x
        forces[i][1] += -velocity_damping * mass * vel.y
        forces[i][2] += -velocity_damping * mass * vel.z

        set_force(body, Vec(forces[i][0], forces[i][1], forces[i][2]))
        set_torque(body, Vec(0.0, 0.0, 0.0))


# ---------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------

def main():
    random.seed(7)

    # Create Chrono physical system
    system = chrono.ChSystemNSC()

    # Disable global gravity because we apply custom particle-particle gravity
    system.Set_G_acc(Vec(0.0, 0.0, 0.0))

    # Solver and timestep settings
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.GetSolver().SetMaxIterations(80)

    timestep = 0.005

    particles = []

    # Create emitter
    emitter = ParticleEmitter(
        system=system,
        particle_list=particles,
        emission_rate=18.0,
        max_particles=140,
        emitter_radius=3.0,
        density=120.0,
    )

    # Emit an initial cloud of particles
    for _ in range(45):
        emitter.emit_one()

    # Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Particle Gravity Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(Vec(0.0, -8.0, 4.0), Vec(0.0, 0.0, 0.0))
    vis.AddTypicalLights()

    # Simulation loop
    while vis.Run():
        # Emit new particles over time
        emitter.update(timestep)

        # Apply custom particle-particle gravitational attraction
        apply_pairwise_gravity(
            particles,
            gravitational_constant=2.5,
            softening_length=0.18,
            velocity_damping=0.02,
        )

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(timestep)


if __name__ == "__main__":
    main()