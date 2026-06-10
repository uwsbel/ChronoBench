"""Gravitational attraction between emitted particles (PyChrono 9.0.1, Irrlicht).

Models a self-gravitating cloud of rigid particles. A ChParticleEmitter fills the
scene with bodies of RANDOM shapes (spheres and boxes), random positions over a
rectangular outlet, random initial velocities, and random orientations. World
gravity is disabled; instead a custom Newtonian pairwise attraction force is applied
to every particle each step via per-body force accumulators, so the cloud clumps
together under its own mutual attraction.

System type: NSC (rigid, impulsive contact). Particles collide with one another,
so a Bullet collision system is required. Main bodies: emitter-created particle
rigid bodies. Expected behavior: particles spawn, drift, and are pulled toward the
cloud's center of mass, condensing into clusters over the run.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / emission parameters
time_step = 5e-3            # physics time step [s]
sim_end = 12.0              # total simulated time [s]
render_fps = 50.0          # review render cadence [frames/s]

emit_rate = 12.0           # particles created per second — sparse, so spawns never overlap
emit_until = 4.5           # emit gradually over the first part of the run, then let it settle [s]
max_particles = 55         # cap cloud size so the N^2 force loop stays bounded & stable
outlet_center = chrono.ChVector3d(0.0, 0.0, 2.2)  # center of the emission rectangle (above the grid)
outlet_width = 2.0         # emission rectangle width  (local X) [m] — compact cloud where 1/r^2 is strong
outlet_height = 2.0        # emission rectangle height (local Y) [m]

part_density = 1000.0      # particle material density [kg/m^3]
sph_dia_min, sph_dia_max = 0.18, 0.32   # random sphere diameter range [m]
box_size_min, box_size_max = 0.18, 0.30 # random box X-size range [m]
speed_min, speed_max = 0.0, 0.02        # random initial speed range [m/s] — released near rest

grav_const = 8.0e-2        # tuned attraction constant (NOT real G; strong, visible collapse)
softening = 0.4            # softening length [m] — close-range cutoff (~ a particle diameter)
damp_coeff = 0.8           # linear drag [1/s] — dissipates kinetic energy so the cloud collapses & settles
max_speed = 1.5            # clamp particle speed [m/s] — bounds contact impulses, keeps solver stable
part_family = 1            # shared collision family for all particles

# === Derived constants (precomputed once) ===
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
soft_sq = softening * softening                               # precomputed once: softening^2

# === System & gravity === NSC system; world gravity OFF so only mutual attraction acts
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # no world gravity — attraction only
# Particles carry collision shapes and collide with each other -> Bullet collision required.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)

# === Contact material === shared NSC material for every emitted particle
part_mat = chrono.ChContactMaterialNSC()
part_mat.SetFriction(0.3)
part_mat.SetRestitution(0.0)

# === Particle bookkeeping === collected by the emitter's add-body callback
particles = []   # list of (body, accumulator_index) for the attraction force loop


class ParticleRegistrar(chrono.ChRandomShapeCreator_AddBodyCallback):
    """Called once per emitted body: register it in the system, give it a force
    accumulator for the custom attraction force, and record it for the force loop.
    (In this build EmitParticles does NOT auto-add bodies, so we add them here.)"""

    def __init__(self, system, material, registry, family):
        super().__init__()
        self.system = system
        self.material = material
        self.registry = registry
        self.family = family

    def OnAddBody(self, body, coords, creator):
        body.SetFixed(False)                 # particles are free to move under attraction
        body.EnableCollision(True)           # collision ON: particles pile up under attraction
        # All particles share one collision family (Bullet collision system handles them
        # together). Collision is kept ON so attracting particles STACK into a clump instead
        # of passing through one another (pass-through under explicit integration would heat
        # the cloud and blow it apart). Emission is sparse enough that spawns never overlap.
        body.GetCollisionModel().SetFamily(self.family)
        acc = body.AddAccumulator()          # one accumulator holds the per-step custom force
        self.system.Add(body)                # register the freshly created particle body
        self.registry.append((body, acc))


# === Particle emitter === random shapes / positions / velocities / orientations
emitter = chrono.ChParticleEmitter()
emitter.SetFlowControlMode(chrono.ChParticleEmitter.FLOW_PARTICLESPERSECOND)
emitter.SetParticlesPerSecond(emit_rate)

# Random position: uniform over a rectangular outlet plane centered at outlet_center.
positioner = chrono.ChRandomParticlePositionRectangleOutlet()
outlet_frame = positioner.Outlet()                 # mutable reference to the outlet coordsys
outlet_frame.pos = outlet_center
outlet_frame.rot = chrono.QuatFromAngleX(chrono.CH_PI_2)  # face the rectangle so particles fill XZ-ish volume
positioner.OutletWidth = outlet_width
positioner.OutletHeight = outlet_height
emitter.SetParticlePositioner(positioner)

# Random velocity: any direction, modulus uniformly distributed (released near rest).
velocity = chrono.ChRandomParticleVelocityAnyDirection()
velocity.SetModulusDistribution(chrono.ChUniformDistribution(speed_min, speed_max))
emitter.SetParticleVelocity(velocity)

# Random orientation: uniform alignment over the sphere of rotations.
emitter.SetParticleAligner(chrono.ChRandomParticleAlignmentUniform())

# Random shapes: a family mix of spheres and boxes -> "random shapes".
sphere_creator = chrono.ChRandomShapeCreatorSpheres()
sphere_creator.SetDiameterDistribution(chrono.ChUniformDistribution(sph_dia_min, sph_dia_max))
sphere_creator.SetDensityDistribution(chrono.ChConstantDistribution(part_density))

box_creator = chrono.ChRandomShapeCreatorBoxes()
box_creator.SetXsizeDistribution(chrono.ChUniformDistribution(box_size_min, box_size_max))
box_creator.SetSizeRatioZDistribution(chrono.ChUniformDistribution(0.7, 1.3))
box_creator.SetSizeRatioYZDistribution(chrono.ChUniformDistribution(0.7, 1.3))
box_creator.SetDensityDistribution(chrono.ChConstantDistribution(part_density))

shape_creator = chrono.ChRandomShapeCreatorFromFamilies()
shape_creator.AddFamily(sphere_creator, 0.5)   # 50% spheres
shape_creator.AddFamily(box_creator, 0.5)      # 50% boxes
shape_creator.Setup()
shape_creator.SetAddCollisionShape(True)        # particles collide
shape_creator.SetAddVisualizationAsset(True)    # particles are drawn
emitter.SetParticleCreator(shape_creator)

# Register the add-body callback so every emitted particle is added + tracked.
registrar = ParticleRegistrar(sys, part_mat, particles, part_family)
emitter.RegisterAddBodyCallback(registrar)

# === Visualization === full Irrlicht scene: window + logo + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Self-gravitating particle cloud")
vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, -6, 3), chrono.ChVector3d(0, 0, 1))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid


def apply_particle_forces(bodies, g_const, soft2, damp):
    """Load each particle's accumulator for this step with two custom forces:
      1. Pairwise Newtonian attraction  F = g_const*m_i*m_j/(r^2+soft2)  along i--j.
         Applied equal-and-opposite, so it is momentum-conserving and pulls the
         cloud toward its own center of mass (the requested attraction dynamics).
      2. A small linear drag  -damp*m*v  per particle, which dissipates the kinetic
         energy injected by random spawn velocities and close encounters so the
         cloud settles into a bound clump instead of dispersing."""
    n = len(bodies)
    for i in range(n):
        bodies[i][0].EmptyAccumulator(bodies[i][1])   # clear last step's accumulated force
    for i in range(n):
        body_i, acc_i = bodies[i]
        pos_i = body_i.GetPos()
        mass_i = body_i.GetMass()
        # Pairwise attraction (upper triangle, add opposite to the partner).
        for j in range(i + 1, n):
            body_j, acc_j = bodies[j]
            d = body_j.GetPos() - pos_i               # vector i -> j
            dist2 = d.x * d.x + d.y * d.y + d.z * d.z
            inv_len = 1.0 / math.sqrt(dist2 + soft2)  # softened inverse distance
            mag = g_const * mass_i * body_j.GetMass() * inv_len * inv_len * inv_len
            fx, fy, fz = d.x * mag, d.y * mag, d.z * mag
            body_i.AccumulateForce(acc_i, chrono.ChVector3d(fx, fy, fz), pos_i, False)
            body_j.AccumulateForce(acc_j, chrono.ChVector3d(-fx, -fy, -fz),
                                   body_j.GetPos(), False)   # Newton's 3rd law
        # Per-body linear drag (dissipative, keeps the cloud bound).
        v = body_i.GetPosDt()
        body_i.AccumulateForce(acc_i,
                               chrono.ChVector3d(-damp * mass_i * v.x,
                                                 -damp * mass_i * v.y,
                                                 -damp * mass_i * v.z),
                               pos_i, False)


def clamp_speeds(bodies, v_max):
    """Cap each particle's linear speed. A near-coincident pair (at spawn, or while the
    cloud is densely collapsing) can receive a large impulsive contact separation from the
    NSC solver; without a cap that velocity integrates to a diverging position. Clamping
    bounds those impulses and keeps the solver stable while attraction + drag govern the
    bulk collapse of the cloud."""
    v_max_sq = v_max * v_max
    for body, _acc in bodies:
        v = body.GetPosDt()
        sp_sq = v.x * v.x + v.y * v.y + v.z * v.z
        if sp_sq > v_max_sq:
            scale = v_max / math.sqrt(sp_sq)
            body.SetPosDt(chrono.ChVector3d(v.x * scale, v.y * scale, v.z * scale))


def remove_net_drift(bodies):
    """Subtract the mass-weighted mean velocity from every particle, i.e. view the cloud
    in its own center-of-mass frame. Internal attraction conserves momentum, but the
    random spawn velocities and the speed clamp leave a small net drift; cancelling it
    each step keeps the self-gravitating cloud collapsing IN PLACE (the requested
    attraction) rather than translating off-screen."""
    total_m = 0.0
    px = py = pz = 0.0
    for body, _acc in bodies:
        m = body.GetMass(); v = body.GetPosDt()
        total_m += m; px += m * v.x; py += m * v.y; pz += m * v.z
    if total_m <= 0.0:
        return
    vmx, vmy, vmz = px / total_m, py / total_m, pz / total_m   # cloud COM velocity
    for body, _acc in bodies:
        v = body.GetPosDt()
        body.SetPosDt(chrono.ChVector3d(v.x - vmx, v.y - vmy, v.z - vmz))


# === Main loop === emit early, then evolve cloud under mutual attraction

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            # Emit a fresh batch only during the early window and until the cap is reached.
            if t <= emit_until and len(particles) < max_particles:
                emitter.EmitParticles(sys, time_step)
                # Newly emitted bodies need an explicit Irrlicht bind to be visible.
                vis.BindAll()   # bind just-emitted particle visuals (BindAll re-binds safely)
            # Custom gravitational attraction (+ light drag) between all live particles.
            if particles:
                apply_particle_forces(particles, grav_const, soft_sq, damp_coeff)
                clamp_speeds(particles, max_speed)
                remove_net_drift(particles)
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid particle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
