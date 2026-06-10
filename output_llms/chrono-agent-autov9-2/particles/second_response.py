"""Self-gravitating particle cloud emitted as random spheres (PyChrono, NSC).

Models a population of rigid spheres dropped from a rectangular outlet by a
ChParticleEmitter. Each sphere's diameter is drawn from a Zhang distribution and
its material density from a constant distribution, so the cloud has a realistic
spread of sizes/masses. The particles are released from rest (no inherited or
random launch velocity); world gravity is disabled so the only driver of motion
is the mutual self-gravity below.

A softened Newtonian pairwise attraction is applied between every pair of bodies
each step via per-body force accumulators; softening (a Plummer-style radius) and
a small linear velocity damping keep the cluster bounded instead of collapsing to
a singularity. The system is NSC with Bullet collision so spheres also contact
one another.

Expected behavior: spheres spawn over the first second, then gradually draw
together under their mutual gravity into a loose clump while colliding. The
kinetic energy, gravitational potential energy, and their sum are computed and
printed every step to monitor the energy budget of the self-gravitating cloud.
"""

import os
import math
import itertools

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants and derived launch state
time_step = 5e-3                 # integration step [s]
sim_end = 8.0                    # total simulated time [s]
render_fps = 50.0                # review-video frame rate
emit_rate = 40.0                 # particles created per second
emit_window = 1.0                # emit only during the first second, then evolve
outlet_z = 3.0                   # outlet height above origin [m]
sphere_density = 1600.0          # constant material density [kg/m^3]
zhang_mean = 0.6                 # Zhang distribution mean diameter [m]
zhang_min = 0.23                 # Zhang distribution minimum diameter [m]
G_constant = 2.0e-2              # scaled gravitational constant (visible clustering)
softening = 0.35                 # Plummer softening length [m] (bounds 1/r force)
lin_damping = 0.6                # linear velocity damping coefficient [1/s]
friction = 0.6                   # inter-particle friction
restitution = 0.0                # inter-particle restitution (inelastic -> cohesive)
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
soft2 = softening * softening                                 # precomputed once

# === System & gravity === NSC + Bullet collision; world gravity OFF so the only
# attraction is the pairwise self-gravity below (particles released from rest).
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(120)

# === Contact material === shared NSC material for every emitted sphere
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(friction)
mat.SetRestitution(restitution)

# === Particle emitter === random spheres, Zhang diameters, constant density
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(emit_rate)
emitter.SetUseParticleReservoir(False)

mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(zhang_mean, zhang_min))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(sphere_density))
mcreator_spheres.SetAddCollisionShape(True)
mcreator_spheres.SetAddVisualizationAsset(True)
emitter.SetParticleCreator(mcreator_spheres)

# Rectangular outlet positioned above the origin, opening downward.
positioner = chrono.ChRandomParticlePositionRectangleOutlet()
positioner.Outlet().pos = chrono.ChVector3d(0, 0, outlet_z)
positioner.Outlet().rot = chrono.QuatFromAngleX(chrono.CH_PI_2)
emitter.SetParticlePositioner(positioner)

# Release from rest: zero launch velocity and no inherited speed -> pure dynamics.
vel_zero = chrono.ChRandomParticleVelocityConstantDirection()
vel_zero.SetDirection(chrono.ChVector3d(0, 0, 0))
emitter.SetParticleVelocity(vel_zero)
emitter.SetInheritSpeed(False)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Self-gravitating emitted sphere cloud")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(7, -7, 4), chrono.ChVector3d(0, 0, 1.5))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Pairwise gravity helper === softened Newtonian attraction via accumulators
def apply_pairwise_gravity(bodies):
    """Accumulate a softened mutual gravitational force on every body pair.

    Uses one force accumulator per body (added once on creation). Softening
    (soft2) caps the 1/r^2 singularity; a linear damping term bleeds energy so
    the cloud clusters into a bounded clump rather than slingshotting apart.
    """
    for b in bodies:
        b.EmptyAccumulator(0)                       # clear last step's force
    for ba, bb in itertools.combinations(bodies, 2):
        d = bb.GetPos() - ba.GetPos()
        r2 = d.Length2() + soft2                    # softened squared distance
        inv_r3 = 1.0 / (r2 * math.sqrt(r2))
        f = d * (G_constant * ba.GetMass() * bb.GetMass() * inv_r3)
        ba.AccumulateForce(0, f, ba.GetPos(), False)
        bb.AccumulateForce(0, -f, bb.GetPos(), False)
    for b in bodies:
        drag = b.GetPosDt() * (-lin_damping * b.GetMass())
        b.AccumulateForce(0, drag, b.GetPos(), False)


def potential_energy(bodies):
    """Total softened gravitational PE summed over all pairs."""
    pe = 0.0
    for ba, bb in itertools.combinations(bodies, 2):
        r_attract = math.sqrt((bb.GetPos() - ba.GetPos()).Length2() + soft2)
        pe += -G_constant * (ba.GetMass() * bb.GetMass()) / r_attract
    return pe


def kinetic_energy(bodies):
    """Total translational kinetic energy of all bodies."""
    return sum(0.5 * b.GetMass() * b.GetPosDt().Length2() for b in bodies)


# === Main loop === emit, accumulate gravity, step, and report the energy budget


frame = 0
known = 0          # count of bodies already given an accumulator + bound
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            if t < emit_window:
                emitter.EmitParticles(sys, time_step)   # adds bodies directly
                for b in sys.GetBodies()[known:]:       # only newly emitted bodies
                    b.AddAccumulator()                  # one accumulator per body
                    vis.BindItem(b)                     # bind so it renders
                known = len(sys.GetBodies())

            bodies = sys.GetBodies()
            apply_pairwise_gravity(bodies)

            ke = kinetic_energy(bodies)
            pe = potential_energy(bodies)
            total = ke + pe
            print(f"t={t:6.3f}  N={len(bodies):3d}  KE={ke:12.4f}  PE={pe:12.4f}  E={total:12.4f}")


            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
