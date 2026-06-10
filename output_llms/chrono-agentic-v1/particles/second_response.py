"""
Particle cluster simulation using ChRandomShapeCreatorSpheres with a Zhang distribution
for sphere diameters (average=0.6, minimum=0.23) and constant density 1600 kg/m³.
Sphere particles are emitted into a zero-gravity space and undergo mutual gravitational
attraction between all pairs using G=6.674e-3 (scaled for visibility, matching the
reference demo pattern). Each simulation step computes and prints the kinetic energy,
gravitational potential energy, and total mechanical energy. G_constant is defined
inside the simulation loop just before the energy calculations. System: ChSystemNSC.
"""

import math
import itertools
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
time_step        = 1e-2         # physics time step [s]
sim_end          = 10.0         # simulation duration [s]
render_fps       = 50.0         # review video frame rate [fps]
render_every     = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Particle emitter configuration
PARTICLES_PER_SEC = 200.0   # emission rate [particles/s]
MAX_PARTICLES     = 200     # total particle reservoir

# === System — zero gravity (gravitational attraction between bodies) ===
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # no external gravity
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)

# Cache collision system reference for BindItem calls in callback
coll = sys.GetCollisionSystem()  # cache: fetched once, used in callback

# === Visualization (built BEFORE the callback so vis is available in callback) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle Cluster — Spheres with Energy Calculation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    5.0, 5.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -15, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

# === Callback: bind new particles to Irrlicht + Bullet at creation time ===
class ParticleCreatorCallback(chrono.ChRandomShapeCreator_AddBodyCallback):
    """Bind each newly emitted particle to the visual system and collision system."""
    def __init__(self, vis_sys, coll_sys):
        chrono.ChRandomShapeCreator_AddBodyCallback.__init__(self)
        self._vis  = vis_sys   # cache: visual system reference
        self._coll = coll_sys  # cache: collision system reference

    def OnAddBody(self, body, coords, creator):
        # Register visual asset with Irrlicht so new particles appear
        body.GetVisualShape(0).SetTexture(
            chrono.GetChronoDataFile("textures/bluewhite.png")
        )
        self._vis.BindItem(body)
        # Register collision shape with Bullet broadphase
        self._coll.BindItem(body)
        # Disable gyroscopic torque for integrator stability
        body.SetUseGyroTorque(False)

creation_callback = ParticleCreatorCallback(vis, coll)

# === Particle emitter — ChRandomShapeCreatorSpheres ===
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(PARTICLES_PER_SEC)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(MAX_PARTICLES)

# Sphere shape creator: Zhang diameter distribution + constant density (as specified)
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(
    chrono.ChZhangDistribution(0.6, 0.23)     # average=0.6, minimum=0.23 [m]
)
mcreator_spheres.SetDensityDistribution(
    chrono.ChConstantDistribution(1600.0)      # constant density 1600 [kg/m³]
)
mcreator_spheres.SetAddVisualizationAsset(True)
mcreator_spheres.SetAddCollisionShape(True)
mcreator_spheres.RegisterAddBodyCallback(creation_callback)
emitter.SetParticleCreator(mcreator_spheres)

# Random positions in a cube volume
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
sampled_cube = chrono.ChBox(chrono.ChVector3d(50, 50, 50))
emitter_positions.SetGeometry(sampled_cube, chrono.ChFramed())
emitter.SetParticlePositioner(emitter_positions)

# Random alignment
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

# Random velocity in any direction with small magnitude
mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

# (No AddAccumulator needed in 9.0.0 — AccumulateForce/EmptyAccumulators are built in)

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            t = sys.GetChTime()   # cache: fetched once per step

            # Emit new sphere particles
            emitter.EmitParticles(sys, time_step)

            # Reset force accumulators on all bodies (9.0.0 API: EmptyAccumulators)
            for body in sys.GetBodies():
                body.EmptyAccumulators()

            # === Energy calculations ===
            # G_constant placed here inside the loop before energy calcs (per requirements)
            G_constant = 6.674e-3  # gravitational constant [scaled for visibility]

            bodies_list = sys.GetBodies()    # cache: fetch body list once per step
            body_pairs  = list(itertools.combinations(bodies_list, 2))  # precomputed once

            # Apply mutual gravitational attraction between each pair of bodies
            for b_a, b_b in body_pairs:
                D_attract = b_b.GetPos() - b_a.GetPos()
                r_attract = D_attract.Length()
                if r_attract > 1e-9:   # guard against coincident bodies
                    f_attract = G_constant * (b_a.GetMass() * b_b.GetMass()) / (r_attract * r_attract)
                    F_vec = (D_attract / r_attract) * f_attract
                    b_a.AccumulateForce(F_vec, b_a.GetPos(), False)
                    b_b.AccumulateForce(-F_vec, b_b.GetPos(), False)

            # Kinetic energy: sum 0.5 * m * |v|²
            kinetic_energy = 0.0
            for b in bodies_list:
                mass = b.GetMass()
                vel  = b.GetLinVel()
                kinetic_energy += 0.5 * mass * vel.Length2()

            # Gravitational potential energy between each pair: -G * mA * mB / r
            potential_energy = 0.0
            for b_a, b_b in body_pairs:
                pos_a = b_a.GetPos()
                pos_b = b_b.GetPos()
                dx = pos_a.x - pos_b.x
                dy = pos_a.y - pos_b.y
                dz = pos_a.z - pos_b.z
                r_attract = math.sqrt(dx * dx + dy * dy + dz * dz)
                if r_attract > 1e-9:   # guard against coincident bodies
                    potential_energy += -G_constant * (b_a.GetMass() * b_b.GetMass()) / r_attract

            total_energy = kinetic_energy + potential_energy
            num_particles = len(bodies_list)

            # Print energy diagnostics once per second
            step_idx = round(t / time_step)
            if step_idx % 100 == 0:
                print(
                    f"t={t:.2f}s  N={num_particles:3d}  "
                    f"KE={kinetic_energy:.4f}  PE={potential_energy:.4e}  "
                    f"E_tot={total_energy:.4f}"
                )


            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
