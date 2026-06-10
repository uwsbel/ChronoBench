"""Self-gravitating particle bodies in a PyChrono NSC system.

The simulation creates dynamic spherical rigid particles with
ChRandomShapeCreatorSpheres, assigns Zhang-distributed diameters and constant
density, applies pairwise attraction, and prints kinetic, potential, and total
energy during the run.
"""

import itertools
import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === values define the particle population and bounded run
time_step = 0.005
sim_end = 3.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
num_particles = 10
initial_speed = 0.75
softening_distance = 0.08


# === System & gravity === no uniform gravity; particles attract each other
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)


# === Particle creator === requested spherical creator and distributions
chrono.ChRandom_SetSeed(7)
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetAddVisualizationAsset(True)
mcreator_spheres.SetAddCollisionShape(True)
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))

initial_positions = [
    chrono.ChVector3d(-1.8, 0.0, -1.2),
    chrono.ChVector3d(-0.6, 0.0, -1.3),
    chrono.ChVector3d(0.7, 0.0, -1.1),
    chrono.ChVector3d(1.8, 0.0, -0.7),
    chrono.ChVector3d(-1.4, 0.0, 0.2),
    chrono.ChVector3d(-0.2, 0.0, 0.0),
    chrono.ChVector3d(1.0, 0.0, 0.2),
    chrono.ChVector3d(2.0, 0.0, 0.8),
    chrono.ChVector3d(-0.9, 0.0, 1.1),
    chrono.ChVector3d(0.7, 0.0, 1.2),
]

particles = []
for idx, pos in enumerate(initial_positions[:num_particles]):
    coords = chrono.ChCoordsysd(pos, chrono.QUNIT)
    body = mcreator_spheres.RandomGenerate(coords)
    body.SetName(f"sphere_particle_{idx}")
    theta = 2.0 * math.pi * idx / num_particles
    body.SetLinVel(chrono.ChVector3d(-initial_speed * math.sin(theta), 0.0, initial_speed * math.cos(theta)))
    particles.append(body)
    sys.AddBody(body)

particle_count = len(particles)  # cache: fixed after generation


# === Visualization === Irrlicht window with sky, lights, camera, and grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle sphere energy simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 4.2, -4.6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.8, 0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === update attraction forces, print requested energies, then step
frame = 0
last_kinetic_energy = 0.0
last_potential_energy = 0.0
last_total_energy = 0.0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame += 1

        for _ in range(render_every):
            for body in particles:
                body.EmptyAccumulators()

            G_constant = 0.04
            kinetic_energy = 0.0
            potential_energy = 0.0

            for body in particles:
                velocity = body.GetLinVel()  # cache: reused for kinetic energy
                kinetic_energy += 0.5 * body.GetMass() * velocity.Length2()

            for body_a, body_b in itertools.combinations(particles, 2):
                pos_a = body_a.GetPos()  # cache: pair separation and force point
                pos_b = body_b.GetPos()  # cache: pair separation and force point
                delta = pos_b - pos_a
                r_attract = max(delta.Length(), softening_distance)
                potential_energy += -G_constant * (body_a.GetMass() * body_b.GetMass()) / r_attract
                force_scale = G_constant * body_a.GetMass() * body_b.GetMass() / (r_attract * r_attract * r_attract)
                force_vec = delta * force_scale
                body_a.AccumulateForce(force_vec, pos_a, False)
                body_b.AccumulateForce(-force_vec, pos_b, False)

            total_energy = kinetic_energy + potential_energy
            last_kinetic_energy = kinetic_energy
            last_potential_energy = potential_energy
            last_total_energy = total_energy
            print(
                f"t={sys.GetChTime():.4f}  KE={kinetic_energy:.6f}  "
                f"PE={potential_energy:.6f}  Total={total_energy:.6f}"
            )

            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError, ArithmeticError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
