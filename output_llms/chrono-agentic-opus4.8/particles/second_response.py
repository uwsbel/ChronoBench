"""Gravitational particle-attractor emitter (PyChrono 9.0.0, NSC, Irrlicht).

Models a swarm of small rigid spheres streamed into the world by a
ChParticleEmitter and mutually attracted by a custom Newtonian-gravity force
field. Each particle is a sphere whose diameter follows a Zhang distribution
(average 0.6, minimum 0.23) and whose density is a constant 1600 kg/m^3. Global
gravity is disabled; the only forces are the pairwise attractions applied by
hand each step via force accumulators.

System type: ChSystemNSC (rigid spheres, collisions enabled).
Main bodies : a fixed reference floor + the emitted attractor spheres.
Expected behavior: the emitter continuously injects spheres which then clump
together under mutual gravitational attraction. Each step prints the system's
kinetic, potential, and total energy so the (near-)conservation of total energy
can be inspected.
"""

import os
import math
import itertools
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / run configuration (no bare literals downstream)
time_step = 5e-3                 # integration step [s]
sim_end = 12.0                   # total simulated time [s]
render_fps = 50.0                # review video frame rate [fps]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

sphere_avg_diam = 0.6            # Zhang average sphere diameter [m]
sphere_min_diam = 0.23          # Zhang minimum sphere diameter [m]
sphere_density = 1600.0         # constant particle density [kg/m^3]

emit_rate = 2000.0              # particles emitted per second
outlet_y = 4.0                  # outlet height above origin [m]
init_speed = 0.30               # initial particle speed magnitude [m/s]
softening = 0.10                # gravitational softening length [m] (avoids singular r)

cam_eye = chrono.ChVector3d(0, 3, -9)     # Irrlicht camera position
cam_target = chrono.ChVector3d(0, 1, 0)   # Irrlicht camera target


# === System & gravity === NSC world with collisions; global gravity OFF (only pairwise attraction)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === a fixed visual floor for spatial reference; spheres are added by the emitter
floor_mat = chrono.ChContactMaterialNSC()
floor_mat.SetFriction(0.2)
floor = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, floor_mat)
floor.SetPos(chrono.ChVector3d(0, -3, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# === Particle emitter === spheres with Zhang diameter + constant density distributions
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(emit_rate)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Shape creator: random spheres sized by Zhang(avg, min), uniform constant density.
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(
    chrono.ChZhangDistribution(sphere_avg_diam, sphere_min_diam))
mcreator_spheres.SetDensityDistribution(
    chrono.ChConstantDistribution(sphere_density))
emitter.SetParticleCreator(mcreator_spheres)

# Positioner: spheres appear over a rectangular outlet placed above the scene.
emitter_positions = chrono.ChRandomParticlePositionRectangleOutlet()
emitter_positions.Outlet().pos = chrono.ChVector3d(0, outlet_y, 0)
emitter_positions.Outlet().rot = chrono.QuatFromAngleX(chrono.CH_PI_2)
emitter.SetParticlePositioner(emitter_positions)

# Velocity: small random initial speed in any direction.
emitter_velocities = chrono.ChRandomParticleVelocityAnyDirection()
emitter_velocities.SetModulusDistribution(chrono.ChConstantDistribution(init_speed))
emitter.SetParticleVelocity(emitter_velocities)


# === Emitter callback === tag every freshly-emitted sphere for the attraction loop
class AttractorBodyAdder(chrono.ChRandomShapeCreator_AddBodyCallback):
    """Collect each emitted body so the per-step gravity field can iterate them."""

    def __init__(self, registry):
        super().__init__()
        self.registry = registry  # cache: shared list, appended once per emission

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.6, 0.2))
        self.registry.append(body)


attractor_bodies = []
body_adder = AttractorBodyAdder(attractor_bodies)
mcreator_spheres.RegisterAddBodyCallback(body_adder)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gravitational particle attractor")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up scene
vis.Initialize()                                     # Initialize FIRST (Irrlicht)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(cam_eye, cam_target)
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -2.49, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only recording setup ===

# === Main loop === emit spheres, apply pairwise gravity, integrate, log energies
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            # Stream new particles into the world for this step.
            emitter.EmitParticles(sys, time_step)

            # Newtonian gravitational constant (defined here, just before the
            # energy / force computation, for locality with the attraction code).
            G_constant = 6.674e-3   # scaled gravitational constant [N*m^2/kg^2]

            bodies = attractor_bodies  # cache: the emitted dynamic spheres

            # Apply mutual attraction and accumulate potential energy.
            potential_energy = 0.0
            for body_a, body_b in itertools.combinations(bodies, 2):
                mass_a = body_a.GetMass()
                mass_b = body_b.GetMass()
                d_vec = body_b.GetPos() - body_a.GetPos()
                r_attract = math.sqrt(d_vec.Length2() + softening * softening)
                force_mag = G_constant * mass_a * mass_b / (r_attract * r_attract)
                f_vec = d_vec * (force_mag / r_attract)
                body_a.AccumulateForce(f_vec, body_a.GetPos(), False)
                body_b.AccumulateForce(-f_vec, body_b.GetPos(), False)
                potential_energy += -G_constant * (mass_a * mass_b) / r_attract

            # Kinetic energy over all dynamic spheres.
            kinetic_energy = 0.0
            for body in bodies:
                kinetic_energy += 0.5 * body.GetMass() * body.GetPosDt().Length2()

            total_energy = kinetic_energy + potential_energy
            print("t=%.3f  KE=%.6e  PE=%.6e  E=%.6e"
                  % (sys.GetChTime(), kinetic_energy, potential_energy, total_energy))


            sys.DoStepDynamics(time_step)
            for body in bodies:
                body.EmptyAccumulators()   # clear hand-applied forces before next step
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
