import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math

# ---------------------------------------------------------------------
#
#  Create the simulation system and set basic parameters
#
# ---------------------------------------------------------------------

# Initialize the PyChrono physical system
sys = chrono.ChSystemNSC()

# Disable the default global gravity, because we will apply a custom
# particle-to-particle gravitational attraction force manually.
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Set the solver and time stepper for a robust many-body simulation
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)


# ---------------------------------------------------------------------
#
#  Create the particle emitter
#
# ---------------------------------------------------------------------

# The ChParticleEmitter creates particles based on randomized properties:
# shape, position, velocity, orientation, and material.
emitter = chrono.ChParticleEmitter()

# Set how many particles are generated per second
emitter.SetParticlesPerSecond(2000)

# ---------------------------------------------------------------------
#  1) Randomizer for the particle creation position
# ---------------------------------------------------------------------
# Particles will spawn somewhere inside a box-shaped region.
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(
    chrono.ChBox(0.50, 0.50, 0.50),     # box of half-dims 0.5
    chrono.ChFramed()                   # centered at origin
)
emitter.SetParticlePositioner(emitter_positions)

# ---------------------------------------------------------------------
#  2) Randomizer for the alignment / orientation of each particle
# ---------------------------------------------------------------------
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

# ---------------------------------------------------------------------
#  3) Randomizer for the initial particle velocity
# ---------------------------------------------------------------------
mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

# ---------------------------------------------------------------------
#  4) Randomizer for the angular velocity (spin) of each particle
# ---------------------------------------------------------------------
mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

# ---------------------------------------------------------------------
#  5) Randomizer for the particle SHAPE / creator
# ---------------------------------------------------------------------
# We mix three different creators (spheres, boxes, cylinders) and pick
# one randomly for every new particle, each with its own probability.

# --- Sphere creator ---
creator_spheres = chrono.ChRandomShapeCreatorSpheres()
creator_spheres.SetDiameterDistribution(
    chrono.ChZhangDistribution(0.15, 0.06))   # avg, dispersion
creator_spheres.SetDensityDistribution(
    chrono.ChConstantDistribution(1000))

# --- Box creator ---
creator_boxes = chrono.ChRandomShapeCreatorBoxes()
creator_boxes.SetXsizeDistribution(
    chrono.ChZhangDistribution(0.20, 0.08))
creator_boxes.SetSizeRatioZDistribution(
    chrono.ChMinMaxDistribution(0.5, 1.0))
creator_boxes.SetSizeRatioYZDistribution(
    chrono.ChMinMaxDistribution(0.5, 1.0))
creator_boxes.SetDensityDistribution(
    chrono.ChConstantDistribution(1000))

# --- Cylinder creator ---
creator_cyls = chrono.ChRandomShapeCreatorCylinders()
creator_cyls.SetDiameterDistribution(
    chrono.ChZhangDistribution(0.20, 0.08))
creator_cyls.SetLengthFactorDistribution(
    chrono.ChMinMaxDistribution(0.5, 1.5))
creator_cyls.SetDensityDistribution(
    chrono.ChConstantDistribution(1000))

# --- Combine the shape creators ---
combined_creator = chrono.ChRandomShapeCreatorFromFamilies()
combined_creator.AddFamily(creator_spheres, 0.4)   # 40 % spheres
combined_creator.AddFamily(creator_boxes,   0.3)   # 30 % boxes
combined_creator.AddFamily(creator_cyls,    0.3)   # 30 % cylinders
combined_creator.Setup()

# Optional callback to assign a random color to each created body
class ParticleColorCallback(chrono.AddBodyCallback):
    def __init__(self):
        chrono.AddBodyCallback.__init__(self)

    def OnAddBody(self, body, coords, creator):
        vis_color = chrono.ChColor(random.random(),
                                   random.random(),
                                   random.random())
        body.GetVisualShape(0).SetColor(vis_color)

color_callback = ParticleColorCallback()
combined_creator.RegisterAddBodyCallback(color_callback)

# Assign the combined shape creator to the emitter
emitter.SetParticleCreator(combined_creator)


# ---------------------------------------------------------------------
#
#  Custom gravitational attraction between all particles
#
# ---------------------------------------------------------------------

# Universal "gravitational" constant (scaled up so the visual effect
# is clearly visible in the simulation timescale).
G_constant = 6.674e-3


def apply_attraction_forces(system):
    """Compute and apply mutual gravitational attraction between all bodies."""
    bodies = system.GetBodies()
    n = len(bodies)

    # First clear any previously accumulated forces
    for b in bodies:
        b.EmptyAccumulators()

    # Pairwise N-body attraction
    for i in range(n):
        bi = bodies[i]
        pi = bi.GetPos()
        mi = bi.GetMass()
        for j in range(i + 1, n):
            bj = bodies[j]
            pj = bj.GetPos()
            mj = bj.GetMass()

            # Direction and distance between the two particles
            dir_vec = pj - pi
            dist2 = dir_vec.Length2()
            if dist2 < 1e-6:
                continue
            dist = math.sqrt(dist2)
            dir_unit = dir_vec * (1.0 / dist)

            # Newtonian gravitational force magnitude
            force_mag = G_constant * mi * mj / dist2
            force_vec = dir_unit * force_mag

            # Apply equal-and-opposite forces at the centers of mass
            bi.AccumulateForce(force_vec,  pi, False)
            bj.AccumulateForce(-force_vec, pj, False)


# ---------------------------------------------------------------------
#
#  Create the Irrlicht visualization system
#
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravitational Attraction')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, 4, 4),    # camera position
              chrono.ChVector3d(0, 0, 0))    # look-at target
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(0, 5, 0), 10)


# ---------------------------------------------------------------------
#
#  Simulation loop
#
# ---------------------------------------------------------------------

time_step = 0.01

while vis.Run():
    # Continuously emit new particles into the system
    emitter.EmitParticles(sys, time_step)

    # Apply our custom gravitational attraction every frame
    apply_attraction_forces(sys)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the dynamics
    sys.DoStepDynamics(time_step)