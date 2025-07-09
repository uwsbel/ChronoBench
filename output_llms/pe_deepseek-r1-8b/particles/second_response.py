import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

# Define the gravitational constant
G_constant = 6.674e-3  # Modified gravitational constant

class MyCreatorForAll(chrono.ChRandomShapeCreatorAddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        self.vis.BindItem(body)
        self.coll.BindItem(body)
        body.SetUseGyroTorque(False)

# Create a Chrono physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()

# Create a sphere body
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody)

# Create a particle emitter
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Randomizers for particle properties
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
emitter.SetParticlePositioner(emitter_positions)

emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

# Create the new particle shape creator
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_spheres)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle emitter demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

# Attach callback to the emitter
mcreation_callback = MyCreatorForAll(vis, coll)
emitter.RegisterAddBodyCallback(mcreation_callback)

# Simulation settings
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Initialize energy variables
total_kinetic_energy = 0.0
total_potential_energy = 0.0
total_energy = 0.0

stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Emit particles
    emitter.EmitParticles(sys, stepsize)

    # Reset accumulators for energy calculations
    total_kinetic_energy = 0.0
    total_potential_energy = 0.0

    # Calculate kinetic energy
    for body in sys.GetBodies():
        vel = body.GetVelocity()
        mass = body.GetMass()
        total_kinetic_energy += 0.5 * mass * vel.Length2()

    # Calculate potential energy
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        posA = abodyA.GetPos()
        posB = abodyB.GetPos()
        r = (posA - posB).Length()
        mass_A = abodyA.GetMass()
        mass_B = abodyB.GetMass()
        potential_energy = -G_constant * (mass_A * mass_B) / (r ** 2)
        total_potential_energy += potential_energy

    # Calculate total energy
    total_energy = total_kinetic_energy + total_potential_energy

    # Print energy values
    print(f"Step: {stepsize:.6f}, Total Kinetic Energy: {total_kinetic_energy:.6f}, Total Potential Energy: {total_potential_energy:.6f}, Total Energy: {total_energy:.6f}")

    # Perform simulation step
    sys.DoStepDynamics(stepsize)