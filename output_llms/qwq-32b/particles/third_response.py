import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
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

# Create three spheres for the three-body problem
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

# Sphere 1
sphere1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere1.SetPos(chrono.ChVectorD(1, 1, 0))
sphere1.SetPos_dt(chrono.ChVectorD(0.5, 0, 0.1))  # Initial velocity
sphere1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere1)
vis.BindItem(sphere1)
coll.BindItem(sphere1)

# Sphere 2
sphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere2.SetPos(chrono.ChVectorD(-10, -10, 0))
sphere2.SetPos_dt(chrono.ChVectorD(-0.5, 0, -0.1))  # Initial velocity
sys.Add(sphere2)
vis.BindItem(sphere2)
coll.BindItem(sphere2)

# Sphere 3
sphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere3.SetPos(chrono.ChVectorD(0, 20, 0))
sphere3.SetPos_dt(chrono.ChVectorD(0, -0.5, 0.2))  # Initial velocity
sys.Add(sphere3)
vis.BindItem(sphere3)
coll.BindItem(sphere3)

# Create an emitter (unchanged except fixing vector type)
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Randomizers for particle properties (corrected vector type)
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFrameD())
emitter.SetParticlePositioner(emitter_positions)

emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_spheres)

# Create the Irrlicht visualization (corrected vector type)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-Body Simulation with Particle Emitter')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 14, -20))  # Corrected vector type
vis.AddTypicalLights()

mcreation_callback = MyCreatorForAll(vis, coll)
emitter.RegisterAddBodyCallback(mcreation_callback)

sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))

# Simulation loop
stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    emitter.EmitParticles(sys, stepsize)

    for body in sys.GetBodies():
        body.EmptyAccumulators()

    G_constant = 6.674e-3  # Modified gravitational constant

    # Calculate kinetic energy
    kinetic_energy = 0
    for body in sys.GetBodies():
        mass = body.GetMass()
        velocity = body.GetPos_dt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    # Calculate potential energy
    potential_energy = 0
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        if r_attract > 0:  # Avoid division by zero
            potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    # Apply gravitational forces
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        if r_attract > 0:
            f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)
            F_attract = (D_attract / r_attract) * f_attract
            abodyA.AddForce(F_attract)
            abodyB.AddForce(-F_attract)

    total_energy = kinetic_energy + potential_energy
    print(
        f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {total_energy:.6f}")

    sys.DoStepDynamics(stepsize)