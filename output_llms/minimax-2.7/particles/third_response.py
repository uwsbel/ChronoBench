import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.particle as particle            # <-- added import for particle module
from itertools import combinations

# -----------------------------------------------------------------
# Callback class used to customize each emitted particle
# -----------------------------------------------------------------
class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        # Give a texture to the emitted particle
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        # Bind to visual and collision systems
        self.vis.BindItem(body)
        self.coll.BindItem(body)
        # Disable gyro torque (not needed here)
        body.SetUseGyroTorque(False)

# -----------------------------------------------------------------
# Create a Chrono physical system
# -----------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()

# -----------------------------------------------------------------
# Material shared by all spheres (friction = 0.2)
# -----------------------------------------------------------------
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

# -----------------------------------------------------------------
# Sphere 1 – original sphere (position and velocity as required)
# -----------------------------------------------------------------
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))                 # <-- velocity set
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody)

# -----------------------------------------------------------------
# Sphere 2 – new sphere (position and velocity as required)
# -----------------------------------------------------------------
sphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
sphere2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))                 # <-- velocity set
sphere2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere2)

# -----------------------------------------------------------------
# Sphere 3 – new sphere (position and velocity as required)
# -----------------------------------------------------------------
sphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))
sphere3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))                  # <-- velocity set
sphere3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere3)

# -----------------------------------------------------------------
# Particle emitter (produces many small spheres)
# -----------------------------------------------------------------
emitter = particle.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Randomiser for particle position (on a box)
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
emitter.SetParticlePositioner(emitter_positions)

# Randomiser for particle alignment
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

# Linear velocity distribution
mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

# Angular velocity distribution
mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

# Creator for spherical particles
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_spheres)

# Register the callback that customises each emitted particle
mcreation_callback = MyCreatorForAll(chronoirr.ChVisualSystemIrrlicht(), coll)  # temporary visual system for callback
# (The real visual system is created later; the callback just needs the interfaces.)
emitter.RegisterAddBodyCallback(mcreation_callback)

# -----------------------------------------------------------------
# Visualisation (Irrlicht)
# -----------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three‑body particle emitter demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

# -----------------------------------------------------------------
# Solver settings
# -----------------------------------------------------------------
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# -----------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------
stepsize = 1e-2
G_constant = 6.674e-3  # (scaled) gravitational constant

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Emit particles this step
    emitter.EmitParticles(sys, stepsize)

    # Clear force accumulators for all bodies
    for body in sys.GetBodies():
        body.EmptyAccumulators()

    # -------------------------------------------------
    # Energy calculations (kinetic + potential)
    # -------------------------------------------------
    kinetic_energy = 0.0
    for body in sys.GetBodies():
        kinetic_energy += 0.5 * body.GetMass() * body.GetPosDt().Length2()

    potential_energy = 0.0
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D = abodyB.GetPos() - abodyA.GetPos()
        r = D.Length()
        potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r

    total_energy = kinetic_energy + potential_energy
    print(f"Kinetic Energy: {kinetic_energy:.6f}  "
          f"Potential Energy: {potential_energy:.6f}  "
          f"Total Energy: {total_energy:.6f}")

    # -------------------------------------------------
    # Apply mutual gravitational attraction
    # -------------------------------------------------
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D = abodyB.GetPos() - abodyA.GetPos()
        r = D.Length()
        f = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r ** 2)
        F = (D / r) * f
        abodyA.AccumulateForce(F, abodyA.GetPos(), False)
        abodyB.AccumulateForce(-F, abodyB.GetPos(), False)

    # Advance the system
    sys.DoStepDynamics(stepsize)