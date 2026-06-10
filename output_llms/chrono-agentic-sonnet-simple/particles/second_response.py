import os
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
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for particle collision
coll = sys.GetCollisionSystem()

# Create a sphere body (central attractor)
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody)

# Create a particle emitter
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)                               # emit rate
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)                           # max 200 particles

# Particle position randomizer — scatter within a 50x50x50 box
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
emitter.SetParticlePositioner(emitter_positions)

emitter_rotations = chrono.ChRandomParticleAlignmentUniform()     # random orientation
emitter.SetParticleAligner(emitter_rotations)

mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))  # speed 0–0.5
emitter.SetParticleVelocity(mvelo)

mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))  # ang vel 0–0.2
emitter.SetParticleAngularVelocity(mangvelo)

# Sphere shape creator — Zhang diameter distribution, constant density
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))  # Zhang(mean, spread)
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))     # 1600 kg/m³
emitter.SetParticleCreator(mcreator_spheres)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle emitter demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))                      # side-front view
vis.AddTypicalLights()

# Attach creation callback so newly-emitted bodies bind to vis + collision
mcreation_callback = MyCreatorForAll(vis, coll)
emitter.RegisterAddBodyCallback(mcreation_callback)

# Solver settings
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))     # no external gravity

# Simulation parameters
stepsize = 1e-2
sim_end = 10.0                                                     # run for 10 s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * stepsize)))        # untagged cadence constant


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    emitter.EmitParticles(sys, stepsize)                           # spawn new particles

    for body in sys.GetBodies():
        body.EmptyAccumulators()                                   # clear forces each step

    G_constant = 6.674e-3                                          # modified gravitational constant

    # Calculate the total kinetic energy of the system
    kinetic_energy = 0
    for body in sys.GetBodies():
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    # Calculate the total potential energy of the system (gravitational between pairs)
    potential_energy = 0
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    # Total energy of the system
    total_energy = kinetic_energy + potential_energy

    # Apply gravitational attraction forces between all pairs
    mlist = list(combinations(sys.GetBodies(), 2))
    for abodyA, abodyB in mlist:
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)
        F_attract = (D_attract / r_attract) * f_attract
        abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)
        abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)

    # Print energy values each step
    print(
        f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {total_energy:.6f}")


    sys.DoStepDynamics(stepsize)

    if sys.GetChTime() >= sim_end:
        break
