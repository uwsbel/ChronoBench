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



sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()


sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody)


emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)


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


mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_spheres)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle emitter demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()


mcreation_callback = MyCreatorForAll(vis, coll)
emitter.RegisterAddBodyCallback(mcreation_callback)


sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


stepsize = 1e-2
G_constant = 6.674e-3  

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    emitter.EmitParticles(sys, stepsize)

    
    for body in sys.GetBodies():
        body.EmptyAccumulators()

    
    total_kinetic = 0.0
    total_potential = 0.0

    
    for body in sys.GetBodies():
        mass = body.GetMass()
        vel = body.GetVelocity()
        total_kinetic += 0.5 * mass * vel.Length2()

    
    for bodyA, bodyB in combinations(sys.GetBodies(), 2):
        r_attract = (bodyA.GetPos() - bodyB.GetPos()).Length()
        if r_attract < 1e-10:  
            continue
        potential_energy = -G_constant * (bodyA.GetMass() * bodyB.GetMass()) / (r_attract ** 2)
        total_potential += potential_energy

    total_energy = total_kinetic + total_potential

    
    print(f"Step: {stepsize:.3f}")
    print(f"Kinetic Energy: {total_kinetic:.3f}")
    print(f"Potential Energy: {total_potential:.3f}")
    print(f"Total Energy: {total_energy:.3f}\n")

    sys.DoStepDynamics(stepsize)