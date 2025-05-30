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



sphere1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere1.SetPos(chrono.ChVector3d(1, 1, 0))
sphere1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))
sphere1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere1)


sphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
sphere2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))
sphere2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere2)


sphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))
sphere3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))
sphere3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere3)


emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)


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


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-Body Simulation with Particles')
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

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    emitter.EmitParticles(sys, stepsize)

    
    for body in sys.GetBodies():
        body.EmptyAccumulators()

    G_constant = 6.674e-3  

    
    body_list = sys.GetBodies()
    for abodyA, abodyB in combinations(body_list, 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        if r_attract > 1e-6:  
            f_magnitude = G_constant * abodyA.GetMass() * abodyB.GetMass() / (r_attract ** 2)
            F_dir = D_attract.GetNormalized()
            F_attract = F_dir * f_magnitude
            abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)
            abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)

    sys.DoStepDynamics(stepsize)