import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        
        body.GetVisualShape(0).SetTexture(
            chrono.GetChronoDataFile("textures/bluewhite.png")
        )
        self.vis.BindItem(body)
        self.coll.BindItem(body)
        body.SetUseGyroTorque(False)



sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()


sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)
msphereBody = chrono.ChBodyEasySphere(
    2.1,     
    1800,    
    True,    
    True,    
    sphere_mat
)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(msphereBody)


emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)


emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFrameMoving())
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

mcreator_spheres.SetDiameterDistribution(
    chrono.ChZhangDistribution(0.6, 0.23)
)

mcreator_spheres.SetDensityDistribution(
    chrono.ChConstantDistribution(1600)
)
emitter.SetParticleCreator(mcreator_spheres)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Particle emitter demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()


creation_callback = MyCreatorForAll(vis, coll)
emitter.RegisterAddBodyCallback(creation_callback)


sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    emitter.EmitParticles(sys, stepsize)

    
    for body in sys.Get_bodylist():
        body.EmptyAccumulators()

    
    pairs = combinations(sys.Get_bodylist(), 2)
    for A, B in pairs:
        D = B.GetPos() - A.GetPos()
        r = D.Length()
        if r > 1e-9:
            
            
            G_constant = 6.674e-3
            Fmag = G_constant * A.GetMass() * B.GetMass() / (r * r)
            Fvec = (D / r) * Fmag
            A.AccumulateForce(Fvec, A.GetPos(), False)
            B.AccumulateForce(-Fvec, B.GetPos(), False)

    
    
    G_constant = 6.674e-3

    
    KE = 0.0
    for body in sys.Get_bodylist():
        v = body.GetPos_dt()
        KE += 0.5 * body.GetMass() * v.Length2()

    
    PE = 0.0
    for A, B in combinations(sys.Get_bodylist(), 2):
        r = (B.GetPos() - A.GetPos()).Length()
        if r > 1e-9:
            PE += -G_constant * A.GetMass() * B.GetMass() / r

    totalE = KE + PE
    print(f"Step={sys.GetChTime():.3f}  KE={KE:.5e}  PE={PE:.5e}  E_tot={totalE:.5e}")

    
    sys.DoStepDynamics(stepsize)