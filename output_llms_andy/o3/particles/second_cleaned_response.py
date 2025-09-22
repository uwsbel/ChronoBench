import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations





class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        
        if body.GetVisualModel():
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

big_sphere = chrono.ChBodyEasySphere(2.1,        
                                     1800,       
                                     True, True, 
                                     sphere_mat)
big_sphere.SetPos(chrono.ChVector3d(1, 1, 0))
big_sphere.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(big_sphere)




emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)


pos_random = chrono.ChRandomParticlePositionOnGeometry()
pos_random.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
emitter.SetParticlePositioner(pos_random)


align_random = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(align_random)


vel_random = chrono.ChRandomParticleVelocityAnyDirection()
vel_random.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(vel_random)


avel_random = chrono.ChRandomParticleVelocityAnyDirection()
avel_random.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(avel_random)




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
vis.SetWindowTitle('Particle emitter demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()


emitter.RegisterAddBodyCallback(MyCreatorForAll(vis, coll))




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

    
    kinetic_energy   = 0.0
    potential_energy = 0.0

    bodies = list(sys.GetBodies())

    
    for b in bodies:
        v = b.GetPos_dt()
        kinetic_energy += 0.5 * b.GetMass() * v.Length2()

    
    for bodyA, bodyB in combinations(bodies, 2):
        d_vec = bodyB.GetPos() - bodyA.GetPos()
        r     = d_vec.Length()
        if r == 0:
            continue

        
        f_mag = G_constant * bodyA.GetMass() * bodyB.GetMass() / (r * r)
        F     = (d_vec / r) * f_mag

        bodyA.AccumulateForce( F, bodyA.GetPos(), False)
        bodyB.AccumulateForce(-F, bodyB.GetPos(), False)

        
        potential_energy += -G_constant * bodyA.GetMass() * bodyB.GetMass() / r

    total_energy = kinetic_energy + potential_energy
    print(f"KE = {kinetic_energy:10.4f} | PE = {potential_energy:10.4f} | "
          f"Total = {total_energy:10.4f}")

    
    sys.DoStepDynamics(stepsize)