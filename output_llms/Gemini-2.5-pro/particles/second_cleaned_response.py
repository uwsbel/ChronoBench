import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis_system):
        super().__init__()
        self.vis_system = vis_system

    def OnAddBody(self, body, coords, creator):
        
        
        
        if body.GetVisualModel() and body.GetVisualModel().GetNumShapes() > 0:
            visual_shape = body.GetVisualShape(0)
            if visual_shape: 
                visual_shape.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        
        
        self.vis_system.BindItem(body)
        
        
        
        

        
        body.SetUseGyroTorque(False)


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2) 

msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0)) 

if msphereBody.GetVisualModel() and msphereBody.GetVisualModel().GetNumShapes() > 0:
    visual_shape = msphereBody.GetVisualShape(0)
    if visual_shape:
        visual_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
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
vis.SetWindowTitle('PyChrono Particle Emitter: N-Body Gravity & Energy') 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20)) 
vis.AddTypicalLights()



mcreation_callback = MyCreatorForAll(vis) 
emitter.RegisterAddBodyCallback(mcreation_callback)


sys.SetSolverType(chrono.ChSolver.Type_PSOR) 
sys.GetSolver().AsIterative().SetMaxIterations(40) 
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0)) 


stepsize = 1e-2 




gravity_eps = 1e-3 


print_interval = 0.1  
last_print_time = -print_interval 

while vis.Run():
    current_sim_time = sys.GetChTime()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    emitter.EmitParticles(sys, stepsize)

    
    
    
    
    
    G_constant = 6.674e-3

    
    kinetic_energy = 0.0
    potential_energy = 0.0
    
    
    all_bodies_list = sys.GetBodies() 

    
    for body in all_bodies_list:
        if body.IsActive() and body.GetMass() > 0:
            lin_vel = body.GetLinVel()
            kinetic_energy += 0.5 * body.GetMass() * lin_vel.Length2()
            
            

    
    for body in all_bodies_list:
        body.EmptyAccumulators()

    
    body_pairs = combinations(all_bodies_list, 2)

    for body_A, body_B in body_pairs:
        
        if not body_A or not body_B or body_A.GetMass() <= 0 or body_B.GetMass() <= 0:
            continue

        D_vec = body_B.GetPos() - body_A.GetPos() 
        r_sq = D_vec.Length2() 
        
        
print("error happened with only start ```python")