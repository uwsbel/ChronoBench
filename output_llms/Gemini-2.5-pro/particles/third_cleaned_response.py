import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self): 
        super().__init__()

    def OnAddBody(self, body, coords, creator):
        
        if body.GetNumVisualShapes() > 0: 
            vis_shape = body.GetVisualShape(0)
            if vis_shape: 
                vis_shape.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        
        
        body.SetUseGyroTorque(False)


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)






sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)
sphere_radius = 2.1
sphere_density = 1800
sphere_texture_file = chrono.GetChronoDataFile("textures/concrete.jpg")


msphereBody = chrono.ChBodyEasySphere(sphere_radius, sphere_density, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1)) 
if msphereBody.GetNumVisualShapes() > 0 and msphereBody.GetVisualShape(0):
    msphereBody.GetVisualShape(0).SetTexture(sphere_texture_file)
sys.Add(msphereBody)


sphere2_body = chrono.ChBodyEasySphere(sphere_radius, sphere_density, True, True, sphere_mat)
sphere2_body.SetPos(chrono.ChVector3d(-10, -10, 0)) 
sphere2_body.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1)) 
if sphere2_body.GetNumVisualShapes() > 0 and sphere2_body.GetVisualShape(0):
    sphere2_body.GetVisualShape(0).SetTexture(sphere_texture_file)
sys.Add(sphere2_body)


sphere3_body = chrono.ChBodyEasySphere(sphere_radius, sphere_density, True, True, sphere_mat)
sphere3_body.SetPos(chrono.ChVector3d(0, 20, 0)) 
sphere3_body.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2)) 
if sphere3_body.GetNumVisualShapes() > 0 and sphere3_body.GetVisualShape(0):
    sphere3_body.GetVisualShape(0).SetTexture(sphere_texture_file)
sys.Add(sphere3_body)



emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)



emitter_positions = chrono.ChRandomParticlePositionOnGeometry()

emitter_positions.SetGeometry(chrono.ChBox(chrono.ChVector3d(50, 50, 50)), chrono.ChFrameD())
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


mcreation_callback = MyCreatorForAll() 
emitter.RegisterAddBodyCallback(mcreation_callback)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('3-Body and Particle Emitter Gravitational Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -40)) 
vis.AddTypicalLights()



sys.SetSolverType(chrono.ChSolver.Type_PSOR)


if isinstance(sys.GetSolver(), chrono.ChIterativeSolver):
    sys.GetSolver().AsIterative().SetMaxIterations(40)


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


stepsize = 1e-2
G_constant = 6.674e-3  

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    emitter.EmitParticles(sys, stepsize)

    
    
    bodylist = sys.GetBodies() 
    for body in bodylist:
        body.EmptyAccumulators()

    
    potential_energy = 0
    
    for body_A, body_B in combinations(bodylist, 2):
        
        if not body_A or not body_B or body_A.GetMass() <= 0 or body_B.GetMass() <= 0:
            continue

        delta_pos = body_B.GetPos() - body_A.GetPos()
        distance = delta_pos.Length()

        if distance < 1e-3:  
            
            
            
            
            continue
        
        
        force_magnitude = G_constant * (body_A.GetMass() * body_B.GetMass()) / (distance * distance)
        
        
        force_vector = (delta_pos / distance) * force_magnitude
        
        
        
        body_A.AccumulateForce(force_vector, body_A.GetPos(), False)  
        body_B.AccumulateForce(-force_vector, body_B.GetPos(), False) 

        
        potential_energy += -G_constant * (body_A.GetMass() * body_B.GetMass()) / distance

    
    kinetic_energy = 0
    for body in bodylist:
        if body and body.GetMass() > 0: 
            mass = body.GetMass()
            velocity_sq = body.GetPosDt().Length2()
            kinetic_energy += 0.5 * mass * velocity_sq
            
    total_energy = kinetic_energy + potential_energy

    
    
    

    
    sys.DoStepDynamics(stepsize)