import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)


sphere1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere1.SetPos(chrono.ChVector3d(1, 1, 0))
sphere1.SetPos_dt(chrono.ChVector3d(0.5, 0, 0.1))  
sphere1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere1)


sphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
sphere2.SetPos_dt(chrono.ChVector3d(-0.5, 0, -0.1))  
sphere2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere2)


sphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))
sphere3.SetPos_dt(chrono.ChVector3d(0, -0.5, 0.2))  
sphere3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere3)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-Body Gravitational Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()


sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    for body in sys.GetBodies():
        body.EmptyAccumulators()

    
    G_constant = 6.674e-3

    
    kinetic_energy = 0
    potential_energy = 0
    
    
    body_pairs = list(combinations(sys.GetBodies(), 2))
    
    
    for bodyA, bodyB in body_pairs:
        distance = bodyB.GetPos() - bodyA.GetPos()
        r = distance.Length()
        potential_energy += -G_constant * (bodyA.GetMass() * bodyB.GetMass()) / r

    
    for bodyA, bodyB in body_pairs:
        distance = bodyB.GetPos() - bodyA.GetPos()
        r = distance.Length()
        
        if r > 1e-6:  
            force_magnitude = G_constant * (bodyA.GetMass() * bodyB.GetMass()) / (r ** 2)
            force_vector = (distance / r) * force_magnitude
            
            
            bodyA.AccumulateForce(force_vector, bodyA.GetPos(), False)
            bodyB.AccumulateForce(-force_vector, bodyB.GetPos(), False)

    
    for body in sys.GetBodies():
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * body.GetMass() * velocity.Length2()

    
    print(f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {kinetic_energy + potential_energy:.6f}")

    
    sys.DoStepDynamics(stepsize)