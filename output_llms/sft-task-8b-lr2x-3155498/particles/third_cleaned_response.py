import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)


msphere1 = chrono.ChBodyEasySphere(1.0, 1800, True, True, sphere_mat)
msphere1.SetPos(chrono.ChVector3d(1, 1, 0))
msphere1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))
msphere1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(msphere1)


msphere2 = chrono.ChBodyEasySphere(1.0, 1800, True, True, sphere_mat)
msphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
msphere2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))
msphere2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(msphere2)


msphere3 = chrono.ChBodyEasySphere(1.0, 1800, True, True, sphere_mat)
msphere3.SetPos(chrono.ChVector3d(0, 20, 0))
msphere3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))
msphere3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(msphere3)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-body simulation demo')
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

    
    kinetic_energy = 0
    for body in sys.GetBodies():
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    
    potential_energy = 0
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        potential_energy += -6.674e-3 * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    
    total_energy = kinetic_energy + potential_energy

    
    print(
        f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {total_energy:.6f}")

    sys.DoStepDynamics(stepsize)