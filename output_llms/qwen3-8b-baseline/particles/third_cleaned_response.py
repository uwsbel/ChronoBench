import pychrono as chrono
import pychrono.irrlicht as chronoirr

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
msphereBody.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))  
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody)


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


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Three-Body Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()


stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for body in sys.GetBodies():
        body.EmptyAccumulators()

    G_constant = 6.674e-3  

    
    kinetic_energy = 0
    for body in sys.GetBodies():
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    
    potential_energy = 0
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    
    total_energy = kinetic_energy + potential_energy
    mlist = list(combinations(sys.GetBodies(), 2))
    for abodyA, abodyB in mlist:
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)
        F_attract = (D_attract / r_attract) * f_attract
        abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)
        abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)

    
    print(
        f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {total_energy:.6f}")

    sys.DoStepDynamics(stepsize)