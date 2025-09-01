import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
from numpy import linalg as LA
import math



def main():
    

    
    
    

    
    mechanical_system = cascade.CMechSystem()

    
    truss = cascade.CLinkTruss()
    truss.SetName("truss")
    truss.SetMass(100.0)
    truss.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    truss.SetInertiaXY(chrono.ChVector3d(1, 1, 1))
    truss.SetFrame(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    truss.SetCollisionType(cascade.CCollisionType_NONE)
    truss.SetFixed(True)
    mechanical_system.AddLink(truss)

    
    revolute_joint = cascade.CLinkRevoluteJoint()
    revolute_joint.SetName("revolute_joint")
    revolute_joint.SetLinkedBody(truss.GetBodyA(), cascade.CBodyEasyBox(1, 1, 1, 1000, False, True, True))
    revolute_joint.SetFrame(chrono.ChFrame(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    revolute_joint.SetCollisionType(cascade.CCollisionType_NONE)
    revolute_joint.SetMotorType(cascade.CMotorType_NONE)
    mechanical_system.AddLink(revolute_joint)

    
    crankshaft = cascade.CLinkShaft()
    crankshaft.SetName("crankshaft")
    crankshaft.SetLinkedBody(revolute_joint.GetBodyB(), cascade.CBodyEasyShaft(1, 1, 1000, True, True, True))
    crankshaft.SetCollisionType(cascade.CCollisionType_NONE)
    crankshaft.SetFrame(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    mechanical_system.AddLink(crankshaft)

    
    revolute_joint2 = cascade.CLinkRevoluteJoint()
    revolute_joint2.SetName("revolute_joint2")
    revolute_joint2.SetLinkedBody(crankshaft.GetBodyB(), cascade.CBodyEasyBox(1, 1, 1, 1000, False, True, True))
    revolute_joint2.SetFrame(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    revolute_joint2.SetCollisionType(cascade.CCollisionType_NONE)
    revolute_joint2.SetMotorType(cascade.CMotorType_NONE)
    mechanical_system.AddLink(revolute_joint2)

    
    connecting_rod = cascade.CLinkPrismatic()
    connecting_rod.SetName("connecting_rod")
    connecting_rod.SetLinkedBody(crankshaft.GetBodyB(), cascade.CBodyEasyBox(1, 1, 1, 1000, False, True, True))
    connecting_rod.SetFrame(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    connecting_rod.SetCollisionType(cascade.CCollisionType_NONE)
    connecting_rod.SetMotorType(cascade.CMotorType_NONE)
    mechanical_system.AddLink(connecting_rod)

    
    piston = cascade.CLinkShaft()
    piston.SetName("piston")
    piston.SetLinkedBody(connecting_rod.GetBodyB(), cascade.CBodyEasyShaft(1, 1, 1000, True, True, True))
    piston.SetCollisionType(cascade.CCollisionType_NONE)
    piston.SetFrame(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    mechanical_system.AddLink(piston)

    
    motor = cascade.CMotorRotationAngle(chrono.ChFunctionSetAngle(chrono.CH_PI / 2, chrono.CH_PI, 0, 0.2, 0.5), 0.1)
    revolute_joint.SetMotor(motor)

    
    
    

    
    sys_coll = cascade.CCascadeSystemCollision()
    sys_coll.AddItem(truss)
    sys_coll.AddItem(revolute_joint)
    sys_coll.AddItem(crankshaft)
    sys_coll.AddItem(revolute_joint2)
    sys_coll.AddItem(connecting_rod)
    sys_coll.AddItem(piston)
    sys_coll.SetResolution(0.002)

    
    vis = cascade.CCascadeSystemVisual()
    vis.AddItem(truss.GetBodyA().GetMesh().GetVisualShape(0))
    vis.AddItem(revolute_joint.GetBodyB().GetMesh().GetVisualShape(0))
    vis.AddItem(crankshaft.GetBodyB().GetMesh().GetVisualShape(0))
    vis.AddItem(revolute_joint2.GetBodyB().GetMesh().GetVisualShape(0))
    vis.AddItem(connecting_rod.GetBodyB().GetMesh().GetVisualShape(0))
    vis.AddItem(piston.GetBodyB().GetMesh().GetVisualShape(0))
    vis.SetCamera(chrono.ChCamera(chrono.ChVector3d(2, 2, 2), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1)))
    vis.SetImageSize(1024, 768)
    vis.SetChronoRenderingType(chrono.ChVisualisationType_BULLET)

    
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Simple crank-slider mechanism')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddChCamera(trackBody)
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AddChHeader()

    
    irrvis = irr.ChVisualSystemIrrlicht()
    irrvis.AttachSystem(vis)
    irrvis.Initialize()
    irrvis.AddTypicalLights()
    irrvis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    irrvis.AddSkyBox()
    irrvis.AddCamera(chrono.ChVector3d(2, 2, 2), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1))

    
    
    

    
    solver = cascade.CCascadeSolver()
    solver.SetSolverType(cascade.CCascadeSolver.ACCOUTAB)

    
    solver.SetStepSizes(1e-5, 1e-3, 1e-3)

    
    solver.SetMaxIter(20)
    solver.SetTolerance(1e-4)
    solver.SetPrintStepSize(1e-2)

    
    driver = cascade.CCascadeDriverPython(sys, solver)
    driver.SetChTime(0)

    
    class MyCollisionCallback(cascade.CCascadeCollisionCallback):
        def __init__(self, driver):
            cascade.CCascadeCollisionCallback.__init__(self)
            self.driver = driver

        def OnAdded(self, item1, item2):
            print('Collision between items', item1.GetName(), 'and', item2.GetName())
            self.driver.SetChTime(0)

    
    class MyVisualCallback(cascade.CCascadeVisualCallback):
        def __init__(self, driver):
            cascade.CCascadeVisualCallback.__init__(self)
            self.driver = driver

        def OnRender(self):
            vis.Render()

    
    collision_callback = MyCollisionCallback(driver)
    visual_callback = MyVisualCallback(driver)

    
    time = 0.
    time_step = 0.1
    time_end = 1.0

    driver.SetChTime(0)
    while (time < time_end):
        time = driver.GetChTime()

        
        driver.Update()

        
        driver.Advance(step_size)

    return 0






veh = cascade.CCascadeSystem()
trackBody = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, True, True)
trackBody.SetPos(chrono.ChVector3d(0, 0, 0))
veh.AddGroundBody(trackBody)


step_size = 1e-3


main()