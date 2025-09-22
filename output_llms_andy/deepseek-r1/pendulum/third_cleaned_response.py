import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


ground_cyl = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(ground_cyl, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(1)  


r = 0.2
L = 2
m = 1
Ixx = 0.5 * m * r**2
Iyy = (1/12) * m * (3*r**2 + L**2)
pend_1.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Iyy))


pend1_cyl = chrono.ChVisualShapeCylinder(0.2, 2)
pend1_cyl.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(pend1_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


joint_pos = chrono.ChVector3d(0, 0, 1)  
pend_1.SetPos(joint_pos + chrono.ChVector3d(1, 0, 0))  

rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(joint_pos, chrono.QUNIT))
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Iyy))  


pend2_cyl = chrono.ChVisualShapeCylinder(0.2, 2)
pend2_cyl.SetColor(chrono.ChColor(0, 0.6, 0))  
pend_2.AddVisualShape(pend2_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_2.SetPos(joint_pos + chrono.ChVector3d(2, 0, 0))  


joint2_pos = joint_pos + chrono.ChVector3d(2, 0, 0)  
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(joint2_pos, chrono.QUNIT))
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 8))  
vis.AddTypicalLights()


log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)  

    
    if log_info and sys.GetChTime() > 1:
        print(f"t = {sys.GetChTime():.3f} s")
        print(f"Pendulum 1 position: {pend_1.GetPos().x:.3f}, {pend_1.GetPos().y:.3f}")
        print(f"Pendulum 2 position: {pend_2.GetPos().x:.3f}, {pend_2.GetPos().y:.3f}")
        log_info = False