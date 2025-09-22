import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
sys.AddBody(ground)  
ground.SetFixed(True)  
ground.EnableCollision(False)  


ground_cyl = chrono.ChVisualShapeCylinder(0.1, 0.2)  
ground_cyl.SetColor(chrono.ChColor(0.5, 0.5, 0.5))  
ground.AddVisualShape(ground_cyl, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(1)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.1, 1, 1))  


pend1_cyl = chrono.ChVisualShapeCylinder(0.05, 2)  
pend1_cyl.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(pend1_cyl, chrono.ChFramed(chrono.ChVector3d(0, -1, 0)))  



pend_1.SetPos(chrono.ChVector3d(1, -1, 0))  
pend_1.SetRot(chrono.Q_from_AngZ(math.pi/6))  


pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)  
pend_2.EnableCollision(False)  
pend_2.SetMass(1)  
pend_2.SetInertiaXX(chrono.ChVector3d(0.1, 1, 1))  


pend2_cyl = chrono.ChVisualShapeCylinder(0.05, 2)  
pend2_cyl.SetColor(chrono.ChColor(0, 0.6, 0))  
pend_2.AddVisualShape(pend2_cyl, chrono.ChFramed(chrono.ChVector3d(0, -1, 0)))  



pend_2.SetPos(chrono.ChVector3d(2, -3, 0))  
pend_2.SetRot(chrono.Q_from_AngZ(-math.pi/4))  



rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)



rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(0, -2, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Double Pendulum Demo')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, -2, 6))  
vis.AddTypicalLights()  


log_info = True  
while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(1e-3)  

    
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()  
        pos_2 = pend_2.GetPos()  
        print("t = ", sys.GetChTime())
        print("Pendulum 1: ", pos_1.x, "  ", pos_1.y)
        print("Pendulum 2: ", pos_2.x, "  ", pos_2.y)
        lin_vel_1 = pend_1.GetPosDt()  
        lin_vel_2 = pend_2.GetPosDt()  
        print("Velocity 1: ", lin_vel_1.x, "  ", lin_vel_1.y)
        print("Velocity 2: ", lin_vel_2.x, "  ", lin_vel_2.y)
        log_info = False