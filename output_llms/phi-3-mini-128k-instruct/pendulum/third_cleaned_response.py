import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetFixed(True)  
ground.EnableCollision(False)  


cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(True)  
pend_1.SetMass(1)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)  
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))


pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)  
pend_2.EnableCollision(True)  
pend_2.SetMass(1)  
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)  
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))



rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0))
sys.AddLink(rev_1)


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0))
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Double Pendulum Simulation')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  
vis.AddTypicalLights()  


log_info = True  
while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    sys.DoStepDynamics(1e-3)  

    
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()  
        pos_2 = pend_2.GetPos()  
        lin_vel_1 = pend_1.GetPosDt()  
        lin_vel_2 = pend_2.GetPosDt()  
        print("t = ", sys.GetChTime())
        print("Pendulum 1: ", pos_1.x, "  ", pos_1.y)
        print("Pendulum 2: ", pos_2.x, "  ", pos_2.y)
        print("Linear velocity of Pendulum 1: ", lin_vel_1.x, "  ", lin_vel_1.y)
        print("Linear velocity of Pendulum 2: ", lin_vel_2.x, "  ", lin_vel_2.y)
        log_info = False