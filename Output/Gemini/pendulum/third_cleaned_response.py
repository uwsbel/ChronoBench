import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  

ground.SetCollide(True) 
ground.SetBodyFixed(True)


cyl_g = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(cyl_g, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1))) 




pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetBodyFixed(False)  

pend_1.SetCollide(True) 
pend_1.SetMass(1)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


cyl_1 = chrono.ChVisualShapeCylinder(0.1, 2)  
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngY(chrono.CH_PI/2)))




pend_1.SetPos(chrono.ChVector3d(1, 0, 1))



rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
sys.Add(pend_2)
pend_2.SetBodyFixed(False)
pend_2.SetCollide(True)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))


cyl_2 = chrono.ChVisualShapeCylinder(0.1, 2)
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngY(chrono.CH_PI/2)))


pend_2.SetPos(chrono.ChVector3d(2, 0, 1))  


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChCoordsysD(chrono.ChVector3d(2, 0, 1), chrono.QUNIT))
sys.AddLink(rev_2)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Double Pendulum')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  
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
        log_info = False