import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.SetCollide(False)  


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))


pend_1 = chrono.ChBody()
sys.Add(pend_1)  
pend_1.SetFixed(False)  
pend_1.SetCollide(False)  
pend_1.SetMass(1)  
pend_1.SetInertiaXX(chrono.ChVectorD(0.2, 1, 1))  


cyl_pend1 = chrono.ChVisualShapeCylinder(0.1, 1.0)  
cyl_pend1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_pend1, chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0), 
                                            chrono.Q_from_AngY(chrono.CH_C_PI_2)))



pend_1.SetPos(chrono.ChVectorD(0.5, 0, 1))



rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), 
                                               chrono.QUNIT))
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
sys.Add(pend_2)
pend_2.SetFixed(False)
pend_2.SetCollide(False)
pend_2.SetMass(0.8)  
pend_2.SetInertiaXX(chrono.ChVectorD(0.15, 0.8, 0.8))  


cyl_pend2 = chrono.ChVisualShapeCylinder(0.08, 0.8)  
cyl_pend2.SetColor(chrono.ChColor(0, 0.6, 0))  
pend_2.AddVisualShape(cyl_pend2, chrono.ChFrameD(chrono.ChVectorD(0.4, 0, 0), 
                                            chrono.Q_from_AngY(chrono.CH_C_PI_2)))


pend_2.SetPos(chrono.ChVectorD(1.4, 0, 1))


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChCoordsysD(chrono.ChVectorD(1.0, 0, 1), 
                                               chrono.QUNIT))
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Double Pendulum Demo')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVectorD(0, 3, 6))  
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
        lin_vel_1 = pend_1.GetPos_dt()  
        lin_vel_2 = pend_2.GetPos_dt()  
        print("Velocity 1: ", lin_vel_1.x, "  ", lin_vel_1.y)
        print("Velocity 2: ", lin_vel_2.x, "  ", lin_vel_2.y)
        log_info = False