import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, chrono.ChVectorD(0, -5, 0))
sys.Add(ground)


pend_1 = chrono.ChBodyEasyCylinder(sys, 0.2, 2, 1000, True)
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))
sys.Add(pend_1)


rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngY(chrono.CH_PI_2)))
sys.AddLink(rev_1)


pend_2 = chrono.ChBodyEasyCylinder(sys, 0.2, 2, 1000, True)
pend_2.SetPos(chrono.ChVectorD(3, 0, 1))
sys.Add(pend_2)


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFrameD(chrono.ChVectorD(2, 0, 1), chrono.Q_from_AngY(chrono.CH_PI_2)))
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
        print("t = ", sys.GetChTime())
        print("     Pendulum 1: ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPos_dt()  
        print("     Pendulum 1 velocity: ", lin_vel_1.x, "  ", lin_vel_1.y)

        pos_2 = pend_2.GetPos()  
        print("     Pendulum 2: ", pos_2.x, "  ", pos_2.y)
        lin_vel_2 = pend_2.GetPos_dt()  
        print("     Pendulum 2 velocity: ", lin_vel_2.x, "  ", lin_vel_2.y)

        log_info = False