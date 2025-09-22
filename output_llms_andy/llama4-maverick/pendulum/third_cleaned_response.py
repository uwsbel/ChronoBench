import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetBodyFixed(True)  
ground.EnableCollision(False)  


cyl_1 = chrono.ChCylinderShape(0.2, 0.4)  
ground.AddAsset(cyl_1, chrono.ChFrame(chrono.ChVector3d(0, 0, +1)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetBodyFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(1)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


cyl_2 = chrono.ChCylinderShape(0.2, 2)  
cyl_2.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddAsset(cyl_2, chrono.ChFrame(chrono.VNULL, chrono.Q_from_AngY(chrono.CH_PI_2)))



pend_1.SetPos(chrono.ChVector3d(1, 0, 0))



rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChCoordsys(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngZ(chrono.CH_PI_2)))
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetBodyFixed(False)  
pend_2.EnableCollision(False)  
pend_2.SetMass(1)  
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


cyl_3 = chrono.ChCylinderShape(0.2, 2)  
cyl_3.SetColor(chrono.ChColor(0, 0, 0.6))  
pend_2.AddAsset(cyl_3, chrono.ChFrame(chrono.VNULL, chrono.Q_from_AngY(chrono.CH_PI_2)))



pend_2.SetPos(chrono.ChVector3d(3, 0, 0))


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChCoordsys(chrono.ChVector3d(2, 0, 0), chrono.Q_from_AngZ(chrono.CH_PI_2)))
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Double Pendulum Demo')  
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
        print("Pendulum 1 Position: ", pos_1.x, "  ", pos_1.y)
        print("Pendulum 2 Position: ", pos_2.x, "  ", pos_2.y)
        lin_vel_1 = pend_1.GetPos_dt()  
        lin_vel_2 = pend_2.GetPos_dt()  
        print("Pendulum 1 Velocity: ", lin_vel_1.x, "  ", lin_vel_1.y)
        print("Pendulum 2 Velocity: ", lin_vel_2.x, "  ", lin_vel_2.y)
        log_info = False