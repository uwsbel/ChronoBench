import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)  
ground.SetCollide(False)  
sys.AddBody(ground)


cyl_ground = chrono.ChCylinderShape()
cyl_ground.GetCylinderGeometry().rad = 0.2
cyl_ground.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.2)  
cyl_ground.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0.2)   
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))


pend_1 = chrono.ChBody()
pend_1.SetBodyFixed(False)  
pend_1.SetCollide(False)  
pend_1.SetMass(1)  
pend_1.SetInertiaXX(chrono.ChVectorD(0.2, 1, 1))  


cyl_pend1 = chrono.ChCylinderShape()
cyl_pend1.GetCylinderGeometry().rad = 0.2
cyl_pend1.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
cyl_pend1.GetCylinderGeometry().p2 = chrono.ChVectorD(2, 0, 0)  
cyl_pend1.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
cyl_pend1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_pend1)





pend_1.SetPos(chrono.ChVectorD(1, 0, 1))
sys.AddBody(pend_1)



rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1,
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
pend_2.SetBodyFixed(False)  
pend_2.SetCollide(False)  
pend_2.SetMass(1)  
pend_2.SetInertiaXX(chrono.ChVectorD(0.2, 1, 1))  


cyl_pend2 = chrono.ChCylinderShape()
cyl_pend2.GetCylinderGeometry().rad = 0.2
cyl_pend2.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
cyl_pend2.GetCylinderGeometry().p2 = chrono.ChVectorD(2, 0, 0)  
cyl_pend2.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
cyl_pend2.SetColor(chrono.ChColor(0, 0, 0.6))  
pend_2.AddVisualShape(cyl_pend2)






pend_2.SetPos(pend_1.GetPos() + chrono.ChVectorD(2, 0, 0))
sys.AddBody(pend_2)



rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2,
                 chrono.ChFrameD(chrono.ChVectorD(2, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Double Pendulum Simulation')  
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
        lin_vel_1 = pend_1.GetPos_dt()  
        lin_vel_2 = pend_2.GetPos_dt()  
        print(f"t = {sys.GetChTime():.4f} s")
        print(f" Pendulum 1 pos = ({pos_1.x:.3f}, {pos_1.y:.3f}, {pos_1.z:.3f})")
        print(f" Pendulum 1 vel = ({lin_vel_1.x:.3f}, {lin_vel_1.y:.3f}, {lin_vel_1.z:.3f})")
        print(f" Pendulum 2 pos = ({pos_2.x:.3f}, {pos_2.y:.3f}, {pos_2.z:.3f})")
        print(f" Pendulum 2 vel = ({lin_vel_2.x:.3f}, {lin_vel_2.y:.3f}, {lin_vel_2.z:.3f})")
        log_info = False