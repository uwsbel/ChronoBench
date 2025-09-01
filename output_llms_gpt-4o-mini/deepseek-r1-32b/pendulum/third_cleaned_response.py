import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(1)  


r = 0.2
l = 2
m = 1
Ixx = 0.5 * m * r**2 + (1/12) * m * l**2
Iyy = Ixx
Izz = 0.5 * m * r**2
pend_1.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Izz))


cyl_1 = chrono.ChVisualShapeCylinder(r, l)  
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_1.SetPos(chrono.ChVector3d(1, 0, 1))  


rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)  
pend_2.EnableCollision(False)  
pend_2.SetMass(1)  


pend_2.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Izz))


cyl_2 = chrono.ChVisualShapeCylinder(r, l)  
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))  
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_2.SetPos(chrono.ChVector3d(2, 0, 1))  


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)),
                 chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
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
        print("Pendulum 1 Position: ", pos_1.x, pos_1.y, pos_1.z)
        print("Pendulum 2 Position: ", pos_2.x, pos_2.y, pos_2.z)
        lin_vel_1 = pend_1.GetPosDt()  
        lin_vel_2 = pend_2
print("error happened with only start ```python")