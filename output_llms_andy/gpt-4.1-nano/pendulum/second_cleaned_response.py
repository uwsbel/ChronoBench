import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.ChVector3d(0, 0, +1)))


pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  


pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  


cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  

pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.ChVector3d(0, -0.75, 0), chrono.Quat(1, 0, 0, 0)))


pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

initial_angular_velocity = 2.0  
pend_1.SetW(chrono.ChVector3d(0, 0, initial_angular_velocity))


sphere_joint = chrono.ChLinkLockSpherical()
sphere_joint.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVector3d(1, 0, 1)))
sys.AddLink(sphere_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Single Pendulum on Moon')  
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
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()  
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        log_info = False