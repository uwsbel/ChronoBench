import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVector3d(0, -1.62, 0))  


ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetBodyFixed(True)  
ground.EnableCollision(False)  


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(cyl_ground, chrono.ChFrame(chrono.ChVector3d(0, 0, +1)))


cyl_joint = chrono.ChVisualShapeSphere(0.2)  
ground.AddVisualShape(cyl_joint)


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetBodyFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  
pend_1.SetPosDt(chrono.ChVector3d(0, 0, 0))  
pend_1.SetWvel(chrono.ChVector3d(0, 0, 1))  


cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1, chrono.ChFrame(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))



pend_1.SetPos(chrono.ChVector3d(0.75, 0, 0))


sph_joint = chrono.ChLinkLockSpherical()
sph_joint.Initialize(ground, pend_1, chrono.ChCoordsys(chrono.ChVector3d(0, 0, 0)))
sys.AddLink(sph_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('ChBodyAuxRef demo')  
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
        print(" ", pos_1.x, " ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()  
        print(" ", lin_vel_1.x, " ", lin_vel_1.y)
        log_info = False