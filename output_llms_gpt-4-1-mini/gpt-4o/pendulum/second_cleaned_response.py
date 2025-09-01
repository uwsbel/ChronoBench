import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVector3d(0, -1.62, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetBodyFixed(True)  
ground.SetCollide(False)  


cyl_1 = chrono.ChCylinderShape()
cyl_1.GetCylinderGeometry().p1 = chrono.ChVector3d(0, 0, 0.2)
cyl_1.GetCylinderGeometry().p2 = chrono.ChVector3d(0, 0, -0.2)
cyl_1.GetCylinderGeometry().rad = 0.2
ground.AddVisualShape(cyl_1)


pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetBodyFixed(False)  
pend_1.SetCollide(False)  
pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  


cyl_2 = chrono.ChCylinderShape()
cyl_2.GetCylinderGeometry().p1 = chrono.ChVector3d(0, -0.75, 0)
cyl_2.GetCylinderGeometry().p2 = chrono.ChVector3d(0, 0.75, 0)
cyl_2.GetCylinderGeometry().rad = 0.1
cyl_2.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_2)



pend_1.SetPos(chrono.ChVector3d(1, 0, 1))


pend_1.SetWvel_par(chrono.ChVector3d(0, 0, 1))


spherical_joint = chrono.ChLinkLockSpherical()
spherical_joint.Initialize(ground, pend_1, chrono.ChCoordsys(chrono.ChVector3d(0, 0, 1)))
sys.AddLink(spherical_joint)


sphere = chrono.ChSphereShape()
sphere.GetSphereGeometry().rad = 0.2
ground.AddVisualShape(sphere, chrono.ChFrame(chrono.ChVector3d(0, 0, 1)))


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
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPos_dt()  
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        log_info = False