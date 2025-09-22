import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVector3d(0, -1.62, 0))  


ground = chrono.ChBody()
sys.AddBody(ground)  
ground.SetBodyFixed(True)  
ground.GetCollisionModel().ClearModel()  
ground.SetCollide(False)  






cyl_1 = chrono.ChCylinderShape()
cyl_1.GetCylinderGeometry().p1 = chrono.ChVector<>(0, 0, 0)  
cyl_1.GetCylinderGeometry().p2 = chrono.ChVector<>(0, 0, 0.4)  
cyl_1.GetCylinderGeometry().rad = 0.2  
ground.AddVisualShape(cyl_1)





ground.AddVisualShape(chrono.ChCylinderShapeEasy(0.4, 0.2))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)  
pend_1.SetBodyFixed(False)  
pend_1.SetCollide(False)  


pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  


cyl_2 = chrono.ChCylinderShapeEasy(1.5, 0.1)
cyl_2.SetColor(chrono.ChColor(0.6, 0, 0))


pend_1.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD().RotationY(chrono.CH_C_PI_2)))




pend_1.SetPos(chrono.ChVectorD(0.75, 0, 1))  





pend_1.SetWvel_loc(chrono.ChVectorD(0, 0, 1))  






pivot_pos = chrono.ChVectorD(0, 0, 1)  

sph_joint = chrono.ChLinkLockSpherical()

sph_joint.Initialize(ground, pend_1, chrono.ChFrameD(pivot_pos))
sys.AddLink(sph_joint)



sphere_vis = chrono.ChSphereShape()
sphere_vis.SetSphereGeometry(chrono.ChVectorD(0, 0, 0), 2)  
sphere_vis.SetColor(chrono.ChColor(0, 0, 1))  

ground.AddVisualShape(sphere_vis, chrono.ChFrameD(pivot_pos))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Spherical Joint Single Pendulum on Moon Gravity')  
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
        print("t = {:.3f}".format(sys.GetChTime()))
        print("     Position: x = {:.3f}, y = {:.3f}, z = {:.3f}".format(pos_1.x, pos_1.y, pos_1.z))
        lin_vel_1 = pend_1.GetPos_dt() if hasattr(pend_1, 'GetPos_dt') else pend_1.GetPosDt()  
        
        
        
        if lin_vel_1 is None:
            lin_vel_1 = pend_1.GetPosDt()
        print("     Linear velocity: vx = {:.3f}, vy = {:.3f}, vz = {:.3f}".format(lin_vel_1.x, lin_vel_1.y, lin_vel_1.z))
        ang_vel_1 = pend_1.GetWvel_loc()
        print("     Angular velocity (local): wx = {:.3f}, wy = {:.3f}, wz = {:.3f}".format(ang_vel_1.x, ang_vel_1.y, ang_vel_1.z))
        log_info = False