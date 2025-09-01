import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


sphere_shape = chrono.ChVisualShapeSphere(0.5)  
ground.AddVisualShape(sphere_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  


sphere_shape = chrono.ChVisualShapeSphere(2)  
sphere_shape.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(sphere_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))



pend_1.SetPos(chrono.ChVector3d(1, 0, 1))



spher_joint = chrono.ChLinkLockSpherical()
spher_joint.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))  
sys.AddLink(spher_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('ChBodyAuxRef demo')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  
vis.AddTypicalLights()  



initial_angle_velocity = chrono.ChFunctionSine(0.1, 1.0)
spher_joint.SetInitialRotation(initial_angle_velocity)


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