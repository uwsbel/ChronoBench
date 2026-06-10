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
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))




pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)       
pend_1.EnableCollision(False)
pend_1.SetMass(1)            
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


cyl_1 = chrono.ChVisualShapeCylinder(0.1, 2)
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_1.SetPos(chrono.ChVector3d(1, 0, 1))


rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1,
                 chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                 chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)




pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)       
pend_2.EnableCollision(False)
pend_2.SetMass(1)            
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


cyl_2 = chrono.ChVisualShapeCylinder(0.1, 2)
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6))  
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))



pend_2.SetPos(chrono.ChVector3d(3, 0, 1))


cyl_joint = chrono.ChVisualShapeCylinder(0.15, 0.4)
cyl_joint.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(cyl_joint, chrono.ChFramed(chrono.ChVector3d(2, 0, 1)))


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2,
                 chrono.ChFramed(chrono.ChVector3d(2, 0, 1),
                                 chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_2)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 8))
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

        print(f"t = {sys.GetChTime():.4f} s")
        print(f"  Pendulum 1 pos : x={pos_1.x:.4f},  y={pos_1.y:.4f}")
        lin_vel_1 = pend_1.GetPosDt()
        print(f"  Pendulum 1 vel : vx={lin_vel_1.x:.4f}, vy={lin_vel_1.y:.4f}")

        print(f"  Pendulum 2 pos : x={pos_2.x:.4f},  y={pos_2.y:.4f}")
        lin_vel_2 = pend_2.GetPosDt()
        print(f"  Pendulum 2 vel : vx={lin_vel_2.x:.4f}, vy={lin_vel_2.y:.4f}")

        log_info = False