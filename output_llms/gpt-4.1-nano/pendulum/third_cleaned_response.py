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
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))


pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(1)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


cyl_1_vis = chrono.ChVisualShapeCylinder(0.2, 2)  
cyl_1_vis.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1_vis, chrono.ChFramed(chrono.ChVector3d(0, -1, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_1.SetPos(chrono.ChVector3d(1, 0, 1))


pend_2 = chrono.ChBody()
sys.Add(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))


cyl_2_vis = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2_vis.SetColor(chrono.ChColor(0, 0, 0.6))  
pend_2.AddVisualShape(cyl_2_vis, chrono.ChFramed(chrono.ChVector3d(0, -1, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))



pend_2.SetPos(chrono.ChVector3d(1, -2, 1))  


rev_ground_to_pend1 = chrono.ChLinkLockRevolute()

rev_ground_to_pend1.Initialize(
    ground,
    pend_1,
    chrono.ChFrameD(chrono.ChVector3d(1, 0, 1), chrono.Quat(1, 0, 0, 0))
)
sys.AddLink(rev_ground_to_pend1)


rev_pend1_to_pend2 = chrono.ChLinkLockRevolute()

rev_pend1_to_pend2.Initialize(
    pend_1,
    pend_2,
    chrono.ChFrameD(chrono.ChVector3d(1, -2, 1), chrono.Quat(1, 0, 0, 0))
)
sys.AddLink(rev_pend1_to_pend2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Simulation')
vis.Initialize()


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
        print("Pendulum 1 position: ", pos_1.x, " ", pos_1.y, " ", pos_1.z)
        lin_vel_1 = pend_1.GetPosDt()
        print("Pendulum 1 velocity: ", lin_vel_1.x, " ", lin_vel_1.y, " ", lin_vel_1.z)
        pos_2 = pend_2.GetPos()
        print("Pendulum 2 position: ", pos_2.x, " ", pos_2.y, " ", pos_2.z)
        lin_vel_2 = pend_2.GetPosDt()
        print("Pendulum 2 velocity: ", lin_vel_2.x, " ", lin_vel_2.y, " ", lin_vel_2.z)
        log_info = False