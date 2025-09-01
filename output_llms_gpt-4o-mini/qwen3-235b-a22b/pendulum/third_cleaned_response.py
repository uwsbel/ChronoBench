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
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(1)  


radius_p1 = 0.2
length_p1 = 2
mass_p1 = 1
Ixx_p1 = 0.5 * mass_p1 * radius_p1**2
Iyy_p1 = (mass_p1 / 12) * (3 * radius_p1**2 + length_p1**2)
pend_1.SetInertiaXX(chrono.ChVector3d(Ixx_p1, Iyy_p1, Iyy_p1))


cyl_p1 = chrono.ChVisualShapeCylinder(radius_p1, length_p1)  
cyl_p1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_p1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_1.SetPos(chrono.ChVector3d(1, 0, 1))  


rev_1 = chrono.ChLinkLockRevolute()

ground_joint_pos = chrono.ChVector3d(0, 0, 1)
pend1_joint_pos = chrono.ChVector3d(-1, 0, 0)  
rev_1.Initialize(
    ground, pend_1,
    chrono.ChFramed(ground_joint_pos, chrono.ChQuaterniond(1, 0, 0, 0)),
    chrono.ChFramed(pend1_joint_pos, chrono.ChQuaterniond(1, 0, 0, 0))
)
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)  


radius_p2 = 0.2
length_p2 = 1.5
mass_p2 = 1
Ixx_p2 = 0.5 * mass_p2 * radius_p2**2
Iyy_p2 = (mass_p2 / 12) * (3 * radius_p2**2 + length_p2**2)
pend_2.SetInertiaXX(chrono.ChVector3d(Ixx_p2, Iyy_p2, Iyy_p2))


cyl_p2 = chrono.ChVisualShapeCylinder(radius_p2, length_p2)
cyl_p2.SetColor(chrono.ChColor(0, 0.6, 0))  
pend_2.AddVisualShape(cyl_p2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_2.SetPos(chrono.ChVector3d(2.75, 0, 1))  


rev_2 = chrono.ChLinkLockRevolute()

pend1_joint_pos_p2 = chrono.ChVector3d(1, 0, 0)  
pend2_joint_pos = chrono.ChVector3d(-0.75, 0, 0)  
rev_2.Initialize(
    pend_1, pend_2,
    chrono.ChFramed(pend1_joint_pos_p2, chrono.ChQuaterniond(1, 0, 0, 0)),
    chrono.ChFramed(pend2_joint_pos, chrono.ChQuaterniond(1, 0, 0, 0))
)
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Double Pendulum Simulation')  
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
        print("Pendulum 1: ", pos_1.x, "  ", pos_1.y)
        print("Pendulum 2: ", pos_2.x, "  ", pos_2.y)
        lin_vel_1 = pend_1.GetPosDt()  
        lin_vel_2 = pend_2.GetPosDt()  
        print("Velocity 1: ", lin_vel_1.x, "  ", lin_vel_1.y)
        print("Velocity 2: ", lin_vel_2.x, "  ", lin_vel_2.y)
        log_info = False