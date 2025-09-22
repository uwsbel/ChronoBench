import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)  
ground.SetCollide(False)   
sys.Add(ground)


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  
cyl_ground.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(cyl_ground, chrono.ChFrame(chrono.ChVector3d(0, 0, 1)))


length1 = 2.0
radius1 = 0.2
mass1 = 1.0


pend_1 = chrono.ChBody()
pend_1.SetBodyFixed(False)
pend_1.SetCollide(False)
pend_1.SetMass(mass1)

Ixx1 = (1/12) * mass1 * (3*radius1**2 + length1**2)
Iyy1 = 0.5 * mass1 * radius1**2
pend_1.SetInertiaXX(chrono.ChVector3d(Ixx1, Iyy1, Ixx1))
sys.Add(pend_1)


cyl_pend1 = chrono.ChVisualShapeCylinder(radius1, length1)
cyl_pend1.SetColor(chrono.ChColor(0.6, 0, 0))

pend_1.AddVisualShape(cyl_pend1, chrono.ChFrame(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngY(chrono.CH_C_PI_2)))



pend_1.SetPos(chrono.ChVector3d(length1/2, 0, 1))


rev_1 = chrono.ChLinkRevolute()
frame_rev1 = chrono.ChFrame(chrono.ChVector3d(0, 0, 1), chrono.QUNIT)  
rev_1.Initialize(ground, pend_1, frame_rev1)
sys.AddLink(rev_1)


length2 = 1.5
radius2 = 0.15
mass2 = 0.8


pend_2 = chrono.ChBody()
pend_2.SetBodyFixed(False)
pend_2.SetCollide(False)
pend_2.SetMass(mass2)
Ixx2 = (1/12) * mass2 * (3*radius2**2 + length2**2)
Iyy2 = 0.5 * mass2 * radius2**2
pend_2.SetInertiaXX(chrono.ChVector3d(Ixx2, Iyy2, Ixx2))
sys.Add(pend_2)


cyl_pend2 = chrono.ChVisualShapeCylinder(radius2, length2)
cyl_pend2.SetColor(chrono.ChColor(0, 0, 0.6))
pend_2.AddVisualShape(cyl_pend2, chrono.ChFrame(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngY(chrono.CH_C_PI_2)))




pend_2.SetPos(chrono.ChVector3d(length1 + length2/2, 0, 1))


rev_2 = chrono.ChLinkRevolute()
frame_rev2 = chrono.ChFrame(chrono.ChVector3d(length1, 0, 1), chrono.QUNIT)
rev_2.Initialize(pend_1, pend_2, frame_rev2)
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
        print("t = ", sys.GetChTime())
        print("Pendulum 1: ", pos_1.x, "  ", pos_1.y, "  ", pos_1.z)
        lin_vel_1 = pend_1.GetPos_dt()
        print("Pendulum 1 vel: ", lin_vel_1.x, "  ", lin_vel_1.y, "  ", lin_vel_1.z)
        print("Pendulum 2: ", pos_2.x, "  ", pos_2.y, "  ", pos_2.z)
        lin_vel_2 = pend_2.GetPos_dt()
        print("Pendulum 2 vel: ", lin_vel_2.x, "  ", lin_vel_2.y, "  ", lin_vel_2.z)
        log_info = False