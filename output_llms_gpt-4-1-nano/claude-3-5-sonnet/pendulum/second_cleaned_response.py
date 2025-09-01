import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -1.62, 0))  


ground = chrono.ChBodyEasyBox(5, 0.2, 5, 1000, True)
sys.Add(ground)
ground.SetBodyFixed(True)
ground.SetCollide(False)


sphere = chrono.ChVisualShapeSphere(0.2)
ground.AddVisualShape(sphere, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  
pend_1.SetCollide(False)


cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.ChVectorD(0, -0.75, 0), chrono.Q_from_AngY(chrono.CH_C_PI_2)))


pend_1.SetPos(chrono.ChVectorD(0, -1.5, 0))


pend_1.SetWvel_loc(chrono.ChVectorD(0, 0, 1))


sph_1 = chrono.ChLinkSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
sys.AddLink(sph_1)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Pendulum Simulation')
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
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPos_dt()
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        log_info = False