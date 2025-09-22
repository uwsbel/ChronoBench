import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()



sys.SetGravitationalAcceleration(chrono.ChVector(0, -1.62, 0))


ground = chrono.ChBody()
sys.Add(ground)
ground.SetBodyFixed(True)  
ground.SetCollide(False)  



sphere_ground = chrono.ChVisualShapeSphere(2)
sphere_ground.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
ground.AddVisualShape(sphere_ground, chrono.ChFrame(chrono.ChVector(0, 0, 1)))  


pend_1 = chrono.ChBody()
sys.Add(pend_1)  
pend_1.SetBodyFixed(False)  
pend_1.SetCollide(False)  

pend_1.SetMass(2)

pend_1.SetInertiaXX(chrono.ChVector(0.4, 1.5, 1.5))



cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
pend_1.AddVisualShape(cyl_1, chrono.ChFrame(chrono.VNULL, chrono.Q_from_AngY(chrono.CH_C_PI_2)))  


pend_1.SetPos(chrono.ChVector(1, 0, 1))


pend_1.SetWvel_par(chrono.ChVector(0, 0, 2.0))  


sph_1 = chrono.ChLinkSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChFrame(chrono.ChVector(0, 0, 1)))
sys.Add(sph_1)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector(0, 3, 6))
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
        print("     ", pos_1.x, "  ", pos_1.y, "  ", pos_1.z)
        lin_vel_1 = pend_1.GetPos_dt()  
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y, "  ", lin_vel_1.z)
        ang_vel_1 = pend_1.GetWvel_loc()
        print("Angular velocity: ", ang_vel_1.x, "  ", ang_vel_1.y, "  ", ang_vel_1.z)
        log_info = False