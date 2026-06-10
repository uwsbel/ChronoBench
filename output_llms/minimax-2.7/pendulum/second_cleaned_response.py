import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math




sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))




ground = chrono.ChBody()
sys.AddBody(ground)               
ground.SetFixed(True)
ground.EnableCollision(False)


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))




pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)


pend_1.SetMass(2)                                 
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  


cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)   
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))       
pend_1.AddVisualShape(cyl_pend,
                      chrono.ChFramed(chrono.VNULL,
                                      chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_1.SetPos(chrono.ChVector3d(1, 0, 1))


pend_1.SetAngVel(chrono.ChVector3d(0, 0, 2))       




spherical_link = chrono.ChLinkLockSpherical()
spherical_link.Initialize(ground, pend_1,
                          chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                          chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(spherical_link)


sphere_vis = chrono.ChVisualShapeSphere(2)
sphere_vis.SetColor(chrono.ChColor(0, 1, 0))   
spherical_link.AddVisualShape(sphere_vis,
                              chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                              chrono.ChQuaterniond(1, 0, 0, 0)))




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Spherical joint pendulum on Moon')
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
        print("t =", sys.GetChTime())
        print("    Position:", pos_1.x, pos_1.y, pos_1.z)
        lin_vel_1 = pend_1.GetPosDt()
        print("    Linear velocity:", lin_vel_1.x, lin_vel_1.y, lin_vel_1.z)
        log_info = False