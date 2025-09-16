import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))


pend = chrono.ChBody()
sys.AddBody(pend)
pend.SetFixed(False)  
pend.EnableCollision(False)  
pend.SetMass(2)  
pend.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  


cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)  
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))  
pend.AddVisualShape(cyl_pend, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend.SetPos(chrono.ChVector3d(1, 0, 1))  
pend.SetWvel_par(chrono.ChVector3d(0, 1, 0))  


spherical_joint = chrono.ChLinkLockSpherical()
spherical_joint.Initialize(ground, pend, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))
sys.AddLink(spherical_joint)


sphere_vis = chrono.ChVisualShapeSphere(2)  
spherical_joint.AddVisualShape(sphere_vis, chrono.ChFramed())


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Spherical Pendulum on the Moon')
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
        pos = pend.GetPos()
        print(f"t = {sys.GetChTime()}")
        print(f"Position: {pos.x}, {pos.y}")
        lin_vel = pend.GetPosDt()
        print(f"Velocity: {lin_vel.x}, {lin_vel.y}")
        log_info = False