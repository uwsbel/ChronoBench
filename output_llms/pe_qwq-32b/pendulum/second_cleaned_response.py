import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()
sys.SetGravity(chrono.ChVectorD(0, -1.62, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetBodyFixed(True)  
ground.SetCollide(False)  


ground_cyl = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(ground_cyl, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))  


joint_sphere = chrono.ChVisualShapeSphere(2)  
ground.AddVisualShape(joint_sphere, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))  


pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetBodyFixed(False)  
pend_1.SetCollide(False)  
pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  


pend_cyl = chrono.ChVisualShapeCylinder(0.1, 1.5)  
pend_cyl.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(pend_cyl, chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD.Y())))  


pend_1.SetPos(chrono.ChVectorD(1, 0, 1))  


pend_1.SetWvel_loc(chrono.ChVectorD(0, 0, 1))  


spher_1 = chrono.ChLinkLockSpherical()
spher_1.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))  
sys.AddLink(spher_1)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum Simulation')
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
        pos = pend_1.GetPos()
        print(f"t = {sys.GetChTime():.2f}")
        print(f"Position: ({pos.x:.2f}, {pos.y:.2f})")
        lin_vel = pend_1.GetPos_dt()
        print(f"Linear Velocity: ({lin_vel.x:.2f}, {lin_vel.y:.2f})")
        log_info = False