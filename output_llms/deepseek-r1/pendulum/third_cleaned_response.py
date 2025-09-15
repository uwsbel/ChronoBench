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
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(1)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))


cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_1.SetPos(chrono.ChVector3d(1, 0, 1))


rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))


cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6))  
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


pend_2.SetPos(chrono.ChVector3d(3, 0, 1))


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFrameD(chrono.ChVector3d(2, 0, 1), chrono.QUNIT))
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Demo')  
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
        pos1 = pend_1.GetPos()
        pos2 = pend_2.GetPos()
        print("t = {:.3f}".format(sys.GetChTime()))
        print("Pendulum 1 position: {:.3f} {:.3f} {:.3f}".format(pos1.x, pos1.y, pos1.z))
        print("Pendulum 2 position: {:.3f} {:.3f} {:.3f}".format(pos2.x, pos2.y, pos2.z))
        log_info = False