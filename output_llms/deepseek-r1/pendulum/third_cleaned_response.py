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


pend1 = chrono.ChBody()
sys.AddBody(pend1)
pend1.SetFixed(False)
pend1.EnableCollision(False)
pend1.SetMass(1)

pend1.SetInertiaXX(chrono.ChVector3d(0.02, 0.3433, 0.3433))


cyl1 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl1.SetColor(chrono.ChColor(0.6, 0, 0))
pend1.AddVisualShape(cyl1, chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
pend1.SetPos(chrono.ChVector3d(1, 0, 1))  


rev1 = chrono.ChLinkLockRevolute()
rev1.Initialize(ground, pend1, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))
sys.AddLink(rev1)


pend2 = chrono.ChBody()
sys.AddBody(pend2)
pend2.SetFixed(False)
pend2.EnableCollision(False)
pend2.SetMass(1)
pend2.SetInertiaXX(chrono.ChVector3d(0.02, 0.3433, 0.3433))  


cyl2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl2.SetColor(chrono.ChColor(0, 0.6, 0))
pend2.AddVisualShape(cyl2, chrono.ChFrameD(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
pend2.SetPos(chrono.ChVector3d(3, 0, 1))  


rev2 = chrono.ChLinkLockRevolute()
rev2.Initialize(pend1, pend2, chrono.ChFrameD(chrono.ChVector3d(2, 0, 1)))
sys.AddLink(rev2)


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
        print("t = {0:.3f}".format(sys.GetChTime()))
        print("Pendulum 1 position:", pend1.GetPos())
        print("Pendulum 2 position:", pend2.GetPos())
        log_info = False