import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)


ground_cyl = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(ground_cyl, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))


pend1 = chrono.ChBody()
sys.Add(pend1)
pend1.SetFixed(False)
pend1.EnableCollision(False)
pend1.SetMass(1)
pend1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))
pend1.SetPos(chrono.ChVector3d(0, 0, 1))  


pend1_cyl = chrono.ChVisualShapeCylinder(0.2, 2)
pend1_cyl.SetColor(chrono.ChColor(0.6, 0, 0))
pend1.AddVisualShape(pend1_cyl, chrono.ChFrameD(chrono.ChVector3d(1, 0, 0), 
                                              chrono.QuatFromAngleY(chrono.CH_PI_2)))


rev1 = chrono.ChLinkLockRevolute()
rev1.Initialize(ground, pend1, 
               chrono.ChFrameD(chrono.ChVector3d(0, 0, 1), 
                              chrono.QUNIT))
sys.AddLink(rev1)


pend2 = chrono.ChBody()
sys.Add(pend2)
pend2.SetFixed(False)
pend2.EnableCollision(False)
pend2.SetMass(1)
pend2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))
pend2.SetPos(chrono.ChVector3d(2, 0, 1))  


pend2_cyl = chrono.ChVisualShapeCylinder(0.15, 1.8)  
pend2_cyl.SetColor(chrono.ChColor(0, 0.6, 0))
pend2.AddVisualShape(pend2_cyl, chrono.ChFrameD(chrono.ChVector3d(0.9, 0, 0), 
                                              chrono.QuatFromAngleY(chrono.CH_PI_2)))


rev2 = chrono.ChLinkLockRevolute()
rev2.Initialize(pend1, pend2, 
               chrono.ChFrameD(chrono.ChVector3d(2, 0, 1), 
                              chrono.QUNIT))
sys.AddLink(rev2)


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
        print(f"t = {sys.GetChTime():.3f} s")
        print(f"Pendulum 1 position: {pend1.GetPos().x:.3f}, {pend1.GetPos().y:.3f}")
        print(f"Pendulum 2 position: {pend2.GetPos().x:.3f}, {pend2.GetPos().y:.3f}")
        log_info = False