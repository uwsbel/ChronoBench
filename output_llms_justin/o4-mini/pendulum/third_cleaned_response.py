import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(False)
sys.Add(ground)


ground_cyl = chrono.ChVisualShapeCylinder(0.2, 0.4)

ground.AddVisualShape(ground_cyl, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.0)))


rotZtoX = chrono.Q_from_AngAxis(math.pi/2, chrono.ChVectorD(0, 1, 0))


pend1 = chrono.ChBody()
pend1.SetMass(1.0)

pend1.SetInertiaXX(chrono.ChVectorD(0.1, 1.0, 1.0))
pend1.SetCollide(False)
sys.Add(pend1)


cyl1 = chrono.ChVisualShapeCylinder(0.05, 2.0)
cyl1.SetColor(chrono.ChColor(0.6, 0.0, 0.0))


frame1 = chrono.ChFrameD(chrono.ChVectorD(1.0, 0, 0), rotZtoX)
pend1.AddVisualShape(cyl1, frame1)


pend1.SetPos(chrono.ChVectorD(1.0, 0, 1.0))


rev1 = chrono.ChLinkLockRevolute()
rev1.Initialize(
    ground,
    pend1,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0),
                       chrono.ChQuaternionD(1, 0, 0, 0))
)
sys.AddLink(rev1)


pend2 = chrono.ChBody()
pend2.SetMass(0.5)
pend2.SetInertiaXX(chrono.ChVectorD(0.05, 0.5, 0.5))
pend2.SetCollide(False)
sys.Add(pend2)


cyl2 = chrono.ChVisualShapeCylinder(0.05, 2.0)
cyl2.SetColor(chrono.ChColor(0.0, 0.6, 0.0))

frame2 = chrono.ChFrameD(chrono.ChVectorD(1.0, 0, 0), rotZtoX)
pend2.AddVisualShape(cyl2, frame2)



pend2.SetPos(chrono.ChVectorD(3.0, 0, 1.0))


rev2 = chrono.ChLinkLockRevolute()
rev2.Initialize(
    pend1,
    pend2,
    chrono.ChCoordsysD(chrono.ChVectorD(2.0, 0, 1.0),
                       chrono.ChQuaternionD(1, 0, 0, 0))
)
sys.AddLink(rev2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()


log_info = True
time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    
    if log_info and sys.GetChTime() > 1.0:
        p1 = pend1.GetPos()
        p2 = pend2.GetPos()
        print(f"t = {sys.GetChTime():.3f} s")
        print(f"  pend1 COM = ({p1.x:.3f}, {p1.y:.3f}, {p1.z:.3f})")
        print(f"  pend2 COM = ({p2.x:.3f}, {p2.y:.3f}, {p2.z:.3f})")
        log_info = False