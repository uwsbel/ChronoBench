import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, 4))
vis.AddTypicalLights()


mass_body = chrono.ChBody()
mass_body.SetPos(chrono.ChVectorD(0, 1, 0))
mass_body.SetMass(10)
mass_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))

ground_body = chrono.ChBody()
ground_body.SetBodyFixed(True)


link = chrono.ChLinkTSDA()
link.Initialize(mass_body, ground_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
link.SetSpringCoefficient(100)
link.SetDampingCoefficient(10)


mass_shape = chrono.ChVisualShapeSphere(0.2)
mass_body.AddVisualShape(mass_shape)

spring_shape = chrono.ChVisualShapeCylinder(0.05, 1)
link.AddVisualShape(spring_shape)


sys.Add(mass_body)
sys.Add(ground_body)
sys.Add(link)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()