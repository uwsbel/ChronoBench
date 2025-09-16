import chrono as chrono
import irrrlicht as irrl


chrono.ChSystemSMC::SetDefaultSolverType(chrono.ChSystemSOLVER_SRIS)
chrono.ChSystemSMC::SetDefaultCollisionType(chrono.ChSystemSOLVER_CONTACT_CD)
my_system = chrono.ChSystemSMC()


mass = my_system.AddBody(chrono.ChBodyEasyBox(1, 1, 1, 100, True))
ground = my_system.AddBody(chrono.ChBodyEasyBox(10, 0.1, 10, 1000, False))


mass.SetPos(chrono.ChVectorD(0, 5, 0))
ground.SetPos(chrono.ChVectorD(0, 0, 0))


link = chrono.ChLinkTSDA()
link.Initialize(mass, ground, chrono.ChVectorD(0, 5, 0))
my_system.AddLink(link)


my_vis = irrl.ChVisualSystemIrrlicht()
my_vis.AttachSystem(my_system)
my_vis.SetWindowSize(800, 600)
my_vis.SetWindowTitle("PyChrono Mass-Spring-Damper Demo")
my_vis.AddLogo()
my_vis.AddSkyBox()
my_vis.AddCamera(chrono.ChVectorD(0, 10, -15), chrono.ChVectorD(0, 0, 0))
my_vis.EnableGUISkyDome(False)


mass_shape = my_system.AddVisualShape(mass, chrono.ChBoxShape())
mass_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

spring_shape = my_system.AddVisualShape(ground, chrono.ChCylinderShape())
spring_shape.SetPos(chrono.ChVectorD(0, 5, 0))
spring_shape.SetColor(chrono.ChColor(0.5, 0, 0.5))


for i in range(1000):
    my_system.DoStepDynamics(1e-3)
    my_vis.BeginScene()
    my_vis.DrawAll()
    my_vis.EndScene()