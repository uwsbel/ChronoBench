import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  


body_ground = chrono.ChBody()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVector3d(0, -1, 0))
body_ground.SetMass(1000)  
body_ground.SetInertiaXX(chrono.ChVector3d(1,1,1))
sys.Add(body_ground)


ground_shape = chrono.ChVisualShapeBox(10, 1, 10)
ground_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
body_ground.AddVisualShape(ground_shape)


body_mass = chrono.ChBody()
body_mass.SetMass(1)
body_mass.SetPos(chrono.ChVector3d(0, 1, 0))
body_mass.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
sys.Add(body_mass)


mass_shape = chrono.ChVisualShapeSphere(0.5)
mass_shape.SetColor(chrono.ChColor(0, 0, 1))
body_mass.AddVisualShape(mass_shape)


link_spring = chrono.ChLinkTSDA()
link_spring.Initialize(body_mass, body_ground, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(2, 2, -2))
vis.AddTypicalLights()
vis.AddSkyBox()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)