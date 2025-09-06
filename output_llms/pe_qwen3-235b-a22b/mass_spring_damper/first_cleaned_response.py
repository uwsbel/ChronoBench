import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(2, 0.1, 2))
ground_shape.SetColor(chrono.ChColor(0.6, 0.6, 0.6))  
ground.AddVisualShape(ground_shape)
sys.Add(ground)


mass = chrono.ChBody()
mass.SetMass(1.0)
mass.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))  
mass.SetPos(chrono.ChVector3d(0, -1, 0))  
mass_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.5, 0.5, 0.5))
mass_shape.SetColor(chrono.ChColor(1, 0, 0))  
mass.AddVisualShape(mass_shape)
sys.Add(mass)


tsda = chrono.ChLinkTSDA()

tsda.Initialize(ground, mass, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))

tsda.SetSpringCoefficient(100)  
tsda.SetDampingCoefficient(5)    

spring_shape = chrono.ChVisualShapeSpring(0.05, 1.0, 10, 20)  
tsda.AddVisualShape(spring_shape)
sys.Add(tsda)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 3))  
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)