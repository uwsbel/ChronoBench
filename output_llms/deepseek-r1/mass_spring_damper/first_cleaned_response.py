import chrono
from chrono import ChSystemNSC, ChBodyEasyBox, ChLinkTSDA, ChVector3d, ChColor
from chrono.irrlicht import ChVisualSystemIrrlicht


system = ChSystemNSC()


ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)


mass = ChBodyEasyBox(0.2, 0.2, 0.2, 1000)  
mass.SetPos(ChVector3d(0, 0, 2))          
mass.GetVisualShape(0).SetColor(ChColor(1, 0, 0))  
system.Add(mass)


spring = ChLinkTSDA()
spring.Initialize(ground, mass, 
                  ChVector3d(0, 0, 0),   
                  ChVector3d(0, 0, 0))   
spring.SetSpringCoefficient(100)        
spring.SetDampingCoefficient(5)         
spring.SetRestLength(2)                 
system.Add(spring)


spring_shape = chrono.ChSpringShape(0.05, 80, 15)  
spring.AddVisualShape(spring_shape)


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddCamera(ChVector3d(0, -3, 2), ChVector3d(0, 0, 2))  
vis.AddTypicalLights()  


time_step = 0.001


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)