import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.8))
system.Add(ground)


mass = chrono.ChBodyEasySphere(0.5, 1000, True, True)
mass.SetPos(chrono.ChVectorD(0, 2, 0))
mass.SetMass(1.0)
mass.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
mass.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
system.Add(mass)


spring = chrono.ChLinkTSDA()
spring.Initialize(
    mass,                                
    ground,                              
    False,                               
    chrono.ChVectorD(0, 2, 0),           
    chrono.ChVectorD(0, 0, 0),           
)


spring.SetSpringCoefficient(50.0)        
spring.SetDampingCoefficient(1.0)        
spring.SetRestLength(2.0)                


spring.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 10))
spring.GetVisualShape(0).SetColor(chrono.ChColor(0.0, 0.7, 0.0))

system.Add(spring)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Mass-Spring-Damper System")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(5, 2, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


time_step = 0.001
realtime_timer = chrono.ChRealtimeStepTimer()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    system.DoStepDynamics(time_step)
    realtime_timer.Spin(time_step)