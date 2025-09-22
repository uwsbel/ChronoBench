import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystem()
vis.SetWindowSize(1024, 768)
vis.SetLogo(" ")
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalCameraMovements()


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, chrono.ChVectorD(0, -5, 0))
sys.Add(ground)


mass = chrono.ChBodyEasySphere(sys, 1, 1000, True, chrono.ChVectorD(0, 0, 0))
sys.Add(mass)


link = chrono.ChLinkTSDA()
link.Initialize(mass, ground, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
sys.Add(link)


link.SetSpringCoefficient(100)
link.SetDampingCoefficient(10)


mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().radius = 1
mass_shape.SetColor(chrono.ChColor(1, 0, 0))
mass.AddVisualShape(mass_shape)

spring_shape = chrono.ChCylinderShape()
spring_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
spring_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 5)
spring_shape.GetCylinderGeometry().radius = 0.1
spring_shape.SetColor(chrono.ChColor(0, 1, 0))
link.AddVisualShape(spring_shape)


sys.SetMaxPenetrationRecoverySpeed(1.0)
sys.SetTol(1e-9)
sys.SetMaxItersSolverSpeed(100)


time_step = 0.01
end_time = 10
while sys.GetChTime() < end_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()

    
    print("Simulation time: ", sys.GetChTime())