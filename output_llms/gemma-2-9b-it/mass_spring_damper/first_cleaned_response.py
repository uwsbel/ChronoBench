import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("..")  
system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 10, 1, 1000)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


mass = chrono.ChBodyEasySphere(0.5, 100)
mass.SetPos(chrono.ChVectorD(0, 1, 0))
system.Add(mass)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(ground, mass, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.Add(spring_damper)


vis = chronoirr.ChIrrApp(system, 'Mass-Spring-Damper')
vis.AddCamera(chrono.ChVectorD(5, 5, 5))
vis.AddLight(chrono.ChVectorD(10, 10, 10))


vis.SetWindowSize(1024, 768)
vis.SetRenderMode(chronoirr.CH_RENDERING_MODE_WIREFRAME)


vis.Start()
while vis.GetDevice().run():
    system.DoStepDynamics(1e-3)
    vis.Render()