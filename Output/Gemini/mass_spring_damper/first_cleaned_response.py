import pychrono.core as chrono
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mass = 1.0  
mass_body = chrono.ChBodyEasySphere(0.2,  
                                    mass,   
                                    True,   
                                    True)  
mass_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(mass_body)


ground = chrono.ChBodyEasyBox(1, 0.1, 1,  
                              1000,     
                              True,     
                              True)     
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)  
system.Add(ground)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(mass_body,               
                         ground,                  
                         False,                   
                         chrono.ChVectorD(0, 0, 0),  
                         chrono.ChVectorD(0, 0, 0)) 
system.Add(spring_damper)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


vis.GetGUIEnvironment().AddSkyBox(
    chrono.GetChronoDataFile("skybox/sky_06.jpg"),
    chrono.GetChronoDataFile("skybox/sky_02.jpg"),
    chrono.GetChronoDataFile("skybox/sky_01.jpg"),
    chrono.GetChronoDataFile("skybox/sky_04.jpg"),
    chrono.GetChronoDataFile("skybox/sky_05.jpg"),
    chrono.GetChronoDataFile("skybox/sky_03.jpg"), 1000)




while vis.Run():
    
    time = system.GetChTime()

    
    system.DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()