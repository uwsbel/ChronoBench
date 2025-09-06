import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




ground = chrono.ChBody()
ground.SetBodyFixed(True)  
system.Add(ground)


mass = chrono.ChBody()
mass.SetMass(1.0)  
mass.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
mass.SetPos(chrono.ChVectorD(0, 1, 0))  
system.Add(mass)




spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(ground, mass, False,
                        chrono.ChVectorD(0, 0, 0),  
                        chrono.ChVectorD(0, 0, 0))  


spring_damper.SetSpringCoefficient(100.0)  
spring_damper.SetDampingCoefficient(5.0)   
spring_damper.SetRestLength(1.0)          

system.Add(spring_damper)


ground_vis = chrono.ChBoxShape()
ground_vis.GetBoxGeometry().SetLengths(chrono.ChVectorD(10, 0.1, 10))
ground_vis.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.AddAsset(ground_vis)

mass_vis = chrono.ChSphereShape()
mass_vis.GetSphereGeometry().rad = 0.2
mass.AddAsset(mass_vis)


spring_vis = chrono.ChSpringShape()
spring_vis.SetSpringCoilDiameter(0.1)
spring_vis.SetSpringFreeLength(1.0)
spring_vis.SetSpringNaturalLength(1.0)
spring_vis.SetSpringWindings(20)
spring_damper.AddAsset(spring_vis)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 3))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1, 5, -2), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    mass_pos = mass.GetPos()
    print(f"Time: {system.GetChTime():.2f}s, Mass position: {mass_pos.y:.3f}m")