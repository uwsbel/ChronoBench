import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  




truss = chrono.ChBodyEasyBox(4, 0.5, 0.1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
system.Add(truss)


bar_length = 2.0
bar = chrono.ChBodyEasyBox(bar_length, 0.2, 0.1, 10, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 0.2))
system.Add(bar)


sun_gear_radius = 0.5
sun_gear = chrono.ChBodyEasyCylinder(sun_gear_radius, 0.2, 1000, True, True)
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0.3))
sun_gear.SetBodyFixed(True)
system.Add(sun_gear)


planet_gear_radius = 0.3
planet_gear = chrono.ChBodyEasyCylinder(planet_gear_radius, 0.2, 5, True, True)
planet_gear.SetPos(chrono.ChVectorD(bar_length/2, 0, 0.3))
system.Add(planet_gear)


link_bar_planet = chrono.ChLinkLockRevolute()
link_bar_planet.Initialize(planet_gear, bar, chrono.ChCoordsysD(chrono.ChVectorD(bar_length/2, 0, 0.3)))
system.Add(link_bar_planet)


gear_ratio = sun_gear_radius / planet_gear_radius
link_gear = chrono.ChLinkGear()
link_gear.Initialize(sun_gear, planet_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.3)),
                     chrono.ChCoordsysD(chrono.ChVectorD(bar_length/2, 0, 0.3)), gear_ratio)
system.Add(link_gear)


link_ground_bar = chrono.ChLinkLockRevolute()
link_ground_bar.Initialize(bar, truss, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.2)))
system.Add(link_ground_bar)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.2)))
motor.SetMotorSpeed(1.0)  
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Epicyclic Gear System")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 0, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 10, 1, 10, 40, 512)


truss.GetVisualModel().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.5, 0.5, 0.5)))
bar.GetVisualModel().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.2, 0.2, 0.8)))
sun_gear.GetVisualModel().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.8, 0.2, 0.2)))
planet_gear.GetVisualModel().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.2, 0.8, 0.2)))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)