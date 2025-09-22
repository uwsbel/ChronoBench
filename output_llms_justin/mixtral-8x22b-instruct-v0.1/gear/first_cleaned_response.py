import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


physics_time = 10  
timestep = 0.01  

my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  


fixed_truss = chrono.ChBodyEasyBox(2, 2, 0.1, 1000, True, True)  
fixed_truss.SetPos(chrono.ChVectorD(0, 0, 0))
fixed_truss.SetBodyFixed(True)
my_system.Add(fixed_truss)


rotating_bar = chrono.ChBodyEasyBox(0.2, 2, 0.2, 1000, True, True)
rotating_bar.SetPos(chrono.ChVectorD(0, 0, 1))
rotating_bar.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
my_system.Add(rotating_bar)


gear1 = chrono.ChBodyEasyCylinder(0.1, 0.2, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(-1, 0, 1))
my_system.Add(gear1)


gear2 = chrono.ChBodyEasyCylinder(0.1, 0.2, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(1, 0, 1))
my_system.Add(gear2)


gear_motor = chrono.ChLinkMotorRotationSpeed()
gear_motor.Initialize(gear1, gear2, chrono.ChVectorD(-1, 0, 1), chrono.ChVectorD(1, 0, 1))
gear_motor.Set_speed_reference(10)  
my_system.Add(gear_motor)


gear_constraint = chrono.ChLinkLockRevolute()
gear_constraint.Initialize(rotating_bar, gear1, chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(-1, 0, 1))
my_system.Add(gear_constraint)


gear_constraint2 = chrono.ChLinkLockRevolute()
gear_constraint2.Initialize(rotating_bar, gear2, chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(1, 0, 1))
my_system.Add(gear_constraint2)


visualizer = chronoirr.ChVisualSystemIrrlicht()
visualizer.SetWindowSize(800, 600)
visualizer.SetWindowTitle('Epicyclic Gears Simulation')
visualizer.Initialize()
visualizer.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visualizer.AddSkyBox()
visualizer.AddCamera(chrono.ChVectorD(0, -5, 2))
visualizer.AddLight(chrono.ChVectorD(0, 0, 10), chrono.ChColor(0.8, 0.8, 0.8))


mat_red = chronoirr.ChIrrTools.ConvertMaterial(chrono.ChColor(0.8, 0, 0))
mat_blue = chronoirr.ChIrrTools.ConvertMaterial(chrono.ChColor(0, 0, 0.8))
mat_gray = chronoirr.ChIrrTools.ConvertMaterial(chrono.ChColor(0.5, 0.5, 0.5))

fixed_truss.GetMaterialSurface().SetRGBcolor(mat_gray)
rotating_bar.GetMaterialSurface().SetRGBcolor(mat_red)
gear1.GetMaterialSurface().SetRGBcolor(mat_blue)
gear2.GetMaterialSurface().SetRGBcolor(mat_blue)


while visualizer.Run():
    visualizer.BeginScene()
    visualizer.DrawAll()
    visualizer.EndScene()

    my_system.DoStepDynamics(timestep)

visualizer.CloseWindow()