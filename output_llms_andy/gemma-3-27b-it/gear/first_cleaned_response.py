import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  






truss = chrono.ChBodyEasy()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetMass(1)
truss.AddBox(0.1, 0.1, 0.1)  
system.Add(truss)






bar = chrono.ChBodyEasy()
bar.SetMass(1)
bar.SetPos(chrono.ChVectorD(0, 0.1, 0))
bar.SetBodyFixed(False)
bar.AddCylinder(0.2, 0.05)  
system.Add(bar)


revolute_bar = chrono.ChLinkRevolute()
revolute_bar.Initialize(truss, bar, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
system.Add(revolute_bar)







gear1_radius = 0.1
gear1_center = chrono.ChVectorD(0.2, 0.1, 0)
gear1_thickness = 0.05


gear2_radius = 0.2
gear2_center = chrono.ChVectorD(0.4, 0.1, 0)
gear2_thickness = 0.05


gear1 = chrono.ChBodyEasy()
gear1.SetMass(1)
gear1.SetPos(gear1_center)
gear1.SetBodyFixed(False)
gear1.AddCylinder(gear1_radius, gear1_thickness)
system.Add(gear1)


gear2 = chrono.ChBodyEasy()
gear2.SetMass(1)
gear2.SetPos(gear2_center)
gear2.SetBodyFixed(False)
gear2.AddCylinder(gear2_radius, gear2_thickness)
system.Add(gear2)


link_gear = chrono.ChLinkMateRevolute()
link_gear.Initialize(gear1, gear2, chrono.ChCoordsys(gear1_center + chrono.ChVectorD(gear1_radius, 0, 0)))
system.Add(link_gear)






motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, gear1, chrono.ChCoordsys(gear1_center))
motor.Set_Rot_speed(2)  
motor.Set_Torque(10)
system.Add(motor)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -2))
vis.AddTypicalLights()


truss.SetMaterialMetallic(0.8, chrono.ChColor(0.5, 0.5, 0.5))
bar.SetMaterialMetallic(0.8, chrono.ChColor(0.7, 0.7, 0.7))
gear1.SetMaterialMetallic(0.8, chrono.ChColor(0.3, 0.6, 0.9))
gear2.SetMaterialMetallic(0.8, chrono.ChColor(0.3, 0.6, 0.9))






time_step = 0.001

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)