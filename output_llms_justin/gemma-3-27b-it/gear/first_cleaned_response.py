import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




body_truss = chrono.ChBodyEasy()
body_truss.SetBodyFixed(True)
body_truss.SetPos(chrono.ChVectorD(0, 0, 0))
body_truss.SetMass(1)
body_truss.AddBox(0.1, 0.1, 0.1)  
system.Add(body_truss)




body_arm = chrono.ChBodyEasy()
body_arm.SetMass(0.1)
body_arm.SetPos(chrono.ChVectorD(0, 0.1, 0))
body_arm.SetBodyFixed(False)
body_arm.AddBox(0.3, 0.02, 0.02)
system.Add(body_arm)


joint_arm = chrono.ChLinkRevolute()
joint_arm.Initialize(body_truss, body_arm, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
system.Add(joint_arm)






gear1 = chrono.ChBodyEasy()
gear1.SetMass(0.05)
gear1.SetPos(chrono.ChVectorD(0.15, 0.1, 0))
gear1.AddCylinder(0.05, 0.02)  
system.Add(gear1)


gear2 = chrono.ChBodyEasy()
gear2.SetMass(0.05)
gear2.SetPos(chrono.ChVectorD(-0.15, 0.1, 0))
gear2.AddCylinder(0.05, 0.02)  
system.Add(gear2)


joint_gear1 = chrono.ChLinkRevolute()
joint_gear1.Initialize(body_arm, gear1, chrono.ChCoordsys(chrono.ChVectorD(0.15, 0, 0)))
system.Add(joint_gear1)


joint_gear2 = chrono.ChLinkRevolute()
joint_gear2.Initialize(body_truss, gear2, chrono.ChCoordsys(chrono.ChVectorD(-0.15, 0, 0)))
system.Add(joint_gear2)




motor = chrono.ChFunction_Const()
motor.Set_y(10)  
motor_driver = chrono.ChLinkMotorRotationSpeed()
motor_driver.Initialize(gear2, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)), motor)
system.Add(motor_driver)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, -2))
vis.AddTypicalLights()


body_truss.SetMaterialMetallic(0.8, chrono.ChColor(0.5, 0.5, 0.5))
body_arm.SetMaterialMetallic(0.8, chrono.ChColor(0.3, 0.7, 0.3))
gear1.SetMaterialMetallic(0.8, chrono.ChColor(0.8, 0.8, 0.2))
gear2.SetMaterialMetallic(0.8, chrono.ChColor(0.8, 0.8, 0.2))




while vis.Run():
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3))
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.001)