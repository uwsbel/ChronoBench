import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


time_step = 0.005
simulation_duration = 10




body_floor = chrono.ChBodyEasy()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
body_floor.SetShapeBoxes(0.5, 0.1, 1)
body_floor.SetCollide(True)
system.Add(body_floor)




crank_length = 0.5
crank_radius = 0.2
crank_mass = 1.0
crank_inertia = 0.01

body_crank = chrono.ChBodyEasy()
body_crank.SetMass(crank_mass)
body_crank.SetInertia(crank_inertia)
body_crank.SetPos(chrono.ChVectorD(0, 0.5, 0))
body_crank.SetShapeBoxes(crank_length, 0.05, 0.05)
body_crank.SetCollide(True)
system.Add(body_crank)




rod_length = 0.6
rod_mass = 0.5
rod_inertia = 0.005

body_rod = chrono.ChBodyEasy()
body_rod.SetMass(rod_mass)
body_rod.SetInertia(rod_inertia)
body_rod.SetPos(chrono.ChVectorD(0, 0.5, 0))
body_rod.SetShapeBoxes(rod_length, 0.05, 0.05)
body_rod.SetCollide(True)
system.Add(body_rod)




piston_length = 0.1
piston_mass = 0.3
piston_inertia = 0.003

body_piston = chrono.ChBodyEasy()
body_piston.SetMass(piston_mass)
body_piston.SetInertia(piston_inertia)
body_piston.SetPos(chrono.ChVectorD(0, 0.5, 0))
body_piston.SetShapeBoxes(0.05, 0.05, piston_length)
body_piston.SetCollide(True)
system.Add(body_piston)






joint_crank_floor = chrono.ChLinkRevolute()
joint_crank_floor.Initialize(body_crank, body_floor, chrono.ChCoordsys(chrono.ChVectorD(0, 0.5, 0)))
system.Add(joint_crank_floor)


joint_rod_crank = chrono.ChLinkRevolute()
joint_rod_crank.Initialize(body_rod, body_crank, chrono.ChCoordsys(chrono.ChVectorD(crank_length, 0.5, 0)))
system.Add(joint_rod_crank)


joint_rod_piston = chrono.ChLinkRevolute()
joint_rod_piston.Initialize(body_rod, body_piston, chrono.ChCoordsys(chrono.ChVectorD(rod_length, 0.5, 0)))
system.Add(joint_rod_piston)


joint_piston_floor = chrono.ChLinkPrismatic()
joint_piston_floor.Initialize(body_piston, body_floor, chrono.ChCoordsys(chrono.ChVectorD(0, 0.5, 0)))
joint_piston_floor.SetFixed(True)
system.Add(joint_piston_floor)




motor = chrono.ChFunction_Const()
motor.Set_yval(2.0)  
motor.Set_time_function(chrono.ChFunction_Time())

link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(body_crank, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
link_motor.Set_motor_function(motor)
system.Add(link_motor)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 2, -2))
vis.AddTypicalLights()




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)