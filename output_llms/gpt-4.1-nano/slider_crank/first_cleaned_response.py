import pychrono.core as chrono


system = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
floor.SetPosition(chrono.ChVectorD(0, -0.25, 0))
floor.SetBodyFixed(True)
system.Add(floor)


crank_radius = 0.5
crank_length = 0.1
crank_mass = 1.0

crank = chrono.ChBodyEasyCylinder(crank_radius, crank_length, crank_mass, True, True)
crank.SetMass(crank_mass)
crank.SetPos(chrono.ChVectorD(0, 0.0, 0))
system.Add(crank)


crank.SetIdentifier(1)


rod_length = 2.0
rod_radius = 0.05
rod_mass = 0.5
connecting_rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length, rod_mass, True, True)
connecting_rod.SetMass(rod_mass)
connecting_rod.SetPos(chrono.ChVectorD(rod_length / 2, 0, 0))
system.Add(connecting_rod)


piston_radius = 0.3
piston_height = 0.2
piston_mass = 1.0
piston = chrono.ChBodyEasyCylinder(piston_radius, piston_height, piston_mass, True, True)
piston.SetMass(piston_mass)
piston.SetPos(chrono.ChVectorD(rod_length + 0.5, 0, 0))
system.Add(piston)


crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crank, floor, chrono.ChVectorD(0, 0, 0))
system.Add(crank_joint)


motor_speed = 10  

crank_motor = chrono.ChLinkMotorSpeed()
crank_motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
crank_motor.SetSpeed(motor_speed)
system.Add(crank_motor)



rod_crank_joint = chrono.ChLinkLockRevolute()
rod_crank_joint.Initialize(connecting_rod, crank, chrono.ChVectorD(0, 0, 0))
system.Add(rod_crank_joint)




rod_end_point = chrono.ChVectorD(rod_length, 0, 0)

rod_piston_joint = chrono.ChLinkLockRevolute()
rod_piston_joint.Initialize(connecting_rod, piston, roe_piston_point := chrono.ChFrameD(chrono.ChVectorD(rod_length, 0, 0)))
system.Add(rod_piston_joint)


piston_constraint = chrono.ChLinkLockPrismatic()
piston_constraint.Initialize(piston, floor, chrono.ChFrameD(chrono.ChVectorD(rod_length + 0.5, 0, 0), chrono.Q_from_YOY(1, 0, 0)))
system.Add(piston_constraint)





system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


visualization = chrono.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('Crank-Slider Mechanism Simulation')


visualization.SetCamera(chrono.ChVectorD(-5, 2, 10),  
                        chrono.ChVectorD(0, 0, 0))   


visualization.EnableLighting(True)
visualization.AddLights()
visualization.LightsBegin()
visualization.LightsSetBack(chrono.ChColor(0.2, 0.2, 0.2))
visualization.LightsSetFront(chrono.ChColor(0.8, 0.8, 0.8))
visualization.LightsEnd()






dt = 0.005  
while visualization.Run():
    system.DoStepDynamics(dt)