import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
floor.GetVisualModel().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(floor)


crank = chrono.ChBodyEasyCylinder(0.2, 1, 10, True, True)
crank.SetPos(chrono.ChVectorD(0, 2, 0))
crank.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
system.Add(crank)


rod_length = 3.0
rod = chrono.ChBodyEasyBox(0.1, 0.1, rod_length, 5, True, True)
rod.SetPos(chrono.ChVectorD(0, 2 + rod_length/2, 0))
system.Add(rod)


piston = chrono.ChBodyEasyBox(0.5, 0.2, 0.5, 5, True, True)
piston.SetPos(chrono.ChVectorD(0, 2 + rod_length, 0))
system.Add(piston)



joint_crank_ground = chrono.ChLinkLockRevolute()
joint_crank_ground.Initialize(floor, crank, chrono.ChCoordinatesysD(chrono.ChVectorD(0, 2, 0)))
system.Add(joint_crank_ground)


joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(crank, rod, chrono.ChCoordinatesysD(chrono.ChVectorD(1, 2, 0)))  
system.Add(joint_crank_rod)


joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(rod, piston, chrono.ChCoordinatesysD(chrono.ChVectorD(0, 2 + rod_length, 0)))
system.Add(joint_rod_piston)


joint_piston_ground = chrono.ChLinkLockPrismatic()
joint_piston_ground.Initialize(floor, piston, chrono.ChCoordinatesysD(chrono.ChVectorD(0, 2 + rod_length, 0)))
joint_piston_ground.SetLimitSpeed(0, 0)  
system.Add(joint_piston_ground)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0, 2, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 2, 0))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 10, chrono.ChColor(1, 1, 1))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(0.01)

    
    print("Piston position:", piston.GetPos().y)