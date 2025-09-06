import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




floor = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
floor.SetBodyFixed(True)
floor.GetVisualModel().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(floor)


crank = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 10, True, True)
crank.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(crank)


rod = chrono.ChBodyEasyBox(0.2, 1.5, 0.2, 5, True, True)
rod.SetPos(chrono.ChVectorD(2, 0.5, 0))
system.Add(rod)


piston = chrono.ChBodyEasyBox(1, 0.5, 1, 20, True, True)
piston.SetPos(chrono.ChVectorD(4, 0.5, 0))
system.Add(piston)





revolute_ground_crank = chrono.ChLinkLockRevolute()
revolute_ground_crank.Initialize(floor, crank, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)))
system.Add(revolute_ground_crank)


revolute_crank_rod = chrono.ChLinkLockRevolute()
revolute_crank_rod.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0.5, 0)))
system.Add(revolute_crank_rod)


revolute_rod_piston = chrono.ChLinkLockRevolute()
revolute_rod_piston.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(3.5, 0.5, 0)))
system.Add(revolute_rod_piston)


prismatic_piston_ground = chrono.ChLinkLockPrismatic()
prismatic_piston_ground.Initialize(piston, floor, chrono.ChCoordsysD(chrono.ChVectorD(4, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(prismatic_piston_ground)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 3, 5), chrono.ChVectorD(0, 0.5, 0))
vis.AddTypicalLights()
vis.SetSymbolScale(0.02)
vis.SetSymbolThickness(0.005)
vis.SetShadows(True)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(0.01)