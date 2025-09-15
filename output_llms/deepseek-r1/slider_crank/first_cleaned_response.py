import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.AddBody(ground)


crank = chrono.ChBodyEasyCylinder(0.02, 0.15, 7800)  
crank.SetPos(chrono.ChVectorD(0, 0, 0))
crank.SetRot(chrono.Q_from_AngY(chrono.CH_C_PI_2))  
system.AddBody(crank)


rod = chrono.ChBodyEasyBox(0.5, 0.015, 0.015, 7800)  
rod.SetPos(chrono.ChVectorD(0.25, 0, 0))
system.AddBody(rod)


piston = chrono.ChBodyEasyCylinder(0.04, 0.06, 7800)
piston.SetPos(chrono.ChVectorD(0.7, 0, 0))
piston.SetRot(chrono.Q_from_AngY(chrono.CH_C_PI_2))  
system.AddBody(piston)



revolute_ground_crank = chrono.ChLinkLockRevolute()
revolute_ground_crank.Initialize(ground, crank, 
                                chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(revolute_ground_crank)


revolute_crank_rod = chrono.ChLinkLockRevolute()
revolute_crank_rod.Initialize(crank, rod, 
                             chrono.ChCoordsysD(chrono.ChVectorD(0.15, 0, 0)))
system.AddLink(revolute_crank_rod)


revolute_rod_piston = chrono.ChLinkLockRevolute()
revolute_rod_piston.Initialize(rod, piston, 
                              chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0)))
system.AddLink(revolute_rod_piston)


prismatic_ground_piston = chrono.ChLinkLockPrismatic()
prismatic_ground_piston.Initialize(ground, piston, 
                                  chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)),
                                  chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
prismatic_ground_piston.SetMotionAxis(chrono.ChVectorD(1, 0, 0))  
system.AddLink(prismatic_ground_piston)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  
system.AddLink(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, -1, 0.2), chrono.ChVectorD(0.5, 0, 0))
vis.AddTypicalLights()


texture = chrono.ChTexture(chrono.GetChronoDataFile('textures/bluewhite.png'))
ground.AddVisualShape(chrono.ChBoxShape(0.2, 0.2, 0.1)).SetTexture(texture)
crank.GetVisualShape(0).SetTexture(texture)
rod.GetVisualShape(0).SetTexture(texture)
piston.GetVisualShape(0).SetTexture(texture)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)