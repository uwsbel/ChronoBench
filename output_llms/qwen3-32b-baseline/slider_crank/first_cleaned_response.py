import pychrono as chrono
import pychrono.irrlicht as irr
import math


my_system = chrono.ChSystemNSC()
my_system.Set_Gravity(chrono.ChVectorD(0, 0, -9.81))  


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, 0, -0.05))  
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
my_system.Add(floor)


crank = chrono.ChBodyEasyCylinder(0.05, 0.2, 1000, True, True)
crank.SetPos(chrono.ChVectorD(0.1, 0, 0))  
crank.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/grey.png"))
my_system.Add(crank)


motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("Crank Motor")
motor.Initialize(floor, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.05)))
motor.Set_speed_function(chrono.ChFunctionConst(chrono.CH_C_PI))  
my_system.AddLink(motor)


rod = chrono.ChBodyEasyBox(0.5, 0.05, 0.05, 1000, True, True)
rod.SetPos(chrono.ChVectorD(0.45, 0, 0))  
rod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/white.png"))
my_system.Add(rod)


rod_rev_joint = chrono.ChLinkLockRevolute()
csys_crank = chrono.ChCoordsysD(chrono.ChVectorD(0.1, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
csys_rod = chrono.ChCoordsysD(chrono.ChVectorD(-0.25, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
rod_rev_joint.Initialize(crank, rod, csys_crank, csys_rod)
my_system.AddLink(rod_rev_joint)


piston = chrono.ChBodyEasyBox(0.2, 0.1, 0.05, 1000, True, True)
piston.SetPos(chrono.ChVectorD(0.7, 0, 0))  
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/red.png"))
my_system.Add(piston)


rod_piston_rev = chrono.ChLinkLockRevolute()
csys_rod_joint = chrono.ChCoordsysD(chrono.ChVectorD(0.25, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
csys_piston_joint = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
rod_piston_rev.Initialize(rod, piston, csys_rod_joint, csys_piston_joint)
my_system.AddLink(rod_piston_rev)


prismatic = chrono.ChLinkLockPrismatic()
csys_floor_joint = chrono.ChCoordsysD(chrono.ChVectorD(0.7, 0, 0.05), chrono.ChQuaternionD(1, 0, 0, 0))
csys_piston_prismatic = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
prismatic.Initialize(floor, piston, csys_floor_joint, csys_piston_prismatic)
my_system.AddLink(prismatic)


application = irr.ChIrrApp(my_system, 'Crank Slider Mechanism', irr.dimension2du(800, 600))
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono.png'))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
application.SetCameraVertical(irr.ECameraVertical.ECV_Z_AXIS)
application.SetTimestep(0.01)
application.SetTryRealtime(True)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()