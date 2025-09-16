import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Epicyclic Gear System")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.5, 2))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0), 10, chrono.ChColor(1, 1, 1))


mat_ground = chrono.ChVisualMaterial()
mat_ground.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
mat_gear = chrono.ChVisualMaterial()
mat_gear.SetKdColor(chrono.ChColor(0.6, 0.2, 0.2))
mat_bar = chrono.ChVisualMaterial()
mat_bar.SetKdColor(chrono.ChColor(0.2, 0.6, 0.2))


truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
truss.GetVisualModel().AddBox(1, 0.1, 1, chrono.ChVectorD(0, -0.05, 0), mat_ground)
system.AddBody(truss)


bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0.5, 0))
bar.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
bar.GetVisualModel().AddCylinder(0.05, 0.5, chrono.ChVectorD(0, 0, 0), mat_bar)
bar.GetVisualModel().AddBox(0.2, 0.05, 0.05, chrono.ChVectorD(0.25, 0, 0), mat_bar)
system.AddBody(bar)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  
system.AddLink(motor)


sun_gear = chrono.ChBody()
sun_gear.SetPos(chrono.ChVectorD(0, 0.5, 0))
sun_gear.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sun_gear.GetVisualModel().AddCylinder(0.05, 0.1, chrono.ChVectorD(0, 0, 0), mat_gear)
sun_gear.GetVisualModel().AddBox(0.1, 0.02, 0.1, chrono.ChVectorD(0, 0, 0), mat_gear)
system.AddBody(sun_gear)


planet_gear = chrono.ChBody()
planet_gear.SetPos(chrono.ChVectorD(0.25, 0.5, 0))
planet_gear.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
planet_gear.GetVisualModel().AddCylinder(0.03, 0.05, chrono.ChVectorD(0, 0, 0), mat_gear)
planet_gear.GetVisualModel().AddBox(0.05, 0.01, 0.05, chrono.ChVectorD(0, 0, 0), mat_gear)
system.AddBody(planet_gear)


gear_mesh = chrono.ChLinkGear()
gear_mesh.Initialize(sun_gear, planet_gear, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
gear_mesh.SetGearRatio(2.0)  
system.AddLink(gear_mesh)


revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(planet_gear, bar, chrono.ChFrameD(chrono.ChVectorD(0.25, 0.5, 0)))
system.AddLink(revolute)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)