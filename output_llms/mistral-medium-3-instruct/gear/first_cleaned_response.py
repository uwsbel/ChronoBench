import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Epicyclic Gear System')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 3, 2), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)


truss_mat = chrono.ChVisualMaterial()
truss_mat.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))

gear_mat = chrono.ChVisualMaterial()
gear_mat.SetKdTexture(chrono.GetChronoDataFile("textures/steel.jpg"))

bar_mat = chrono.ChVisualMaterial()
bar_mat.SetKdColor(chrono.ChColor(0.2, 0.3, 0.4))


truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetCollide(False)
system.Add(truss)


truss_geom = chrono.ChBoxShape()
truss_geom.GetBoxGeometry().SetLengths(chrono.ChVectorD(2, 0.2, 0.2))
truss_geom.SetPos(chrono.ChVectorD(0, 0, 0))
truss.AddVisualShape(truss_geom, truss_mat)


bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0, 0.2))
system.Add(bar)


bar_geom = chrono.ChCylinderShape()
bar_geom.GetCylinderGeometry().rad = 0.05
bar_geom.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
bar_geom.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0.5)
bar.AddVisualShape(bar_geom, bar_mat)


sun_gear = chrono.ChBody()
sun_gear.SetPos(chrono.ChVectorD(0, 0, 0.7))
system.Add(sun_gear)


sun_gear_geom = chrono.ChCylinderShape()
sun_gear_geom.GetCylinderGeometry().rad = 0.1
sun_gear_geom.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.05)
sun_gear_geom.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0.05)
sun_gear.AddVisualShape(sun_gear_geom, gear_mat)


planet_gear = chrono.ChBody()
planet_gear.SetPos(chrono.ChVectorD(0.3, 0, 0.7))
system.Add(planet_gear)


planet_gear_geom = chrono.ChCylinderShape()
planet_gear_geom.GetCylinderGeometry().rad = 0.05
planet_gear_geom.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.05)
planet_gear_geom.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0.05)
planet_gear.AddVisualShape(planet_gear_geom, gear_mat)



link_bar_sun = chrono.ChLinkLockRevolute()
link_bar_sun.Initialize(bar, sun_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.7)))
system.Add(link_bar_sun)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(sun_gear, planet_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.7)),
                           chrono.ChCoordsysD(chrono.ChVectorD(0.3, 0, 0.7)), 0.1, 0.05, 0.05)
system.Add(gear_constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetMotorFunction(chrono.ChFunction_Const(chrono.ChFunction_Const(1.0)))  
system.Add(motor)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)