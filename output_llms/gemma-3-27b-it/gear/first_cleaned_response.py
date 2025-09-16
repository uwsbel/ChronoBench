import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




body_truss = chrono.ChBodyEasy()
body_truss.SetBodyFixed(True)
body_truss.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(body_truss)


truss_shape = chrono.ChBoxShape()
truss_shape.GetBoxGeometry().SetMinMax(chrono.ChVectorD(-0.5, -0.5, -0.5), chrono.ChVectorD(0.5, 0.5, 0.5))
body_truss.AddAsset(truss_shape)
body_truss.SetCollide(True)
body_truss.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())




body_bar = chrono.ChBodyEasy()
body_bar.SetPos(chrono.ChVectorD(1, 0, 0))
body_bar.SetBodyFixed(False)
system.Add(body_bar)


bar_shape = chrono.ChCylinderShape()
bar_shape.GetCylinderGeometry().SetRadius(0.1)
bar_shape.GetCylinderGeometry().SetHeight(1)
body_bar.AddAsset(bar_shape)
body_bar.SetCollide(True)
body_bar.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())






body_gear1 = chrono.ChBodyEasy()
body_gear1.SetPos(chrono.ChVectorD(2, 0, 0))
system.Add(body_gear1)

gear1_shape = chrono.ChGearShape()
gear1_shape.GetGearGeometry().SetRadius(0.3)
gear1_shape.GetGearGeometry().SetHeight(0.2)
gear1_shape.GetGearGeometry().SetNumTeeth(20)
body_gear1.AddAsset(gear1_shape)
body_gear1.SetCollide(True)
body_gear1.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())


body_gear2 = chrono.ChBodyEasy()
body_gear2.SetPos(chrono.ChVectorD(3, 0, 0))
system.Add(body_gear2)

gear2_shape = chrono.ChGearShape()
gear2_shape.GetGearGeometry().SetRadius(0.3)
gear2_shape.GetGearGeometry().SetHeight(0.2)
gear2_shape.GetGearGeometry().SetNumTeeth(40)
body_gear2.AddAsset(gear2_shape)
body_gear2.SetCollide(True)
body_gear2.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())




gear_pair = chrono.ChLinkGear()
gear_pair.Initialize(body_gear1, body_gear2, chrono.ChCoordsys(chrono.ChVectorD(0,0,0)))
system.Add(gear_pair)




motor = chrono.ChMotorLinearSpring()
motor.Set_Rest_Spring_K(1000)
motor.Set_Rest_Spring_R(10)
motor.Set_Motor_Speed(1) 
motor.Set_Motor_Torque(1)
motor.Initialize(body_gear1, body_truss, chrono.ChCoordsys(chrono.ChVectorD(0,0,0)))
system.Add(motor)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddTypicalCamera(chrono.ChVectorD(2, 1.5, -2))
vis.AddTypicalLights()




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)