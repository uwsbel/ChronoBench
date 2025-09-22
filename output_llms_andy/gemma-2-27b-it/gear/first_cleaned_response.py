import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math







mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.2)







truss = chrono.ChBody()
truss.SetBodyFixed(True)


truss_shape = chrono.ChBoxShape()
truss_shape.SetSize(chrono.ChVectorD(1, 0.1, 0.1))
truss.AddAsset(truss_shape)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
mysystem.Add(truss)







bar = chrono.ChBody()
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(0.05, 0.1, 0.1))


bar_shape = chrono.ChCylinderShape()
bar_shape.SetRadius(0.05)
bar_shape.SetHeight(0.5)
bar.AddAsset(bar_shape)
bar.SetPos(chrono.ChVectorD(0.5, 0, 0))
mysystem.Add(bar)







gear1 = chrono.ChBody()
gear1.SetMass(2)
gear1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


gear1_shape = chrono.ChCylinderShape()
gear1_shape.SetRadius(0.1)
gear1_shape.SetHeight(0.05)
gear1.AddAsset(gear1_shape)
gear1.SetPos(chrono.ChVectorD(0.8, 0, 0))
mysystem.Add(gear1)

gear2 = chrono.ChBody()
gear2.SetMass(2)
gear2.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


gear2_shape = chrono.ChCylinderShape()
gear2_shape.SetRadius(0.08)
gear2_shape.SetHeight(0.05)
gear2.AddAsset(gear2_shape)
gear2.SetPos(chrono.ChVectorD(1.1, 0, 0))
mysystem.Add(gear2)








joint1 = chrono.ChLinkRevolute()
joint1.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0)))
mysystem.Add(joint1)


joint2 = chrono.ChLinkRevolute()
joint2.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVectorD(0.8, 0, 0)))
mysystem.Add(joint2)







motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, chrono.ChFrameD(chrono.ChVectorD(0.8, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(10))
mysystem.Add(motor)







myapplication = chronoirr.ChIrrApp(mysystem, "Epicyclic Gears", chronoirr.dimension2du(800, 600))

myapplication.AddTypicalSky()
myapplication.AddTypicalCamera(chronoirr.vector3df(2, 2, 2))
myapplication.SetSymbolSize(0.05)

myapplication.AssetBindAll();
myapplication.AssetUpdateAll();







while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()