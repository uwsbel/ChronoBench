import pychrono as chrono
import pychrono.irrlicht as chronoirr







mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_time_step = 0.01







vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Epicyclic Gears")
vis.Initialize()
vis.AddCameraLight()
vis.AddSkyBox()







ground = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
mysystem.AddBody(ground)







truss_material = chrono.ChMaterialSurfaceNSC()
truss_material.SetFriction(0.3)
truss_material.SetRestitution(0.2)

truss = chrono.ChBody()
truss.SetBodyFixed(True)
mysystem.AddBody(truss)










bar = chrono.ChBodyEasyBox(1, 0.1, 0.1, 100, True, True)
bar.SetPos(chrono.ChVectorD(0, 0.5, 0))
bar.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))
mysystem.AddBody(bar)


bar_joint = chrono.ChLinkRevolute()
bar_joint.Initialize(truss, bar, chrono.ChVectorD(0, 0.5, 0))
mysystem.AddLink(bar_joint)








gear1_radius = 0.5
gear2_radius = 0.25


gear1 = chrono.ChBodyEasyCylinder(gear1_radius, 0.1, 100, True, True)
gear1.SetPos(chrono.ChVectorD(gear1_radius, 0.5, 0))
mysystem.AddBody(gear1)


gear2 = chrono.ChBodyEasyCylinder(gear2_radius, 0.1, 100, True, True)
gear2.SetPos(chrono.ChVectorD(gear1_radius + gear2_radius, 0.5, 0))
mysystem.AddBody(gear2)


gear1_joint = chrono.ChLinkRevolute()
gear1_joint.Initialize(bar, gear1, chrono.ChVectorD(gear1_radius, 0, 0))
mysystem.AddLink(gear1_joint)

gear2_joint = chrono.ChLinkRevolute()
gear2_joint.Initialize(gear1, gear2, chrono.ChVectorD(gear1_radius + gear2_radius, 0, 0))
mysystem.AddLink(gear2_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, chrono.ChVectorD(0, 0, 0))
motor.SetSpeedFunction(chrono.ChFunction_Const(10)) 
mysystem.AddLink(motor)







ground.AddAsset(chrono.ChAssetShape(chrono.ChSphereShape(0.5)))
ground.AddAsset(chrono.ChAssetMaterial(chrono.ChColorAsset(0.5, 0.5, 0.5)))

bar.AddAsset(chrono.ChAssetShape(chrono.ChBoxShape(0.5, 0.05, 0.05)))
bar.AddAsset(chrono.ChAssetMaterial(chrono.ChColorAsset(1, 0, 0)))

gear1.AddAsset(chrono.ChAssetShape(chrono.ChCylinderShape(gear1_radius, 0.1)))
gear1.AddAsset(chrono.ChAssetMaterial(chrono.ChColorAsset(0, 1, 0)))

gear2.AddAsset(chrono.ChAssetShape(chrono.ChCylinderShape(gear2_radius, 0.1)))
gear2.AddAsset(chrono.ChAssetMaterial(chrono.ChColorAsset(0, 0, 1)))







while vis.Run():
    mysystem.DoStepDynamics(my_time_step)
    vis.Render()