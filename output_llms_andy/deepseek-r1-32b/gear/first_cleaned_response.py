importchrono
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = chrono.ChIrrApp(system, "Epicyclic Gear System", chronoirr.dimension2D(800, 600))
application.AddTypicalIrrlichtLogo()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 1, 3), chrono.ChVectorD(0, 0, 0))


truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetMass(0)
truss.GetVisualShape().SetMaterialPtr(chrono.ChVisualMaterial())
truss.GetVisualShape().SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
truss.GetVisualShape().SetScale(chrono.ChVectorD(1, 1, 1))
truss.GetVisualShape().SetGeometry(chrono.ChCylinder(0.1, 0.5))
system.Add(truss)


bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0.25, 0))
bar.SetMass(1)
bar.GetVisualShape().SetMaterialPtr(chrono.ChVisualMaterial())
bar.GetVisualShape().SetTexture(chrono.GetChronoDataFile("textures/cyan.png"))
bar.GetVisualShape().SetGeometry(chrono.ChBox(0.1, 0.25, 0.1))
system.Add(bar)


joint_bar = chrono.ChLinkRev()
joint_bar.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
joint_bar.SetRotationLimit(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), 0, 2 * math.pi))
system.Add(joint_bar)


gear_fixed = chrono.ChBody()
gear_fixed.SetBodyFixed(True)
gear_fixed.SetPos(chrono.ChVectorD(0, 0, 0))
gear_fixed.SetMass(0)
gear_fixed.GetVisualShape().SetMaterialPtr(chrono.ChVisualMaterial())
gear_fixed.GetVisualShape().SetTexture(chrono.GetChronoDataFile("textures/yellow.png"))
gear_fixed.GetVisualShape().SetGeometry(chrono.ChCylinder(0.2, 0.1))
system.Add(gear_fixed)


gear_rotating = chrono.ChBody()
gear_rotating.SetPos(chrono.ChVectorD(0.3, 0.25, 0))
gear_rotating.SetMass(1)
gear_rotating.GetVisualShape().SetMaterialPtr(chrono.ChVisualMaterial())
gear_rotating.GetVisualShape().SetTexture(chrono.GetChronoDataFile("textures/yellow.png"))
gear_rotating.GetVisualShape().SetGeometry(chrono.ChCylinder(0.15, 0.1))
system.Add(gear_rotating)


joint_fixed_gear = chrono.ChLinkRev()
joint_fixed_gear.Initialize(truss, gear_fixed, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
joint_fixed_gear.SetRotationLimit(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), 0, 2 * math.pi))
system.Add(joint_fixed_gear)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear_fixed, gear_rotating, 1, 1)
system.Add(gear_constraint)


motor = chrono.ChMotorRotation()
motor.SetName("Motor")
motor.SetType(chrono.ChMotorRotation.MOT_TYPE_RPM)
motor.SetRPM(20)  
joint_bar.AddMotor(motor)


system.Add(truss)
system.Add(bar)
system.Add(gear_fixed)
system.Add(gear_rotating)
system.Add(joint_bar)
system.Add(joint_fixed_gear)
system.Add(gear_constraint)


application.AssetSetLight(chronoirr.VEMLightType.VLT_DIRECTIONAL, 0, chrono.ChVectorD(0.5, 0.5, 0.5))
application.AssetSetLight(chronoirr.VEMLightType.VLT_AMBIENT, 0, chrono.ChVectorD(0.3, 0.3, 0.3))
application.AssetSetLight(chronoirr.VEMLightType.VLT_POINT, 0, chrono.ChVectorD(0.5, 0.5, 0.5))
application.AssetUpdateAllLight()
application.AssetUpdateAllShadow()

application.SetTimestep(0.001)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

application.GetDevice().closeDevice()
chrono.ChDeleteAll()