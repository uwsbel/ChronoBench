importchrono
importchrono irrlicht as chvis
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


renderer = chvis.ChIrrRenderer()
renderer.AttachSystem(system)
renderer.SetWindowTitle("Epicyclic Gear System")
renderer.SetCameraPosition(chrono.ChVectorD(2, 2, 2))
renderer.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
renderer.SetLightPosition(chrono.ChVectorD(0, 1, 1))
renderer.SetLightIntensity(1)


truss = chrono.ChBodyFixed()
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetMaterialSurface(chrono.ChMaterialSurfaceRGBA(0.8, 0.8, 0.8, 1))
system.Add(truss)


bar = chrono.ChBody()
bar.SetMass(1)
bar.SetPos(chrono.ChVectorD(0, 1, 0))
bar.SetMaterialSurface(chrono.ChMaterialSurfaceRGBA(0.5, 0.5, 0.5, 1))
system.Add(bar)


rev_bar = chrono.ChLinkRev()
rev_bar.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D()))
system.Add(rev_bar)


gear1 = chrono.ChBody()
gear1.SetMass(2)
gear1.SetPos(chrono.ChVectorD(1, 0, 0))
gear1.SetMaterialSurface(chrono.ChMaterialSurfaceRGBA(0.8, 0.3, 0.3, 1))
system.Add(gear1)


rev_gear1 = chrono.ChLinkRev()
rev_gear1.Initialize(truss, gear1, chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0), chrono.ChMatrix33D()))
system.Add(rev_gear1)


gear2 = chrono.ChBody()
gear2.SetMass(2)
gear2.SetPos(chrono.ChVectorD(-1, 0, 0))
gear2.SetMaterialSurface(chrono.ChMaterialSurfaceRGBA(0.3, 0.8, 0.3, 1))
system.Add(gear2)


rev_gear2 = chrono.ChLinkRev()
rev_gear2.Initialize(truss, gear2, chrono.ChCoordsysD(chrono.ChVectorD(-1, 0, 0), chrono.ChMatrix33D()))
system.Add(rev_gear2)


gear_ratio = 1.0  
radius_bar = 0.5
radius_gear1 = 0.5
radius_gear2 = 0.5


gearConstraint1 = chrono.ChLinkLockRevolute()
gearConstraint1.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.ChMatrix33D()))
gearConstraint1.SetGearRatio(gear_ratio * (radius_bar / radius_gear1))
system.Add(gearConstraint1)


gearConstraint2 = chrono.ChLinkLockRevolute()
gearConstraint2.Initialize(bar, gear2, chrono.ChCoordsysD(chrono.ChVectorD(0, -1, 0), chrono.ChMatrix33D()))
gearConstraint2.SetGearRatio(gear_ratio * (radius_bar / radius_gear2))
system.Add(gearConstraint2)


motor = chrono.ChMotorRotationSpeed()
motor.SetMotorType(chrono.ChMotorRotationSpeed.MOTOR_TYPE_RPM)
motor.SetSpeed(10)  
rev_gear1.AddMotor(motor)


ground = chrono.ChBodyFixed()
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetMaterialSurface(chrono.ChMaterialSurfaceRGBA(0.4, 0.4, 0.4, 1))
system.Add(ground)


bar.AddCollisionShape(chrono.ChCollisionBox(chrono.ChVectorD(0.1, 0.5, 0.1)))
gear1.AddCollisionShape(chrono.ChCollisionCylinder(0.5, 0.1))
gear2.AddCollisionShape(chrono.ChCollisionCylinder(0.5, 0.1))


bar.AddAsset(chrono.ChVisualShapeBox(chrono.ChVectorD(0.1, 0.5, 0.1), chrono.ChColor(0.5, 0.5, 0.5)))
gear1.AddAsset(chrono.ChVisualShapeCylinder(0.5, 0.1, chrono.ChColor(0.8, 0.3, 0.3)))
gear2.AddAsset(chrono.ChVisualShapeCylinder(0.5, 0.1, chrono.ChColor(0.3, 0.8, 0.3)))


renderer.Render()
for _ in range(1000):
    system.DoStepDynamics(0.001)
    renderer.Render()