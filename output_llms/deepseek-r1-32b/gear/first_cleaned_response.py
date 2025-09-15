importchrono
import math


system =chrono.ChSystem()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


truss_body1 =chrono.ChBody()
truss_body1.SetPos(chrono.ChVectorD(-1, 0, 0))
truss_body1.SetMass(1)
truss_body1.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
system.Add(truss_body1)

truss_body2 =chrono.ChBody()
truss_body2.SetPos(chrono.ChVectorD(1, 0, 0))
truss_body2.SetMass(1)
truss_body2.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
system.Add(truss_body2)


truss_link =chrono.ChLinkFixed()
truss_link.Initialize(truss_body1, truss_body2, chrono.ChFrameD(chrono.ChVectorD(2, 0, 0)))
system.Add(truss_link)


rotating_bar =chrono.ChBody()
rotating_bar.SetPos(chrono.ChVectorD(0, 0, 0))
rotating_bar.SetMass(1)
rotating_bar.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
system.Add(rotating_bar)


bar_joint =chrono.ChLinkRev()
bar_joint.Initialize(truss_body1, rotating_bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(bar_joint)


fixed_gear =chrono.ChBody()
fixed_gear.SetPos(chrono.ChVectorD(0, 0, 0))
fixed_gear.SetMass(2)
fixed_gear.SetInertiaXX(chrono.ChVectorD(2, 2, 2))
fixed_gear.SetMaterialSurface(chrono.ChVisualMaterial(chrono.ChColor(0.2, 0.8, 0.2)))  
system.Add(fixed_gear)


rotating_gear =chrono.ChBody()
rotating_gear.SetPos(chrono.ChVectorD(0, 1, 0))
rotating_gear.SetMass(2)
rotating_gear.SetInertiaXX(chrono.ChVectorD(2, 2, 2))
rotating_gear.SetMaterialSurface(chrono.ChVisualMaterial(chrono.ChColor(0.8, 0.2, 0.2)))  
system.Add(rotating_gear)


gear_joint =chrono.ChLinkRev()
gear_joint.Initialize(rotating_bar, rotating_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(gear_joint)


gear_motor =chrono.ChLinkMotorRotation()
gear_motor.Initialize(fixed_gear, rotating_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
gear_motor.SetMotorType(chrono.ChLinkMotorRotation.MOTOR_TYPE_SPEED)
gear_motor.SetSpeed(chrono.CH_C_2PI * 10)  
system.Add(gear_motor)


renderer =chrono.ChIrrRenderer()
renderer.AttachSystem(system)
renderer.SetCameraPosition(chrono.ChVectorD(3, 3, 3))
renderer.SetCameraRotation(chrono.ChQuaternionD(1, 0, 0, 0))
renderer.AddPointLight(chrono.ChVectorD(2, 2, 2), chrono.ChColor(1, 1, 1))


fixed_gear_shape =chrono.ChCylinder()
fixed_gear_shape.SetRadius(0.5)
fixed_gear_shape.SetHeight(0.2)
fixed_gear.AddVisualShape(fixed_gear_shape)

rotating_gear_shape =chrono.ChCylinder()
rotating_gear_shape.SetRadius(0.3)
rotating_gear_shape.SetHeight(0.2)
rotating_gear.AddVisualShape(rotating_gear_shape)


renderer.Render()
chrono.WaitKey()