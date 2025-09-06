import chrono
import chrono.robot
import chrono.vehicle
import chrono.irrlicht
import chrono.fea
import chrono.robot
import numpy as np


chrono.Initialize()


vis = chrono.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()


sys = chrono.ChSystemSMC()
sys.SetGravity(chrono.ChVector3d(0, 0, -9.81))


truss_frame = chrono.ChFrame()
truss_frame.SetOrigin(chrono.ChVector3d(0, 0, 0))
truss_frame.SetRotation(chrono.ChQuaternion(chrono.ChVector3d(0, 0, 0)))
sys.AddFrame(truss_frame)


bar_frame = chrono.ChFrame()
bar_frame.SetOrigin(chrono.ChVector3d(0, 1, 0))
bar_frame.SetRotation(chrono.ChQuaternion(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0)))
bar_frame.Rotate(chrono.ChRotationd(0.0, 0.0, 0.0, 1.0))
bar_body = chrono.ChBody()
bar_body.SetPos(bar_frame.GetOrigin())
bar_body.SetFrame(bar_frame)
bar_body.SetMass(10)
bar_body.SetInertiaXX(100)
bar_body.SetFixed(False)
bar_body.EnableCollision(True)
sys.Add(bar_body)


gear1_frame = chrono.ChFrame()
gear1_frame.SetOrigin(chrono.ChVector3d(0, 2, 0))
gear1_frame.SetRotation(chrono.ChQuaternion(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0)))
gear1_body = chrono.ChBody()
gear1_body.SetPos(gear1_frame.GetOrigin())
gear1_body.SetFrame(gear1_frame)
gear1_body.SetMass(5)
gear1_body.SetInertiaXX(10)
gear1_body.SetFixed(False)
gear1_body.EnableCollision(True)

gear2_frame = chrono.ChFrame()
gear2_frame.SetOrigin(chrono.ChVector3d(0, 4, 0))
gear2_frame.SetRotation(chrono.ChQuaternion(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, -1, 0)))
gear2_body = chrono.ChBody()
gear2_body.SetPos(gear2_frame.GetOrigin())
gear2_body.SetFrame(gear2_frame)
gear2_body.SetMass(5)
gear2_body.SetInertiaXX(10)
gear2_body.SetFixed(False)
gear2_body.EnableCollision(True)


gear1_tooth_mesh = chrono.ChMesh()
gear1_tooth_mesh.AddTriangle(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0.1, 0, 0), chrono.ChVector3d(0.1, 0.1, 0))
gear1_tooth_mesh.AddTriangle(chrono.ChVector3d(0.1, 0, 0), chrono.ChVector3d(0.1, 0.1, 0))
gear1_tooth_mesh.AddTriangle(chrono.ChVector3d(0, 0.1, 0), chrono.ChVector3d(0, 0.1, 0.1))
gear1_tooth_mesh.AddTriangle(chrono.ChVector3d(0.1, 0.1, 0), chrono.ChVector3d(0, 0.1, 0.1))
gear1_tooth_mesh.AddTriangle(chrono.ChVector3d(0, 0.1, 0.1), chrono.ChVector3d(0.1, 0.1, 0))

gear2_tooth_mesh = chrono.ChMesh()
gear2_tooth_mesh.AddTriangle(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0.1, 0, 0), chrono.ChVector3d(0.1, 0.1, 0))
gear2_tooth_mesh.AddTriangle(chrono.ChVector3d(0.1, 0, 0), chrono.ChVector3d(0.1, 0.1, 0))
gear2_tooth_mesh.AddTriangle(chrono.ChVector3d(0, 0.1, 0), chrono.ChVector3d(0, 0.1, 0.1))
gear2_tooth_mesh.AddTriangle(chrono.ChVector3d(0, 0.1, 0.1), chrono.ChVector3d(0, 0.1, 0.1))


gear1 = chrono.ChBody()
gear1.SetPos(gear1_frame.GetOrigin())
gear1.SetMass(5)
gear1.SetInertiaXX(10)
gear1.SetFixed(False)
gear1.EnableCollision(True)
gear1.AddVisualShape(gear1_tooth_mesh)
sys.Add(gear1)

gear2 = chrono.ChBody()
gear2.SetPos(gear2_frame.GetOrigin())
gear2.SetMass(5)
gear2.SetInertiaXX(10)
gear2.SetFixed(False)
gear2.EnableCollision(True)
gear2.AddVisualShape(gear2_tooth_mesh)


shaft_frame = chrono.ChFrame()
shaft_frame.SetOrigin(chrono.ChVector3d(0, 3, 0))
shaft_frame.SetRotation(chrono.ChQuaternion(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1)))
shaft_body = chrono.ChBody()
shaft_body.SetPos(shaft_frame.GetOrigin())
shaft_body.SetFrame(shaft_frame)
shaft_body.SetMass(2)
shaft_body.SetInertiaXX(2)
shaft_body.SetFixed(False)
shaft_body.EnableCollision(True)

shaft_link = chrono.ChLinkLockRotation()
shaft_link.Initialize(gear1, shaft_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternion(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1)))
sys.Add(shaft_link)

gear2_link = chrono.ChLinkLockRotation()
gear2_link.Initialize(shaft_body, gear2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternion(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1)))
sys.Add(gear2_link)


gear_motor = chrono.ChLinkMotorRotationSpeed()
gear_motor.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternion(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1)))
gear_motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  
sys.Add(gear_motor)


gear1_visual_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.1, 0.1, 0.1))
gear2_visual_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.1, 0.1, 0.1))


gear1_material = chrono.ChContactMaterialSMC()
gear2_material = chrono.ChContactMaterialSMC()
shaft_material = chrono.ChContactMaterialSMC()


gear1_visual_shape.SetColor(chrono.ChColor(1, 0, 0))  
gear2_visual_shape.SetColor(chrono.ChColor(0, 1, 0))  
shaft_visual_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(0.05, 0.05, 0.05))
shaft_visual_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))


gear1.AddVisualShape(gear1_visual_shape)
gear2.AddVisualShape(gear2_visual_shape)
shaft_body.AddVisualShape(shaft_visual_shape)


table_frame = chrono.ChFrame()
table_frame.SetOrigin(chrono.ChVector3d(0, -5, 0))
table_body = chrono.ChBody()
table_body.SetPos(table_frame.GetOrigin())
table_body.SetMass(1000)
table_body.SetInertiaXX(1000)
table_body.SetFixed(False)
table_body.EnableCollision(True)
table_shape = chrono.ChVisualShapeBox(10, 1, 0.5)
table_body.AddVisualShape(table_shape)


vis.AttachSystem(sys)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()