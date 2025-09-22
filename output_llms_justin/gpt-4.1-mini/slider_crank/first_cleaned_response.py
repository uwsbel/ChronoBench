import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  
system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor_shape = chrono.ChBoxShape()
floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.6, 0.1, 0.3)  
floor.GetVisualShapeList().push_back(floor_shape)
floor.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(floor)


crank_radius = 0.2
crank_width = 0.05

connecting_rod_length = 0.6
connecting_rod_width = 0.04
connecting_rod_height = 0.04

piston_radius = 0.08
piston_height = 0.1


crank_axis_pos = chrono.ChVectorD(0, 0.15, 0)


crank = chrono.ChBody()
crank.SetPos(crank_axis_pos)
crank.SetMass(2.0)
crank.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
crank_shape = chrono.ChCylinderShape()
crank_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -crank_width / 2)
crank_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, crank_width / 2)
crank_shape.GetCylinderGeometry().rad = crank_radius * 0.2
crank.GetVisualShapeList().push_back(crank_shape)

crank_arm = chrono.ChBoxShape()
crank_arm.GetBoxGeometry().Size = chrono.ChVectorD(crank_radius, 0.02, crank_width / 2)
crank_arm.SetColor(chrono.ChColor(0.8, 0.1, 0.1))
crank_arm_pos = chrono.ChCoordsysD(chrono.ChVectorD(crank_radius/2, 0, 0))
crank_arm.SetPose(crank_arm_pos)
crank.GetVisualShapeList().push_back(crank_arm)
system.Add(crank)


conn_rod = chrono.ChBody()

conn_rod.SetPos(crank_axis_pos + chrono.ChVectorD(crank_radius, 0, 0))
conn_rod.SetMass(1.5)
conn_rod.SetInertiaXX(chrono.ChVectorD(0.02, 0.01, 0.01))
conn_rod_shape = chrono.ChBoxShape()
conn_rod_shape.GetBoxGeometry().Size = chrono.ChVectorD(connecting_rod_length * 0.5, connecting_rod_height * 0.5, connecting_rod_width * 0.5)
conn_rod_shape.SetColor(chrono.ChColor(0.1, 0.7, 0.1))
conn_rod.GetVisualShapeList().push_back(conn_rod_shape)
system.Add(conn_rod)


piston = chrono.ChBody()
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))

piston_start_y = crank_axis_pos.y - connecting_rod_length - piston_height*0.5 + crank_radius
piston.SetPos(chrono.ChVectorD(crank_axis_pos.x + crank_radius, piston_start_y, crank_axis_pos.z))
piston_shape = chrono.ChCylinderShape()
piston_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -piston_radius)
piston_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, piston_height, -piston_radius)
piston_shape.GetCylinderGeometry().rad = piston_radius
piston_shape.SetColor(chrono.ChColor(0.1, 0.1, 0.7))
piston.GetVisualShapeList().push_back(piston_shape)

piston_box = chrono.ChBoxShape()
piston_box.GetBoxGeometry().Size = chrono.ChVectorD(piston_radius, piston_height * 0.5, piston_radius*0.8)
piston_box.SetColor(chrono.ChColor(0.1, 0.1, 0.7))
piston_box_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, piston_height*0.5, 0))
piston_box.SetPose(piston_box_pos)
piston.GetVisualShapeList().push_back(piston_box)
system.Add(piston)




joint_crank = chrono.ChLinkLockRevolute()
joint_crank.Initialize(crank, floor, chrono.ChFrameD(crank_axis_pos, chrono.Q_from_AxisAngle(chrono.ChVectorD(0, 0, 1), 0)))
system.Add(joint_crank)




pin_pos = crank_axis_pos + chrono.ChVectorD(crank_radius, 0, 0)
joint_crank_conn = chrono.ChLinkLockRevolute()
joint_crank_conn.Initialize(conn_rod, crank, chrono.ChFrameD(pin_pos))
system.Add(joint_crank_conn)


rod_piston_pos = piston.GetPos() + chrono.ChVectorD(0, piston_height*0.25, 0)
joint_conn_pist = chrono.ChLinkLockRevolute()
joint_conn_pist.Initialize(piston, conn_rod, chrono.ChFrameD(rod_piston_pos))
system.Add(joint_conn_pist)


prismatic = chrono.ChLinkLockPrismatic()
prismatic_axis = chrono.ChVectorD(0, 1, 0)  
prismatic.Initialize(piston, floor, chrono.ChFrameD(piston.GetPos(), chrono.Q_from_AngAxis(0, prismatic_axis)))
system.Add(prismatic)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(crank, floor, chrono.ChFrameD(crank_axis_pos, chrono.Q_from_AxisAngle(chrono.ChVectorD(0, 0, 1), 0)))


motor_speed = 2.0  


motor_function = chrono.ChFunction_Const(motor_speed)

motor.SetMotorFunction(motor_function)
system.Add(motor)



application = chronoirr.ChIrrApp(system, "Crank-Slider Demo", chronoirr.dimension2du(1024,768))


application.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))


application.AddSkyBox()
application.AddTypicalLights()


cam_pos = chrono.ChVectorD(1.0, 0.6, 1.0)
cam_look_at = chrono.ChVectorD(0, 0.2, 0)
application.SetCameraPosition(cam_pos, cam_look_at)


application.AssetBindAll()
application.AssetUpdateAll()


time_step = 0.01


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()