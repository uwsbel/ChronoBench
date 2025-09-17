import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


floor_mat = chrono.ChMaterialSurfaceNSC()
floor_mat.SetFriction(0.5)

floor_body = chrono.ChBody()
floor_body.SetBodyFixed(True)
floor_body.SetPos(chrono.ChVectorD(0, -1, 0))
floor_body.SetCollide(True)
floor_body.GetCollisionModel().ClearModel()
floor_body.GetCollisionModel().AddBox(floor_mat, 10, 1, 10)
floor_body.GetCollisionModel().BuildModel()
floor_body.SetMass(1)


floor_visual = chrono.ChBoxShape()
floor_visual.GetBoxGeometry().Size = chrono.ChVectorD(10, 1, 10)
floor_body.AddVisualShape(floor_visual)

sys.Add(floor_body)


crank_mat = chrono.ChMaterialSurfaceNSC()
crank_mat.SetFriction(0.5)

crank_body = chrono.ChBody()
crank_body.SetMass(1)
crank_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
crank_body.SetPos(chrono.ChVectorD(0, 0, 0))
crank_body.SetCollide(True)
crank_body.GetCollisionModel().ClearModel()
crank_body.GetCollisionModel().AddCylinder(crank_mat, 0.1, 0.1, 0.5)
crank_body.GetCollisionModel().BuildModel()


crank_visual = chrono.ChCylinderShape()
crank_visual.GetCylinderGeometry().rad = 0.1
crank_visual.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.5, 0, 0)
crank_visual.GetCylinderGeometry().p2 = chrono.ChVectorD(0.5, 0, 0)
crank_body.AddVisualShape(crank_visual)

sys.Add(crank_body)


rod_mat = chrono.ChMaterialSurfaceNSC()
rod_mat.SetFriction(0.5)

rod_body = chrono.ChBody()
rod_body.SetMass(1)
rod_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
rod_body.SetPos(chrono.ChVectorD(1, 0, 0))
rod_body.SetCollide(True)
rod_body.GetCollisionModel().ClearModel()
rod_body.GetCollisionModel().AddBox(rod_mat, 1, 0.1, 0.1)
rod_body.GetCollisionModel().BuildModel()


rod_visual = chrono.ChBoxShape()
rod_visual.GetBoxGeometry().Size = chrono.ChVectorD(1, 0.1, 0.1)
rod_body.AddVisualShape(rod_visual)

sys.Add(rod_body)


piston_mat = chrono.ChMaterialSurfaceNSC()
piston_mat.SetFriction(0.5)

piston_body = chrono.ChBody()
piston_body.SetMass(1)
piston_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
piston_body.SetPos(chrono.ChVectorD(2, 0, 0))
piston_body.SetCollide(True)
piston_body.GetCollisionModel().ClearModel()
piston_body.GetCollisionModel().AddCylinder(piston_mat, 0.2, 0.2, 0.1)
piston_body.GetCollisionModel().BuildModel()


piston_visual = chrono.ChCylinderShape()
piston_visual.GetCylinderGeometry().rad = 0.2
piston_visual.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.1, 0, 0)
piston_visual.GetCylinderGeometry().p2 = chrono.ChVectorD(0.1, 0, 0)
piston_body.AddVisualShape(piston_visual)

sys.Add(piston_body)


crankshaft_joint = chrono.ChLinkRevolute()
crankshaft_joint.Initialize(crank_body, floor_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(crankshaft_joint)

crank_rod_joint = chrono.ChLinkRevolute()
crank_rod_joint.Initialize(rod_body, crank_body, chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(crank_rod_joint)

rod_piston_joint = chrono.ChLinkRevolute()
rod_piston_joint.Initialize(piston_body, rod_body, chrono.ChCoordsysD(chrono.ChVectorD(-1, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.Add(rod_piston_joint)

piston_slider_joint = chrono.ChLinkSlider()
piston_slider_joint.Initialize(piston_body, floor_body, chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.Add(piston_slider_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank_body, floor_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_2PI))
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)