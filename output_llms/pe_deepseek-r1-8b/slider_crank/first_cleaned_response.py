sys = chrono.ChSystemNSC()


chrono.SetChronoDataPath('/path/to/chrono/data')


vis = chronoirr.ChVisualSystemIrrlicht()

vis.AttachSystem(sys)

vis.SetWindowSize(1024, 768)

vis.SetWindowTitle('Crank-Slider Mechanism Simulation')

vis.AddSkyBox()

cam_pos = chrono.ChVector3d(0, 3, 6)
vis.AddCamera(cam_pos)

vis.AddTypicalLights()

vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


floor_body = chrono.ChBodyEasyBox(5, 0.5, 0.5, 1000, True, True, chrono.ChContactMaterialSMC())
floor_body.SetPos(chrono.ChVector3d(0, 0, 0))
floor_body.SetFixed(True)
sys.Add(floor_body)


floor_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(5, 0.5, 0.5))
floor_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))  
floor_body.AddVisualShape(floor_shape)


crankshaft_body = chrono.ChBodyEasyRod(0.5, 0.05, 1000, True, True, chrono.ChContactMaterialSMC())
crankshaft_body.SetPos(chrono.ChVector3d(0, 0, 0))
crankshaft_body.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  
crankshaft_body.EnableCollision(True)
sys.Add(crankshaft_body)


crankshaft_shape = chrono.ChVisualShapeCylinder(0.025, 0.05, True)
crankshaft_shape.SetColor(chrono.ChColor(200, 200, 200))  
crankshaft_body.AddVisualShape(crankshaft_shape)


connecting_rod_body = chrono.ChBodyEasyRod(0.5, 0.05, 1000, True, True, chrono.ChContactMaterialSMC())

connect_pos = chrono.ChVector3d(crankshaft_body.GetPosition() + chrono.ChVector3d(0.5, 0, 0), 0, 0)
connecting_rod_body.SetPos(connect_pos)
connecting_rod_body.EnableCollision(True)
sys.Add(connecting_rod_body)


connecting_rod_shape = chrono.ChVisualShapeCylinder(0.025, 0.05, True)
connecting_rod_shape.SetColor(chrono.ChColor(150, 150, 150))  
connecting_rod_body.AddVisualShape(connecting_rod_shape)


piston_body = chrono.ChBodyEasySphere(0.2, 1000, True, True, chrono.ChContactMaterialSMC())

piston_pos = connect_pos + chrono.ChVector3d(0, 0, 0.2)
piston_body.SetPos(piston_pos)
piston_body.EnableCollision(True)
sys.Add(piston_body)


piston_shape = chrono.ChVisualShapeSphere(0.2)
piston_shape.SetColor(chrono.ChColor(1, 0, 0))  
piston_body.AddVisualShape(piston_shape)


crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crankshaft_body, floor_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))


crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crankshaft_body, floor_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
crank_motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  

sys.Add(crank_joint)
sys.Add(crank_motor)


piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(piston_body, floor_body, chrono.ChCoordsysD(piston_pos, chrono.QuatFromAngleX(chrono.CH_PI / 2)))
sys.Add(piston_joint)


connecting_rod_body.AddVisualShape(connecting_rod_shape)
piston_body.AddVisualShape(piston_shape)


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


time_step = 0.01


while True:
    
    sys.DoStepDynamics(time_step)
    
    
    if not vis.Run():
        break
    
    
    vis.BeginScene()
    
    
    vis.Render()
    
    
    vis.EndScene()

import os
import math
import numpy as np
import chrono
from chrono import ChSystem, ChBody, ChContactMaterialSMC, ChFunction_Const
from chronoirr import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeCylinder, ChVisualShapeSphere
from pychrono import robot, vehicle


sys = chrono.ChSystemNSC()


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
vis.AddSkyBox()
cam_pos = ChVector3d(0, 3, 6)
vis.AddCamera(cam_pos)
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


floor_body = ChBodyEasyBox(5, 0.5, 0.5, 1000, True, True, ChContactMaterialSMC())
floor_body.SetPos(ChVector3d(0, 0, 0))
floor_body.SetFixed(True)
sys.Add(floor_body)

floor_shape = ChVisualShapeBox(ChVector3d(5, 0.5, 0.5))
floor_shape.SetColor(ChColor(0.3, 0.3, 0.3))  
floor_body.AddVisualShape(floor_shape)


crankshaft_body = ChBodyEasyRod(0.5, 0.05, 1000, True, True, ChContactMaterialSMC())
crankshaft_body.SetPos(ChVector3d(0, 0, 0))
crankshaft_body.SetInertiaXX(ChVector3d(0, 0, 0))  
crankshaft_body.EnableCollision(True)
sys.Add(crankshaft_body)

crankshaft_shape = ChVisualShapeCylinder(0.025, 0.05, True)
crankshaft_shape.SetColor(ChColor(200, 200, 200))  
crankshaft_body.AddVisualShape(crankshaft_shape)


connecting_rod_body = ChBodyEasyRod(0.5, 0.05, 1000, True, True, ChContactMaterialSMC())
connect_pos = ChVector3d(crankshaft_body.GetPosition() + ChVector3d(0.5, 0, 0), 0, 0)
connecting_rod_body.SetPos(connect_pos)
connecting_rod_body.EnableCollision(True)
sys.Add(connecting_rod_body)

connecting_rod_shape = ChVisualShapeCylinder(0.025, 0.05, True)
connecting_rod_shape.SetColor(ChColor(150, 150, 150))  
connecting_rod_body.AddVisualShape(connecting_rod_shape)


piston_body = ChBodyEasySphere(0.2, 1000, True, True, ChContactMaterialSMC())
piston_pos = connect_pos + ChVector3d(0, 0, 0.2)
piston_body.SetPos(piston_pos)
piston_body.EnableCollision(True)
sys.Add(piston_body)

piston_shape = ChVisualShapeSphere(0.2)
piston_shape.SetColor(ChColor(1, 0, 0))  
piston_body.AddVisualShape(piston_shape)


crank_joint = ChLinkLockRevolute()
crank_joint.Initialize(crankshaft_body, floor_body, ChCoordsysD(ChVector3d(0, 0, 0), ChQuatFromAngleX(Ch.PI / 2)))
sys.Add(crank_joint)

crank_motor = ChLinkMotorRotationSpeed()
crank_motor.Initialize(crankshaft_body, floor_body, ChCoordsysD(ChVector3d(0, 0, 0), ChQuatFromAngleX(Ch.PI / 2)))
crank_motor.SetSpeedFunction(ChFunction_Const(1.0))  
sys.Add(crank_motor)

piston_joint = ChLinkLockPrismatic()
piston_joint.Initialize(piston_body, floor_body, ChCoordsysD(piston_pos, ChQuatFromAngleX(Ch.PI / 2)))
sys.Add(piston_joint)


vis.AddVisualShape(crankshaft_shape)
vis.AddVisualShape(connecting_rod_shape)
vis.AddVisualShape(piston_shape)


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


while True:
    sys.DoStepDynamics(0.01)
    if not vis.Run():
        break
    vis.BeginScene()
    vis.Render()
    vis.EndScene()