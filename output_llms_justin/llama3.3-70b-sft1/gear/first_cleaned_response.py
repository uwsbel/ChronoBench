import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import pychrono.fea as fea
from OCC.Core import BRepPrimAPI
from OCC.Core import BRepFilletAPI
from OCC.Core import BRepAlgoAPI
from OCC.Core import BRepMesh
from OCC.Core import TopoDS
import math

print ("Example: test")
print ("-----------------------------------------------")
print ("")
print ("Generate a system of epicyclic gears using the PyChrono simulation package and visualize it using the Irrlicht visualization system.")
print ("")
print ("Note that the cascade module, which is used to generate a mesh for the gears, is not included in the standard PyChrono distribution. It can be obtained from https://github.com/ChronoLlama/PyChrono-cascade")
print ("")






sys = chrono.ChSystemNSC()



chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001);
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001);


material = chrono.ChContactMaterialNSC()
material.SetFriction(0.5)


floor = chrono.ChBodyEasyBox(1, 0.2, 1, 1000) 
floor.SetPos(chrono.ChVector3d(0,-0.3,0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetMaterial(0, material)
sys.Add(floor)



gear1 = chrono.ChBodyEasyBox(0.2,0.2,0.2,1000)
gear1.SetPos(chrono.ChVector3d(0,0.4,0))
gear1.SetRot(chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(1,0,0)))
gear1.GetVisualShape(0).SetMaterial(0, material)
sys.Add(gear1)



gear2 = chrono.ChBodyEasyBox(0.1,0.1,0.1,1000)
gear2.SetPos(chrono.ChVector3d(0.6,0.4,0))
gear2.SetRot(chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(1,0,0)))
gear2.GetVisualShape(0).SetMaterial(0, material)
sys.Add(gear2)


motor = chrono.ChLinkMotorRotationAngle()
motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
motor.SetAngleFunction(chrono.ChFunctionRamp(0, math.pi / 4))
motor.Initialize(gear1, gear2, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0))))
sys.AddLink(motor)


truss = chrono.ChBodyEasyBox(0.2,0.1,0.2,1000)
truss.SetPos(chrono.ChVector3d(1.5,0.3,0))
truss.SetRot(chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(1,0,0)))
truss.GetVisualShape(0).SetMaterial(0, material)
sys.Add(truss)


bar = chrono.ChBodyEasyBox(0.1,0.1,0.6,1000)
bar.SetPos(chrono.ChVector3d(1.5,-0.1,0))
bar.SetRot(chrono.QuatFromAngleAxis(-math.pi/2,chrono.ChVector3d(1,0,0)))
bar.GetVisualShape(0).SetMaterial(0, material)
sys.Add(bar)


rev = chrono.ChLinkLockRevolute()
rev.Initialize(bar, truss, chrono.ChFramed(chrono.ChVector3d(1.5, -0.1, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0))))
sys.AddLink(rev)


rev = chrono.ChLinkLockRevolute()
rev.Initialize(floor, bar, chrono.ChFramed(chrono.ChVector3d(1.5, -0.5, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0))))
sys.AddLink(rev)


link = chrono.ChLinkLockPoint()
link.Initialize(gear1, truss, chrono.ChFramed(chrono.ChVector3d(0.2, 0.4, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 0))))
sys.AddLink(link)


link = chrono.ChLinkLockPoint()
link.Initialize(gear2, bar, chrono.ChFramed(chrono.ChVector3d(0.6, 0.4, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 0))))
sys.AddLink(link)



mesh = cascade.ChCascadeTriangulator()
gear1_shape = BRepPrimAPI.BRepPrimAPI_MakeCylinder(0.02, 0.01).Shape()
mesh.SetShape(gear1_shape)
mesh.SetMaxSize(0.01)
mesh.GenerateMesh()
trimesh1 = cascade.ChCascadeTriangulator.GetChronoMesh(mesh)
gear1.GetVisualShape(0).SetMesh(trimesh1)
gear1.GetVisualShape(0).SetMeshTransform(chrono.ChVector3d(0.02, 0.02, 0.02))

mesh.SetShape(BRepPrimAPI.BRepPrimAPI_MakeCylinder(0.02, 0.01).Shape())
mesh.GenerateMesh()
trimesh2 = cascade.ChCascadeTriangulator.GetChronoMesh(mesh)
gear2.GetVisualShape(0).SetMesh(trimesh2)
gear2.GetVisualShape(0).SetMeshTransform(chrono.ChVector3d(0.02, 0.02, 0.02))


wall = chrono.ChBodyEasyBox(1, 0.2, 1, 1000) 
wall.SetPos(chrono.ChVector3d(3,0.3,0))
wall.SetFixed(True)
wall.GetVisualShape(0).SetMaterial(0, material)
sys.Add(wall)


trackPoint = chrono.ChVector3d(0.8, 0.1, 0.2)
sys.SetChTime(0)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.2,0.2,-0.2))
vis.AddTypicalLights()


my_time = 0
while vis.Run() :
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)