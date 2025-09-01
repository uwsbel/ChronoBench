import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


section = fea.ChBeamSectionRectangular(0.1, 0.01)


beam = fea.ChElementBeamIGA()
beam.SetSection(section)


control_points = [
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0.5, 0, 0),
    chrono.ChVector3d(1, 0, 0),
]


beam.SetControlPoints(control_points)
beam.SetNumSpans(2)


beam.SetFrameRef(chrono.ChFramed(control_points[0]))


beam.SetMesh(mesh)


mesh.AddElement(beam)


flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.1, 0.1, 1000, True, True)
flywheel.SetPos(chrono.ChVector3d(0.25, 0, 0))
flywheel.SetFixed(False)
flywheel.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))


sys.Add(flywheel)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(flywheel, beam.GetBody(), chrono.ChFramed(flywheel.GetPos(), chrono.ChAxis_Y))
sys.Add(joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetBody(), flywheel, chrono.ChFramed(flywheel.GetPos(), chrono.ChAxis_Y))
motor.SetMotorFunction(chrono.ChFunctionConst(10))
sys.Add(motor)


ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)


prismatic_joint = chrono.ChLinkLockPrismatic()
prismatic_joint.Initialize(beam.GetBody(), ground, chrono.ChFramed(beam.GetBody().GetPos(), chrono.ChAxis_Y))
sys.Add(prismatic_joint)


force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetF_x(0)
force.SetF_y(0)
force.SetF_z(0)
force.SetRelPos(chrono.ChVector3d(0.5, 0, 0))
beam.GetBody().AddForce(force)


load = chrono.ChLoadBodyForce()
load.SetBody(beam.GetBody())
load.SetForce(chrono.ChVector3d(0, 0, 0))
load.SetPos(chrono.ChVector3d(0.5, 0, 0))
mesh.AddLoad(load)


load2 = chrono.ChLoadBody()
load2.SetBody(beam.GetBody())
load2.SetForce(chrono.ChVector3d(0, 0, 0))
load2.SetPos(chrono.ChVector3d(0.5, 0, 0))
mesh.AddLoad(load2)


visual_shape = chrono.ChVisualShapeFEA(mesh)
visual_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
visual_shape.SetSmoothFaces(True)
visual_shape.SetWireframe(False)
visual_shape.SetDrawElements(chrono.ChVisualShapeFEA.ElemType_TRIANGLE_MESH, True)
visual_shape.SetDrawElements(chrono.ChVisualShapeFEA.ElemType_BEAM_MESH, True)
visual_shape.SetBeamRadius(0.005)
visual_shape.SetTriangleMesh(chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(chrono.GetChronoDataFile("feaparticles/tri_mesh_1.obj"), True, True))
visual_shape.SetDefaultMeshColor(chrono.ChColor(0.6, 0.6, 0.6))
visual_shape.SetDrawHexahedra(True)
visual_shape.SetDrawQuadrilaterals(True)
visual_shape.SetDrawTetrahedra(True)
visual_shape.SetDrawPolygons(True)
visual_shape.SetDrawEdges(True)
visual_shape.SetDrawVertices(True)


mesh.AddVisualShapeFEA(visual_shape)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(chrono.ChVector2i(1280, 720))
vis.SetWindowTitle('Jeffcott rotor with IGA beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5), chrono.ChVector3d(0.5, 0, 0))
vis.AddTypicalLights()


vis.AddSystem(mesh)


sys.SetSolverType(chrono.ChSolver.Type_SOR)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)