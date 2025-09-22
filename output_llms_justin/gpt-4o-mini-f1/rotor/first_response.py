import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math

# Create the Chrono physical system
sys = chrono.ChSystemSMC()

# Create a mesh object to hold the IGA elements
mesh = fea.ChMesh()

# Create a rectangular IGA beam section
section = fea.ChBeamSectionRectangular(0.1, 0.01)

# Create the IGA beam element
beam = fea.ChElementBeamIGA()
beam.SetSection(section)

# Define the control points for the IGA beam
control_points = [
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0.5, 0, 0),
    chrono.ChVector3d(1, 0, 0),
]

# Set the control points and the number of spans for the IGA beam
beam.SetControlPoints(control_points)
beam.SetNumSpans(2)

# Set the beam's reference frame
beam.SetFrameRef(chrono.ChFramed(control_points[0]))

# Set the mesh to which the beam belongs
beam.SetMesh(mesh)

# Add the beam element to the mesh
mesh.AddElement(beam)

# Create a flywheel body
flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.1, 0.1, 1000, True, True)
flywheel.SetPos(chrono.ChVector3d(0.25, 0, 0))
flywheel.SetFixed(False)
flywheel.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))

# Add the flywheel to the physical system
sys.Add(flywheel)

# Create a revolute joint to attach the flywheel to the beam
joint = chrono.ChLinkLockRevolute()
joint.Initialize(flywheel, beam.GetBody(), chrono.ChFramed(flywheel.GetPos(), chrono.ChAxis_Y))
sys.Add(joint)

# Create a motor to drive the beam
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetBody(), flywheel, chrono.ChFramed(flywheel.GetPos(), chrono.ChAxis_Y))
motor.SetMotorFunction(chrono.ChFunctionConst(10))
sys.Add(motor)

# Create a fixed body to serve as a ground
ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)

# Create a prismatic joint to constrain the beam's motion
prismatic_joint = chrono.ChLinkLockPrismatic()
prismatic_joint.Initialize(beam.GetBody(), ground, chrono.ChFramed(beam.GetBody().GetPos(), chrono.ChAxis_Y))
sys.Add(prismatic_joint)

# Create a force to apply to the beam
force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetF_x(0)
force.SetF_y(0)
force.SetF_z(0)
force.SetRelPos(chrono.ChVector3d(0.5, 0, 0))
beam.GetBody().AddForce(force)

# Create a load to apply to the beam
load = chrono.ChLoadBodyForce()
load.SetBody(beam.GetBody())
load.SetForce(chrono.ChVector3d(0, 0, 0))
load.SetPos(chrono.ChVector3d(0.5, 0, 0))
mesh.AddLoad(load)

# Create a load to apply to the beam (using ChLoadBody)
load2 = chrono.ChLoadBody()
load2.SetBody(beam.GetBody())
load2.SetForce(chrono.ChVector3d(0, 0, 0))
load2.SetPos(chrono.ChVector3d(0.5, 0, 0))
mesh.AddLoad(load2)

# Create a ChVisualShapeFEA for FEM visualization
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

# Add the visual shape to the mesh
mesh.AddVisualShapeFEA(visual_shape)

# Create an Irrlicht visualization window
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(chrono.ChVector2i(1280, 720))
vis.SetWindowTitle('Jeffcott rotor with IGA beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5), chrono.ChVector3d(0.5, 0, 0))
vis.AddTypicalLights()

# Add the mesh to the visualization system
vis.AddSystem(mesh)

# Set the solver for the physical system
sys.SetSolverType(chrono.ChSolver.Type_SOR)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)