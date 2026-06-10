import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr






system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
system.SetSolver(solver)


timestepper = chrono.ChTimestepperEulerImplicitLinearized(system)
system.SetTimestepper(timestepper)






mesh = fea.ChMesh()
system.Add(mesh)


beam_section = fea.ChBeamSectionEulerAdvanced()

beam_width = 0.04
beam_height = 0.04
beam_section.SetAsRectangularSection(beam_width, beam_height)

beam_section.SetYoungModulus(2.0e7)
beam_section.SetGshearModulus(8.0e6)
beam_section.SetDensity(1200.0)
beam_section.SetBeamRaleyghDamping(0.002)


beam_length = 2.0
num_elements = 16
dx = beam_length / num_elements

nodes = []


for i in range(num_elements + 1):
    x = i * dx

    node = fea.ChNodeFEAxyzrot(
        chrono.ChFramed(
            chrono.ChVector3d(x, 0, 0),
            chrono.QUNIT
        )
    )

    node.SetMass(0.0)

    
    if i == 0:
        node.SetFixed(True)

    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(beam_section)
    mesh.AddElement(element)


tip_force = chrono.ChVector3d(0, -20.0, 0)
nodes[-1].SetForce(tip_force)







beam_vis = fea.ChVisualShapeFEA(mesh)
beam_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_SURFACE)
beam_vis.SetSmoothFaces(True)
beam_vis.SetWireframe(False)
mesh.AddVisualShapeFEA(beam_vis)


moment_vis = fea.ChVisualShapeFEA(mesh)
moment_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
moment_vis.SetColorscaleMinMax(-20.0, 20.0)
moment_vis.SetSmoothFaces(True)
moment_vis.SetWireframe(False)
mesh.AddVisualShapeFEA(moment_vis)


node_vis = fea.ChVisualShapeFEA(mesh)
node_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
node_vis.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
node_vis.SetSymbolsThickness(0.01)
node_vis.SetSymbolsScale(0.02)
node_vis.SetZbufferHide(False)
mesh.AddVisualShapeFEA(node_vis)






vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono FEA Beam Demo")
vis.Initialize()

vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.2, 0.7, 2.8), chrono.ChVector3d(1.0, -0.25, 0.0))
vis.AddTypicalLights()






time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)