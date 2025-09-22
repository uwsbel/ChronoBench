import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemSMC()  




beam_length = 1.0
beam_height = 0.02
beam_width = 0.02
beam_thickness = 0.02
num_elements = 20


rho = 7800      
E = 2.1e11      
nu = 0.3        

beam_material = fea.ChMaterialBeamANCF(E, nu, rho)


mesh = fea.ChMesh()


nodes = []
for i in range(num_elements + 1):
    x = beam_length * i / num_elements
    node = fea.ChNodeFEAxyzD(chrono.ChVectorD(x, 0, 0), chrono.ChVectorD(0, 1, 0))
    node.SetMass(0.1)
    mesh.AddNode(node)
    nodes.append(node)

for i in range(num_elements):
    element = fea.ChElementBeamANCF()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetDimensions(beam_length / num_elements, beam_width, beam_height)
    element.SetMaterial(beam_material)
    mesh.AddElement(element)

system.Add(mesh)




fix_constraint = chrono.ChLinkMateGeneric(True, True, True, True, True, True)
fix_constraint.Initialize(nodes[0], chrono.ChBody(), False, nodes[0].GetPos(), chrono.ChVectorD(0,0,0))
system.Add(fix_constraint)



ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(nodes[-1], ground, chrono.ChCoordsysD(nodes[-1].GetPos(), chrono.Q_from_AngZ(0)))
system.Add(prismatic)


class CustomDisplacement(chrono.ChFunction):
    def Get_y(self, t):
        
        return min(0.01 * t, 0.01)

motor = chrono.ChLinkLinActuator()
motor.Initialize(nodes[-1], ground, chrono.ChFrameD(nodes[-1].GetPos(), chrono.QUNIT))
motor.SetDistanceOffset(0)
motor.SetActuatorFunction(CustomDisplacement())
system.Add(motor)


application = chronoirr.ChVisualSystemIrrlicht()
application.AttachSystem(system)
application.SetWindowSize(1024,768)
application.SetWindowTitle('PyChrono FEA Beam Buckling')
application.Initialize()
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0.5, 0.1, 0.5), chrono.ChVectorD(0.5,0,0))
application.AddTypicalLights()


beam_vis = fea.ChVisualizationFEAmesh(mesh)
beam_vis.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
beam_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
beam_vis.SetSymbolsThickness(0.006)
beam_vis.SetSymbolsScale(0.01)
mesh.AddAsset(beam_vis)

beam_colormap = fea.ChVisualizationFEAmesh(mesh)
beam_colormap.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
beam_colormap.SetColorscaleMinMax(-500, 500)
beam_colormap.SetSmoothFaces(True)
beam_colormap.SetWireframe(True)
mesh.AddAsset(beam_colormap)


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
system.SetSolver(solver)

system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


timestep = 1e-4
while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(timestep)