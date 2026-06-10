import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math






sys = chrono.ChSystemSMC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))






mesh = fea.ChMesh()





beam_section = fea.ChBeamSectionEulerAdvanced()


beam_width  = 0.012   
beam_height = 0.025   
beam_section.SetAsRectangularSection(beam_width, beam_height)


beam_section.SetYoungModulus(0.01e9)        
beam_section.SetShearModulusFromPoisson(0.3)  
beam_section.SetRayleighDamping(0.000)       
beam_section.SetDensity(1000)                




beam_length = 0.5            
num_elements = 8             
num_nodes = num_elements + 1

nodes = []
for i in range(num_nodes):
    x = i * (beam_length / num_elements)
    
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    mesh.AddNode(node)
    nodes.append(node)


nodes[0].SetFixed(True)




for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(beam_section)
    mesh.AddElement(element)




nodes[-1].SetForce(chrono.ChVector3d(0, -1.0, 0))     



sys.Add(mesh)






vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.4, 0.4)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)


vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono - Beam Finite Elements (FEA)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.3, -0.6),
              chrono.ChVector3d(0.25, 0.0, 0.0))
vis.AddTypicalLights()




solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-12)
solver.EnableDiagonalPreconditioner(True)
solver.SetVerbose(False)


sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
stepper = chrono.CastToChTimestepperHHT(sys.GetTimestepper())
if stepper is not None:
    stepper.SetAlpha(-0.2)
    stepper.SetMaxIters(8)
    stepper.SetAbsTolerances(1e-6)




time_step = 0.001

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)

print("Simulation finished.")