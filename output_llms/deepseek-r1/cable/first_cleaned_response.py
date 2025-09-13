import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh = fea.ChMesh()
system.Add(mesh)


length = 1.0          
n_elements = 8        
density = 1000        
youngs_modulus = 1e7  
area = 0.01 * 0.01    
damping = 0.01        


nodes = []
for i in range(n_elements + 1):
    x_pos = i * (length / n_elements)
    node = fea.ChNodeFEAxyzD(chrono.ChVectorD(x_pos, 0, 0), 
                            chrono.ChVectorD(0, 1, 0))  
    mesh.AddNode(node)
    nodes.append(node)


constraint = fea.ChLinkPointFrame()
constraint.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(constraint)


for i in range(n_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(area)
    element.SetMaterial(chrono.ChBeamSectionCable(density, youngs_modulus, damping))
    mesh.AddElement(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.5))  
vis.AddTypicalLights()


visualize_nodes = chronoirr.ChVisualShapePoint(0.02)
visualize_beam = chronoirr.ChVisualShapeFEA(mesh)
visualize_beam.SetFEMdataType(chronoirr.FEMDataType.ELEM_BEAM_MZ)
visualize_beam.SetColorscaleMinMax(-0.1, 0.1)
visualize_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualize_beam)


timestep = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)