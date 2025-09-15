import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh = fea.ChMesh()
system.Add(mesh)


beam_length = 2.0         
n_elements = 10           
node_spacing = beam_length / n_elements
section_area = 0.01 * 0.01  
density = 7800            
young_modulus = 2e11      
damping_ratio = 0.01      


nodes = []
for i in range(n_elements + 1):
    position = chrono.ChVectorD(i * node_spacing, 0, 0)
    node = fea.ChNodeFEAxyzD(position, chrono.ChVectorD(1, 0, 0))
    nodes.append(node)
    mesh.AddNode(node)

    
    if i == 0:
        constraint = fea.ChLinkPointFrame()
        constraint.Initialize(node, chrono.ChFrameD(position))
        system.Add(constraint)


for i in range(n_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSectionArea(section_area)
    element.SetDensity(density)
    element.SetYoungModulus(young_modulus)
    element.SetDampingCoefficient(damping_ratio)
    mesh.AddElement(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 0.5, 1.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


step_size = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.RenderFrames(2.0)  
    vis.RenderMeshlines()   
    vis.EndScene()
    system.DoStepDynamics(step_size)