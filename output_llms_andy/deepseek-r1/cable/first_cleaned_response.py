import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
system.AddBody(ground)
ground.SetBodyFixed(True)  


mesh = fea.ChMesh()
system.Add(mesh)


beam_length = 2.0          
num_elements = 10          
node_spacing = beam_length / num_elements
section_area = 0.01        
density = 1000             
young_modulus = 2e9        


nodes = []
for i in range(num_elements + 1):
    
    position = chrono.ChVectorD(i * node_spacing, 0, 0)
    
    
    node = fea.ChNodeFEAxyzD(position, chrono.ChVectorD(0, 0, 1))
    mesh.AddNode(node)
    nodes.append(node)
    
    
    if i == 0:
        constraint = fea.ChLinkPointFrame()
        constraint.Initialize(node, ground)
        mesh.AddLink(constraint)


for i in range(num_elements):
    element = fea.ChElementBeamANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSectionDimensions(section_area, 0.1)  
    element.SetMaterial(fea.ChBeamSectionCable(density, young_modulus))
    element.SetAlphaDamp(0.01)  
    mesh.AddElement(element)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 0.5, 2))
vis.AddTypicalLights()


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)