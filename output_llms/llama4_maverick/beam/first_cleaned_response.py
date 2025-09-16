import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemSMC()


mesh = chrono.ChMesh()


mat = chrono.ChMaterialSurfaceSMC()
mat.E = 2.1e11  
mat.nu = 0.3   
mat.rho = 7850  


beam_section = chrono.ChBeamSectionEulerAdvanced()
beam_section.SetAsRectangularSection(0.1, 0.2)
beam_section.SetYoungModulus(2.1e11)
beam_section.SetGshearModulus(8.0769e10)
beam_section.SetBeamRho(7850)


num_nodes = 10
num_elements = num_nodes - 1
node_pos = np.linspace(0, 1, num_nodes)

for i in range(num_nodes):
    node = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(node_pos[i], 0, 0)))
    node.SetMass(0.1)
    mesh.AddNode(node)

for i in range(num_elements):
    element = chrono.ChElementBeamEuler()
    element.SetNodes(mesh.GetNode(i), mesh.GetNode(i+1))
    element.SetSection(beam_section)
    mesh.AddElement(element)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Finite Elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, -1, 0.5), chrono.ChVectorD(0.5, 0, 0.5))
vis.AddTypicalLights()


node0 = mesh.GetNode(0)
node0.SetFixed(True)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)