import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  


beam_length = 2.0
beam_height = 0.1
beam_width = 0.1
num_elements = 10
density = 1000.0
young_modulus = 2.0e11  
shear_modulus = 0.8e11


mesh = fea.ChMesh()


nodes = []
for i in range(num_elements + 1):
    pos = chrono.ChVector3d(i * beam_length / num_elements, 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(pos))
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    element = fea.ChElementCableANCF(mesh)
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetArea(beam_height * beam_width)
    element.SetMaterial(fea.ChMaterialSurfaceNSC(young_modulus, shear_modulus, density))
    mesh.AddElement(element)


nodes[0].SetFixed(True)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -5))
vis.AddTypicalLights()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)