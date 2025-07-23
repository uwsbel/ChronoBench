import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()


material = fea.ChMaterialBeamANCF(1000, 0.01e9, 0.3)


beam_L = 1.0  
num_elements = 10  


for i in range(num_elements + 1):
    node = fea.ChNodeFEAxyzD(chrono.ChVector3d(i * beam_L / num_elements, 0, 0), chrono.ChVector3d(1, 0, 0))
    node.SetMass(0)
    mesh.AddNode(node)

for i in range(num_elements):
    node1 = mesh.GetNode(i)
    node2 = mesh.GetNode(i + 1)
    element = fea.ChElementCableANCF()
    element.SetNodes(node1, node2)
    element.SetMaterial(material)
    element.SetSection(0.01)
    mesh.AddElement(element)


sys.Add(mesh)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


constraint = chrono.ChLinkMateGeneric(True, True, True, True, True, True)
constraint.Initialize(mesh.GetNode(0), ground, False, mesh.GetNode(0).GetFrame_COG(), mesh.GetNode(0).GetFrame_COG())
sys.Add(constraint)


sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.5, 1.0))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    sys.DoStepDynamics(0.001)
    
    
    for node in mesh.GetNodes():
        pos = node.GetPos()
        vis.GetSceneManager().addSphereSceneNode(0.01, 16, None, -1, pos)
    
    vis.EndScene()