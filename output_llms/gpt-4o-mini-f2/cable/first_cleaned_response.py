import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


mesh = fea.ChMesh()


section = fea.ChBeamSectionCable()
section.SetDiameter(0.02)  
section.SetYoungModulus(0.01e9)  
section.SetRayleighDamping(0.000)  


nodes = []
for i in range(6):
    node = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 0))  
    mesh.AddNode(node)  
    nodes.append(node)  


for i in range(5):
    element = fea.ChElementBeamANCF()  
    element.SetNodes(nodes[i], nodes[i + 1])  
    element.SetSection(section)  
    mesh.AddElement(element)  


truss_nodes = []
for i in range(3):
    node = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0.1 * i, 0))  
    mesh.AddNode(node)  
    truss_nodes.append(node)  


for i in range(2):
    element = fea.ChElementBeamEuler()  
    element.SetNodes(truss_nodes[i], truss_nodes[i + 1])  
    element.SetSection(section)  
    mesh.AddElement(element)  


fixed_node = fea.ChNodeFEAxyzrot(chrono.ChVector3d(0, 0, 0))
mesh.AddNode(fixed_node)
element = fea.ChElementBeamEuler()
element.SetNodes(fixed_node, truss_nodes[0])
element.SetSection(section)
mesh.AddElement(element)


fea_manager = system.AddFEMBaseContrived()
fea_manager.AddMesh(mesh)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


load = fea.ChLoadNodeBody()
load.SetNode(nodes[0])  
load.SetBodyFixed(True)  
load.SetForce(chrono.ChVector3d(0, 0, -0.1))  
mesh.AddLoad(load)  


load = fea.ChLoadNodeBody()
load.SetNode(truss_nodes[2])  
load.SetBodyFixed(True)  
load.SetForce(chrono.ChVector3d(0, 0, -0.1))  
mesh.AddLoad(load)  


bc = fea.ChBoundaryNodeFixed()
bc.SetNode(nodes[5])  
mesh.AddBoundary(bc)  


bc = fea.ChBoundaryNodeFixed()
bc.SetNode(truss_nodes[0])  
mesh.AddBoundary(bc)  


bc = fea.ChBoundaryNodeConstrained()
bc.SetNode(truss_nodes[2])  
bc.ConstrainX(True)  
bc.ConstrainY(True)  
mesh.AddBoundary(bc)  


bc = fea.ChBoundaryNodeConstrained()
bc.SetNode(nodes[0])  
bc.ConstrainX(True)  
bc.ConstrainY(True)  
mesh.AddBoundary(bc)  


system.Setup()
system.CalculateNFm()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
    system.DoStepDynamicsFEA(0.001)