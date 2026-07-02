import pychrono as chrono
import pychrono.fea as fea

system = chrono.ChSystemNSC()
mesh = fea.ChMesh()
system.Add(mesh)


length = 1.0
num_elements = 10
element_length = length / num_elements
E = 2e5  
nu = 0.3  
G = E / (2 * (1 + nu))  
area = 0.01  
density = 1000  
gravity = chrono.ChVector3d(0, -9.81, 0)


nodes = []
for i in range(num_elements + 1):
    x = i * element_length
    node = fea.ChNodeFEAxyz(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    if i == 0:
        node.SetFixed(True)  
    mesh.AddNode(node)
    nodes.append(node)


material = fea.ChMaterialBeamANCF(E, G, nu, density)
for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetRestLength(element_length)
    element.SetArea(area)
    element.SetMaterial(material)
    mesh.AddElement(element)


system.SetGravitationalAcceleration(gravity)


element_vis = chrono.ChVisualShapeFEAbeam()
element_vis.SetFEMdataType(chrono.ChVisualShapeFEAbeam.DataType_ELEM_BEAM_MX)
element_vis.SetColorscaleMinMax(-100, 100)
element_vis.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
mesh.AddVisualShapeFEA(element_vis)


node_vis_bodies = []
for node in nodes:
    body = chrono.ChBodyAuxRef()
    body.SetName("Node Visual")
    body.SetFixed(True)
    body.EnableCollision(False)
    sphere = chrono.ChVisualShapeSphere(0.02)
    sphere.SetColor(chrono.ChColor(1, 0, 0))
    body.AddVisualShape(sphere)
    system.Add(body)
    node_vis_bodies.append(body)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 720)
vis.SetWindowTitle("ANCF Beam Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


time_step = 0.001
while vis.Run():
    
    for i, body in enumerate(node_vis_bodies):
        body.SetPos(nodes[i].GetPos())
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)