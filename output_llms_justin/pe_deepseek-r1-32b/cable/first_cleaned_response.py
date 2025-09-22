import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath(chrono.GetChronoDataPath() + "/")


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beam ANCF Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


beam_length = 5.0
num_elements = 10
element_length = beam_length / num_elements
beam_width = 0.1
beam_height = 0.05


mesh = fea.ChMesh()
mesh.SetMaterial(chrono.ChMaterialANCF())


nodes = []
for i in range(num_elements + 1):
    pos = chrono.ChVector3d(i * element_length, 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(pos))
    nodes.append(node)
    mesh.AddNode(node)


for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetRadius(0.02)
    element.SetYoungModulus(2.0e8)
    element.SetShearModulus(8.0e7)
    element.SetMassPerUnitLength(0.1)
    mesh.AddElement(element)


fixed_body = chrono.ChBody()
fixed_body.SetPos(chrono.ChVector3d(0, 0, 0))
fixed_body.SetFixed(True)
sys.Add(fixed_body)

link = chrono.ChLinkLock()
link.Initialize(fixed_body, nodes[0], chrono.ChFramed())
sys.Add(link)


mesh.SetBodyFixed(False)
mesh.SetGravity(chrono.ChVector3d(0, -9.81, 0))
sys.Add(mesh)


mesh_shape = fea.ChVisualShapeFEA()
mesh_shape.SetFEMesh(mesh)
mesh_shape.SetColor(chrono.ChColor(0.8, 0, 0))
mesh.AddVisualShape(mesh_shape)


time_step = 0.001
num_steps = 5000

for step in range(num_steps):
    sys.DoStepDynamics(time_step)
    
    if vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        if step % 100 == 0:
            print(f"Time: {step * time_step:.3f}s")
            for i, node in enumerate(nodes):
                pos = node.GetPos()
                print(f"Node {i}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")

vis.Close()