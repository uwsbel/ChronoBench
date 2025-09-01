import pychrono as chrono


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)



beam_length = 2.0    
num_nodes = 10       
node_positions = []


for i in range(num_nodes):
    x = (beam_length / (num_nodes - 1)) * i
    node_positions.append(chrono.ChVectorD(x, 0, 0))


nodes = []

for i, pos in enumerate(node_positions):
    node = chrono.ChNodeFEAxyz()  
    node.SetPos(pos)
    
    node.SetMass(0.1)  
    
    system.Add(node)
    nodes.append(node)





elements = []

for i in range(num_nodes - 1):
    nodeA = nodes[i]
    nodeB = nodes[i + 1]
    
    
    
    
    element = chrono.ChElementCableANCF()
    element.SetNodes(nodeA, nodeB)
    element.SetMaterial(chrono.ChMaterialCable())
    
    element.SetStiffness(1e4)  
    system.Add(element)
    elements.append(element)




fixed_constraint = chrono.ChLinkMateFix()
fixed_constraint.Initialize(nodes[0], ground)
system.Add(fixed_constraint)


application = chrono.ChIrrApp(system, 'ANCF Beam Simulation', chrono.dimension2d(800, 600))
application.AddTypicalCamera()
application.SetSkyBox()
application.DrawAll()
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 1e-3
total_time = 2.0  
current_time = 0

while application.GetDevice().run():
    system.DoStepDynamics(time_step)
    current_time += time_step

    
    application.AssetBindAll()
    application.AssetUpdateAll()

    
    
    if int(current_time * 100) % 100 == 0:
        print(f"Time: {current_time:.3f}s")
        for i, node in enumerate(nodes):
            pos = node.GetPos()
            print(f"Node {i}: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")

    
    application.GetSceneManager().render()