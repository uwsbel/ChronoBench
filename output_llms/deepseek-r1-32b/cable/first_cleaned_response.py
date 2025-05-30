importchrono
import math


system = chrono.ChSystem()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


application = chrono.ChIrrApp(system, "ANCF Beam Simulation", chrono.CHDimension(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.SetCamera(chrono.ChVectorD(0, 5, 5), chrono.ChVectorD(0, 0, 0))


num_elements = 10
element_length = 0.5
total_length = num_elements * element_length
mass_per_unit_length = 1.0
cross_section_area = 0.01
young_modulus = 2e5


nodes = []
for i in range(num_elements + 1):
    node = chrono.ChNodeFEAxyz()
    node.SetPos(chrono.ChVectorD(i * element_length, 0, 0))
    system.Add(node)
    nodes.append(node)


for i in range(num_elements):
    cable = chrono.ChElementCableANCF()
    cable.SetNodes(nodes[i], nodes[i+1])
    cable.SetProperties(mass_per_unit_length, cross_section_area, young_modulus)
    system.Add(cable)


nodes[0].GetBody().SetMass(0)
nodes[0].GetBody().SetBodyFixed(True)


mesh = chrono.ChTriangleMeshShape()
for i in range(num_elements):
    
    p1 = nodes[i].GetPos()
    p2 = nodes[i+1].GetPos()
    
    mesh.AddPoint(p1)
    mesh.AddPoint(p2)


body = chrono.ChBody()
body.SetMaterialSurface(chrono.ChMaterialSurface())
body.GetCollisionModel().SetFamily(chrono.COLLIDE_FAMILY_GHOST)
body.AddAsset(mesh)
system.Add(body)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChBoxShape()
ground_shape.SetDimensions(chrono.ChVectorD(total_length, 1, 1))
ground.AddAsset(ground_shape)
ground.AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))
system.Add(ground)


mesh.Compile()


application.AssetSetShow(chrono.CHAABB_TYPE, False)
application.AssetSetShow(chrono.CHNODE_TYPE, True)
application.AssetSetShow(chrono.CHELEMENT_TYPE, True)

while application.GetDevice().Run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    
    
    system.DoStepDynamics(0.001)
    
    
    if system.GetChTime() % 0.1 < 0.001:
        print(f"Time: {system.GetChTime():.3f}s")