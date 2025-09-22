import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  

system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




mesh = fea.ChMesh()


beam_length = 2.0  
beam_radius = 0.02  
beam_density = 7850  
E = 2.1e11         
nu = 0.3           
G = E / (2 * (1 + nu))


num_elements = 10
num_nodes = num_elements + 1
element_length = beam_length / num_elements





nodes = []
for i in range(num_nodes):
    x = i * element_length
    y = 0
    z = 0
    
    
    node = fea.ChNodeFEAxyzD(chrono.ChVectorD(x, y, z), chrono.ChVectorD(0, 0, 1))
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    
    element.SetDimensions(beam_radius, element_length)
    element.SetDensity(beam_density)
    element.SetYoungModulus(E)
    element.SetGshearModulus(G)
    mesh.AddElement(element)


system.Add(mesh)







nodes[0].SetFixed(True)






application = chronoirr.ChIrrApp(system, "ANCF Cable Beam", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(2, -4, 1.5))


for node in nodes:
    sphere = chrono.ChSphereShape()
    sphere.GetSphereGeometry().rad = 0.025
    node.AddAsset(sphere)


mesh_shape = fea.ChVisualizationFEAmesh(mesh)
mesh_shape.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_ELEM)
mesh_shape.SetColorscaleMinMax(-1.0, 1.0)
mesh_shape.SetShrinkElements(True, 0.85)
mesh_shape.SetWireframe(False)
mesh.AddAsset(mesh_shape)

application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(1e-3)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    
    
    

    application.DoStep()
    application.EndScene()