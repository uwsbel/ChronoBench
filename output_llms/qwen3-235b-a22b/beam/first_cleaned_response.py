import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()


mesh = fea.ChMesh()


num_nodes = 10
beam_length = 1.0
beam_width = 0.02
beam_height = 0.02
E = 2e11  
nu = 0.3   
G = E / (2 * (1 + nu))  
density = 7800  


area = beam_width * beam_height
Iyy = (beam_width * beam_height**3) / 12
Izz = (beam_height * beam_width**3) / 12
J = Iyy + Izz  


material = fea.ChMaterialBeamANCF(density, E, G, area, Iyy, Izz, J)


nodes = []
for i in range(num_nodes):
    x = i * beam_length / (num_nodes - 1)
    node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_nodes - 1):
    element = fea.ChElementBeam()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetMaterial(material)
    mesh.AddElement(element)


system.Add(mesh)


constraint = chrono.ChLinkPointFrame()
constraint.Initialize(nodes[0], system.GetGroundBody())
system.Add(constraint)


application = irrlicht.ChIrrApp(system, 'Beam Finite Element Simulation', irrlicht.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(irrlicht.vector3df(0, 0.5, 1))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


visual_asset = fea.ChVisualizationFEAmesh(mesh)
visual_asset.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_VONMISES_STRAIN)
visual_asset.SetColorscaleMinMax(0, 5e7)
visual_asset.SetBeamRadius(0.005)
mesh.AddAsset(visual_asset)


application.SetTimestep(0.001)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())