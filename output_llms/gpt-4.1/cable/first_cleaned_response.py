import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


beam_length = 2.0
beam_height = 0.02
beam_width = 0.02
num_elements = 10
element_length = beam_length / num_elements
beam_density = 1000  
beam_E = 2e7         
beam_G = 1e7         


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


nodes = []
elements = []


for i in range(num_elements + 1):
    x = i * element_length
    node = chrono.ChNodeFEAxyzD(chrono.ChVectorD(x, 0, 0), chrono.ChVectorD(1, 0, 0))
    node.SetMass(beam_density * beam_width * beam_height * element_length / 2)
    system.Add(node)
    nodes.append(node)


for i in range(num_elements):
    element = chrono.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(chrono.ChBeamSectionCable())
    element.GetSection().SetDiameter(beam_width)
    element.GetSection().SetYoungModulus(beam_E)
    element.GetSection().SetDensity(beam_density)
    system.Add(element)
    elements.append(element)


constraint = chrono.ChLinkPointFrame()
constraint.Initialize(nodes[0], ground)
system.Add(constraint)


application = chronoirr.ChVisualSystemIrrlicht()
application.AttachSystem(system)
application.SetWindowSize(1024, 768)
application.SetWindowTitle('ANCF Cable Beam Deformation')
application.Initialize()
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0.5 * beam_length, 0.2, 0.6), chrono.ChVectorD(0.5 * beam_length, 0, 0))
application.AddTypicalLights()


for element in elements:
    cable_shape = chrono.ChVisualizationFEAmesh(element)
    cable_shape.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
    cable_shape.SetColorscaleMinMax(0.0, 5.0)
    cable_shape.SetSmoothFaces(True)
    cable_shape.SetWireframe(False)
    element.AddAsset(cable_shape)


for node in nodes:
    sphere = chrono.ChSphereShape()
    sphere.GetSphereGeometry().rad = 0.01
    node.AddAsset(sphere)


application.AssetBindAll()
application.AssetUpdateAll()

timestep = 0.002

while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(timestep)