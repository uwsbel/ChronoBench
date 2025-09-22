import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()


mesh = fea.ChMesh()
system.Add(mesh)


beam_section = fea.ChBeamSectionAdvanced()
beam_section.SetYoungModulus(2.1e7)      
beam_section.SetGwithPoissonRatio(0.3)   
beam_section.SetDensity(7800)            
beam_section.SetAsRectangularSection(0.02, 0.01)  


n_nodes = 5
beam_length = 1.0
node_spacing = beam_length / (n_nodes - 1)
nodes = []


for i in range(n_nodes):
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * node_spacing, 0, 0)))
    node.SetMass(0.1)
    mesh.AddNode(node)
    nodes.append(node)


nodes[0].SetFixed(True)


for i in range(n_nodes - 1):
    beam = fea.ChElementBeamEuler()
    beam.SetNodes(nodes[i], nodes[i+1])
    beam.SetSection(beam_section)
    mesh.AddElement(beam)


load_container = chrono.ChLoadContainer()
system.Add(load_container)


tip_force = chrono.ChVectorD(0, -100, 0)  
tip_load = fea.ChLoadNodeForce(nodes[-1], tip_force)
load_container.Add(tip_load)



beam_visual = fea.ChVisualizationFEAmesh(mesh)
beam_visual.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
beam_visual.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
beam_visual.SetSymbolsThickness(0.006)
beam_visual.SetSymbolsScale(1)
mesh.AddAsset(beam_visual)


beam_deform = fea.ChVisualizationFEAmesh(mesh)
beam_deform.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
beam_deform.SetColorscaleMinMax(-0.4, 0.4)
beam_deform.SetSmoothFaces(True)
mesh.AddAsset(beam_deform)


application = chronoirr.ChIrrApp(system, "PyChrono Beam Finite Element Demo", chronoirr.dimension2du(1024,768))
application.AddTypicalSky()
application.AddTypicalCamera(chrono.ChVectorD(0.5, 0.2, 1.0), chrono.ChVectorD(0.5, 0, 0))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.001)
while(application.GetDevice().run()):
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()