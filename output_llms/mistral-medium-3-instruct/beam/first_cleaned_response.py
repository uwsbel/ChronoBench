import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh = chrono.fea.ChMesh()


material = chrono.ChMaterialShellANCF()
material.SetYoungModulus(2.1e11)  
material.SetPoissonRatio(0.3)
material.SetDensity(7850)         


num_nodes = 10
nodes = []
for i in range(num_nodes):
    node = chrono.fea.ChNodeFEAxyzD(chrono.ChVectorD(i * 0.1, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)


nodes[0].SetFixed(True)


for i in range(num_nodes - 1):
    beam = chrono.fea.ChElementBeamANCF()
    beam.SetNodes(nodes[i], nodes[i+1])
    beam.SetMaterial(material)
    beam.SetDimensions(0.01, 0.01)  
    mesh.AddElement(beam)


system.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam FEA Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)


for beam in mesh.GetElementsBeam():
    vis_asset = chrono.fea.ChVisualizationFEAmesh(mesh)
    vis_asset.SetFEMdataType(chrono.fea.ChVisualizationFEAmesh.E_PLOT_NODES)
    vis_asset.SetFEMglyphType(chrono.fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
    vis_asset.SetFEMcolorscaleMinMax(-0.1, 0.1)
    vis_asset.SetSmoothFaces(True)
    beam.AddAsset(vis_asset)


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)