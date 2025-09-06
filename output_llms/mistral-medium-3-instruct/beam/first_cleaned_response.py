import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




beam_mesh = chrono.fea.ChMesh()


material = chrono.fea.ChContinuumElastic()
material.SetYoungModulus(2.1e11)  
material.SetPoissonRatio(0.3)
material.SetDensity(7850)  


num_nodes = 10
nodes = []
for i in range(num_nodes):
    
    node = chrono.fea.ChNodeFEAxyz(chrono.ChVectorD(i*0.1, 0, 0))
    node.SetMass(0.1)  
    beam_mesh.AddNode(node)
    nodes.append(node)


nodes[0].SetFixed(True)


for i in range(num_nodes - 1):
    beam = chrono.fea.ChElementBeamEuler()
    beam.SetNodes(nodes[i], nodes[i+1])

    
    beam.SetSection(chrono.fea.ChBeamSectionCircular(0.01))  
    beam.SetMaterial(material)

    beam_mesh.AddElement(beam)


my_system.Add(beam_mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Beam Finite Element Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.5))
vis.AddTypicalLights()


beam_asset = chrono.fea.ChVisualizationFEAmesh(beam_mesh)
beam_asset.SetFEMdataType(chrono.fea.ChVisualizationFEAmesh.E_PLOT_NODES)
beam_asset.SetFEMglyphType(chrono.fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
beam_asset.SetFEMdataType(chrono.fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
beam_asset.SetColorscaleMinMax(-100, 100)
beam_asset.SetSmoothFaces(True)
beam_mesh.AddAsset(beam_asset)


time_step = 0.01
simulation_time = 5.0


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if my_system.GetChTime() < 1.0:
        nodes[-1].SetForce(chrono.ChVectorD(0, -50, 0))

    
    my_system.DoStepDynamics(time_step)

    
    if my_system.GetChTime() >= simulation_time:
        break