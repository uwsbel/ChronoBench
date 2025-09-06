import pychrono

import pychrono.features as fea

import pychrono.Irrlicht as irr




chrono = pychrono.ChSystemNSC()




chrono.AddGravity(pychrono.ChVectorD(0, -9.81, 0))




beam_length = 1.0

beam_height = 0.1

beam_width = 0.1

beam_density = 7800




material = pychrono.ChMaterialEE(pychrono.ChMaterial_Aluminum)

material.SetDensity(beam_density)

material.SetYoungsModulus(70e9)

material.SetPoissonsRatio(0.33)




node1 = chrono.CreateNode(pychrono.ChVectorD(0, 0, 0))

node2 = chrono.CreateNode(pychrono.ChVectorD(beam_length, 0, 0))




beam1 = chrono.CreateBeamD1Element(node1, node2, beam_width, beam_height, material)




chrono.AddBody(beam1)




visualization = irr.IVVisualSystem()

chrono.SetVisualization(visualization)




irrlicht_renderer = irr.IVideoDriverSceneNodeRenderBridge(chrono.GetVisualization(), chrono.GetTimeStep())




while(chrono.GetChTime() < 10.0):

    chrono.DoStepDynamics(0.01)

    irrlicht_renderer.Render()




chrono.Destroy()