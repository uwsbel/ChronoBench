import pychrono as chrono

import pychrono.visualization as visual

import pychrono.core as core

import pychrono.fea as fea

import pychrono.analytical_solvers as analytical

import pychrono.compiler as compiler

import pychrono.compiler.gmsh as gmsh

import pychrono.compiler.irrlicht as irrlicht




core.Initialize()




my_system = core.System_Create()






beam_props = fea.Beam_Create()

beam_props.SetElasticModulus(210e9)  

beam_props.SetPoissonsRatio(0.3)     

beam_props.SetDensity(7800)          

beam_props.SetLength(2.0)            

beam_props.SetMassPerLength(100)     




node_props = fea.Node_Create()

node_props.SetPosition(chrono.ChVectorD(0, 0, 0))  

node_props.SetMass(10)                           




beam1 = fea.Beam_Create()

beam1.SetProperties(beam_props)

node1 = fea.Node_Create()

node1.SetProperties(node_props)

beam1.SetNodes(node1)




my_system.Add(beam1)






force = chrono.ChVectorD(0, 0, -1000)  

beam1.AddForce(force)




vis = visual.Visualization_Create()

vis.SetGlOptions("-w 1280 -h 720 -t 2")




compiler.Compile(my_system, "beam_simulation", "irrlicht")




while core.GetCondition() == core.Conditions.ITERATIVE:

    core.DoStep(1e-3, 1e-3)

    vis.Draw()




core.End()