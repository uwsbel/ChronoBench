import pychrono as chrono
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()


mesh = chrono.ChMesh()


num_elements = 10
beam_length = 1.0
beam_radius = 0.05  


for i in range(num_elements + 1):
    x = i * beam_length / num_elements
    node = chrono.ChNodeFEAxyz(chrono.ChVectorD(x, 0, 0))
    if i == 0:  
        node.SetFixed(True)
    mesh.AddNode(node)


section = chrono.ChBeamSectionTimoshenko()
section.SetE(1e4)  
section.SetG(1e4 / (2 * (1 + 0.3)))  
section.SetArea(0.01)  
section.SetIyy(0.001)  
section.SetIzz(0.001)
section.SetJ(0.0005)  
section.SetDensity(1000)  


for i in range(num_elements):
    nodeA = mesh.GetNode(i)
    nodeB = mesh.GetNode(i + 1)
    beam = chrono.ChElementBeamTimoshenko()
    beam.SetNodes(nodeA, nodeB)
    beam.SetSection(section)
    beam.SetReferenceDirection(chrono.ChVectorD(0, 1, 0))  
    mesh.AddElement(beam)


my_system.Add(mesh)


application = chronoirr.ChIrrApp(my_system, 'Beam FEA Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0.5, -1, -2))  


vis_mesh = chrono.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_ELEM_BEAM)
vis_mesh.SetColorscaleMinMax(0, 1)
vis_mesh.SetBeamRadius(beam_radius)
mesh.AddAsset(vis_mesh)


node_vis = chrono.ChVisualizationFEAmesh(mesh)
node_vis.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_NODE_POINT)
node_vis.SetColor(chrono.ChColor(1, 0, 0))
mesh.AddAsset(node_vis)

application.AssetBindAll()
application.AssetUpdateAll()


my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
my_system.SetSolverType(chrono.ChSolver.Type_MINRES)
my_system.SetSolverMaxIterations(100)
my_system.SetMaxPenetrationHysteresis(0.01)

time_step = 0.01
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    my_system.DoStepDynamics(time_step)
    application.EndScene()