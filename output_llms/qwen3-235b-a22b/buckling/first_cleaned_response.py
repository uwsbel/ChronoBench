import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


material = fea.ChMaterialElasticity()
material.SetYoungModulus(200e9)  
material.SetPoissonRatio(0.3)
material.SetDensity(7800)  


mesh = fea.ChMesh()
L = 1.0  
N = 10   
radius = 0.01  


nodes = []
for i in range(N + 1):
    pos = chrono.ChVectorD(L * i / N, 0, 0)
    node = fea.ChNodeFEAxyzrot()
    node.SetPos(pos)
    mesh.AddNode(node)
    nodes.append(node)


for i in range(N):
    element = fea.ChElementBeamEulerBernoulli()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetMaterial(material)
    element.SetRadius(radius)
    mesh.AddElement(element)


my_system.Add(mesh)


fixed = chrono.ChLinkLockFixed()
fixed.Initialize(nodes[0], chrono.ChFrameD())
my_system.AddLink(fixed)


force_func = chrono.ChFunction_Ramp(0, 400, 2)  
force = chrono.ChForce()
force.SetF_x(force_func)  
force.SetMode(chrono.ChForce.FORCE)  
nodes[-1].AddForce(force)


my_system.SetSolverType(chrono.ChSolver.Type_MINRES)
my_system.SetSolverMaxIterations(100)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
my_system.SetStep(0.001)


application = irr.ChIrrApp(my_system, 'Beam Buckling Simulation', irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0.5, 0.5, -1), chrono.ChVectorD(0.5, 0, 0))
application.AddGrid(0.1, 0.1, 10, 10, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))


vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
vis_mesh.SetWireframe(True)
vis_mesh.SetDefaultMeshColor(chrono.ChColor(0.4, 0.4, 1.0))
mesh.AddAsset(vis_mesh)

application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    my_system.DoStepDynamics(0.001)