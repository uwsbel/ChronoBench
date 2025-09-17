import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


sys = fea.ChSystemFEM()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


nodes = []
for i in range(6):
    node = fea.ChNodeFEAxyz()
    node.SetPos(chrono.ChVectorD(i, 0, 0))
    sys.AddNode(node)
    nodes.append(node)


nodes[0].SetFixed(True)


for i in range(len(nodes) - 1):
    beam = fea.ChLinkFEAbeam()
    beam.Initialize(nodes[i], nodes[i+1])
    
    
    beam.Set_A(0.1)        
    beam.Set_Iy(0.01)      
    beam.Set_Iz(0.01)      
    beam.Set_E(2e11)       
    beam.Set_nu(0.3)       
    
    
    vis = fea.ChVisualizationFEAbeam()
    vis.SetRadius(0.1)     
    beam.AddAsset(vis)
    
    sys.AddLink(beam)


renderer = irr.ChIrrApp(sys, 'Beam FEA Simulation', irr.dimension2du(800, 600))
renderer.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
renderer.SetCameraVertical(chrono.CH_VECT_Y)
renderer.AllocateResources()


for node in nodes:
    sphere = irr.ChSphereShape()
    sphere.GetSphereGeometry().rad = 0.2  
    node.AddAsset(sphere)


renderer.SetTimestep(0.001)
renderer.SetTryRealtime(True)
renderer.Execute()