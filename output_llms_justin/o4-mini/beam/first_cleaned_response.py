import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))   




mesh = fea.ChMesh()


section = chrono.ChBeamSectionEuler()
E = 2e11                
rho = 7800              
nu = 0.3                
section.SetYoungModulus(E)
section.SetGwithPoissonRatio(E / (2*(1+nu)), nu)
section.SetDensity(rho)
section.SetAsCircularSection(0.02)  




num_elems   = 6
elem_length = 0.5
nodes = []

for i in range(num_elems + 1):
    x = i * elem_length
    node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, 0, 0))
    node.SetMass(0)           
    mesh.AddNode(node)
    nodes.append(node)


nodes[0].SetFixed(True)




for i in range(num_elems):
    beam = fea.ChElementBeamEuler()
    beam.SetNodes(nodes[i], nodes[i+1])    
    beam.SetSection(section)               
    mesh.AddElement(beam)




system.Add(mesh)





vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetWireframe(True)
vis_mesh.SetDefaultMeshColor(chrono.ChColor(0.2, 0.5, 0.8))
mesh.AddAsset(vis_mesh)




application = irr.ChIrrApp(system, "PyChrono Beam FEA Demo", irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(3,1.5,3))


application.AssetBindAll()
application.AssetUpdateAll()




application.SetTimestep(1e-3)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()