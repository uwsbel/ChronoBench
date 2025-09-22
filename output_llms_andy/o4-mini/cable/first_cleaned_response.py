import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr




system = chrono.ChSystemSMC()

system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




mesh = fea.ChMesh()


young_modulus = 2e11     
density       = 7800     
area          = 1e-4     

section = fea.ChCableSectionAdvanced()
section.SetDensity(density)
section.SetYoungModulus(young_modulus)
section.SetDiameter(2.0 * chrono.sqrt(area / chrono.CH_C_PI))
section.SetAlphaDamp(0.0)
section.SetGamma(0.0)




L      = 1.0       
nSeg   = 20        
dL     = L / nSeg  

nodes = []
for i in range(nSeg + 1):
    
    x = dL * i
    pos = chrono.ChVectorD(x, 0, 0)
    
    D0 = chrono.ChVectorD(1, 0, 0)
    node = fea.ChNodeFEAxyzD(pos, D0)
    
    if i == 0:
        node.SetFixed(True)
    mesh.AddNode(node)
    nodes.append(node)


for i in range(nSeg):
    elem = fea.ChElementCableANCF()
    elem.SetNodes(nodes[i], nodes[i+1])
    elem.SetSection(section)
    
    mesh.AddElement(elem)





vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetWireframe(True)
vis_mesh.SetDrawNodes(True)
mesh.AddAsset(vis_mesh)




system.Add(mesh)




application = irr.ChIrrApp(
    system,
    "ANCF Cable Beam Example",
    irr.dimension2du(800, 600),
    irr.DO_NOT_SHARE_CONTEXT)


application.AddTypicalLogo(irr.vector2di(10, 10))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(0.5, 0.5, 1.5))
application.AssetBindAll()
application.AssetUpdateAll()




application.SetTimestep(1e-3)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    application.DoStep()
    application.EndScene()