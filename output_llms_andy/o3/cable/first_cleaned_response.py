import math
import pychrono.core      as chrono
import pychrono.irrlicht  as chronoirr
import pychrono.fea       as fea            


chrono.SetChronoDataPath( chrono.GetChronoDataPath() )    
sys = chrono.ChSystemSMC()                                
sys.Set_G_acc( chrono.ChVectorD(0, -9.81, 0) )            





mesh          = fea.ChMesh()
sys.Add(mesh)


density       = 7.8e3                    
E             = 2.1e11                   
G             = 8.1e10                   
diam          = 0.02                     
A             = math.pi * (diam**2) / 4  
I             = math.pi * (diam**4) / 64 
mat_cable     = fea.ChMaterialCableANCF(E, G, density)


L             = 2.0          
Ne            = 20           
dx            = L / Ne


nodes = []
for i in range(Ne + 1):
    pos = chrono.ChVectorD(i * dx, 0.0, 0.0)
    n   = fea.ChNodeFEAxyzD(pos)   
    n.SetMass(density * A * dx)    
    mesh.AddNode(n)
    nodes.append(n)


nodes[0].SetFixed(True)



for i in range(Ne):
    cab = fea.ChElementCableANCF()
    cab.SetNodes( nodes[i], nodes[i+1] )
    cab.SetMaterial(mat_cable)
    cab.SetSectionRadius(diam/2)
    cab.SetAlphaDamp(0.02)          
    mesh.AddElement(cab)






mesh_vis_nodes = fea.ChVisualizationFEAmesh(mesh)
mesh_vis_nodes.SetFEMglyphType( fea.ChVisualizationFEAmesh.GlyphType_NODE_DOT_POS )
mesh_vis_nodes.SetSymbolsThickness(0.008)
mesh_vis_nodes.SetDataType( fea.ChVisualizationFEAmesh.DataType_NONE )
mesh.AddAsset(mesh_vis_nodes)


mesh_vis_cable = fea.ChVisualizationFEAmesh(mesh)
mesh_vis_cable.SetFEMdatasmooth(True)
mesh_vis_cable.SetWireframe(False)
mesh_vis_cable.SetDataType( fea.ChVisualizationFEAmesh.DataType_NONE )
mesh.AddAsset(mesh_vis_cable)





app = chronoirr.ChIrrApp(sys,
                         "ANCF Cable beam – hinged end",
                         chronoirr.dimension2du(1024, 768))

app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
app.AddTypicalCamera( chronoirr.vector3df(-1.6, 0.6, 1.6),
                      chronoirr.vector3df( 0.8, 0.0, 0.0) )


app.AssetBindAll()
app.AssetUpdateAll()





h          = 1e-3    
t_end      = 5       

while(app.GetDevice().run()):
    if sys.GetChTime() >= t_end:
        break

    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    sys.DoStepDynamics(h)

print("Simulation finished at t =", sys.GetChTime(), "s")