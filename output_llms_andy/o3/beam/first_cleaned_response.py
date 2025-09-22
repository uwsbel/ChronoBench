import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math
import sys



chrono.SetChronoDataPath(chrono.GetChronoDataPath())   

system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))        




mesh = chrono.ChMesh()


section = chrono.ChBeamSectionAdvanced()

E  = 2.1e11             
G  = 8.0e10             
rho= 7800               
d  = 0.03               

section.SetAsCircularSection(d)          
section.SetYoungModulus(E)
section.SetGshearModulus(G)
section.SetDensity(rho)
section.SetBeamRaleyghDamping(0.0002)    


N_SEG  = 10                  
L      = 1.0                 
dx     = L / N_SEG           

prev_node = None

for i in range(N_SEG+1):
    
    pos = chrono.ChVectorD(i * dx, 0, 0)
    
    
    node = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(pos))
    node.SetMass(0)          
    mesh.AddNode(node)
    
    
    if prev_node:
        
        element = chrono.ChElementBeamEuler()
        element.SetNodes(prev_node, node)
        element.SetSection(section)
        mesh.AddElement(element)
    
    prev_node = node


ground = chrono.ChBody()           
ground.SetBodyFixed(True)
system.Add(ground)

link = chrono.ChLinkMateGeneric()
link.Initialize(ground, mesh.GetNode(0), False,
                mesh.GetNode(0).Frame(), mesh.GetNode(0).Frame())
system.Add(link)


system.Add(mesh)





vis_beam = chrono.ChVisualizationFEAmesh(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_CHR_STRESS_VONMISES)
vis_beam.SetColorscaleMinMax(0, 1.0e7)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddAsset(vis_beam)


vis_wire = chrono.ChVisualizationFEAmesh(mesh)
vis_wire.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_NONE)
vis_wire.SetWireframe(True)
vis_wire.SetWireframeThickness(0.002)
mesh.AddAsset(vis_wire)


vis_nodes = chrono.ChVisualizationFEAmesh(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
vis_nodes.SetSymbolsThickness(0.006)
mesh.AddAsset(vis_nodes)




app = chronoirr.ChIrrApp(system, "Beam FEA – PyChrono", chronoirr.dimension2du(1024,768))
app.AddTypicalCamera(chronoirr.vector3df(0.5, 0.3, 0.7), chronoirr.vector3df(0,0,0))  
app.AddTypicalLights()
app.AddSkyBox()
app.AssetBindAll()
app.AssetUpdateAll()




tip_node = mesh.GetNode(N_SEG)               
load_container = chrono.ChLoadContainer()
system.Add(load_container)


class TipLoad(chrono.ChLoaderU):
    def __init__(self, loadable):
        chrono.ChLoaderU.__init__(self, loadable)  
        
    
    def ComputeF(self, U, V, W):
        return chrono.ChVectorD(0, -200, 0)  


load = chrono.ChLoad< chrono.ChLoaderPoint >(
    tip_node, TipLoad(tip_node))
load_container.Add(load)




time_step = 1e-3

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()
    
    system.DoStepDynamics(time_step)