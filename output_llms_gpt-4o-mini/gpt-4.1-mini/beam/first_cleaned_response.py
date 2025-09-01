import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  


system = chrono.ChSystemSMC()



mesh = fea.ChMesh()


beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetDiameter(0.02)  
beam_section.SetYoungModulus(2.1e11)  
beam_section.SetGwithPoissonRatio(8.1e10, 0.3)  
beam_section.SetBeamRaleyghDamping(0.02)  




num_nodes = 5
length = 1.0  
node_spacing = length / (num_nodes - 1)

nodes = []
for i in range(num_nodes):
    pos = chrono.ChVectorD(i * node_spacing, 0, 0)
    node = fea.ChNodeFEAxyzrot(pos)
    node.SetMass(0.1)  
    
    if i == 0:
        node.SetFixed(True)
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_nodes - 1):
    beam_element = fea.ChElementBeamEuler()
    beam_element.SetNodes(nodes[i], nodes[i + 1])
    beam_element.SetSection(beam_section)
    beam_element.SetDensity(7850)  
    mesh.AddElement(beam_element)


system.Add(mesh)



mesh.AddVisualization(fea.ChVisualization.Type_BEAMS)
mesh.AddVisualization(fea.ChVisualization.Type_NODE_CSYS)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


app = chronoirr.ChIrrApp(system, "Beam Finite Element Demo", chronoirr.dimension2du(800, 600))
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(1.5, 1.5, 1.5))

app.AssetBindAll()
app.AssetUpdateAll()

app.SetTimestep(1e-3)
app.SetTryRealtime(True)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()