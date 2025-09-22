import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.AddSkyBox()
vis.AddTypicalLights()


mesh = fea.ChBeamSectionEuler()
mesh.SetYoungModulus(210e9)  
mesh.SetIz(0.001)  
mesh.SetIy(0.001)  
mesh.SetJ(0.0001)  
mesh.SetA(0.01)  
mesh.SetDensity(7850)  


nodes = []
for i in range(10):
    node = fea.ChNodeFEAxyz()
    node.SetId(i)
    node.SetPosition(chrono.ChVectorD(i, 0, 0))
    nodes.append(node)


beams = []
for i in range(9):
    beam = fea.ChElementBeamEuler()
    beam.SetId(i)
    beam.SetNodes(nodes[i], nodes[i+1])
    beam.SetSection(mesh)
    beams.append(beam)


sys.Add(mesh)
for node in nodes:
    sys.Add(node)
for beam in beams:
    sys.Add(beam)
vis.Add(sys)
vis.AddTypicalCamera()


for node in nodes:
    node.SetFixed(False)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()