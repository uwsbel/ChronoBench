import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
system.SetStep(0.01)




beam_length = 10
num_elements = 20
element_length = beam_length / num_elements


material = fea.ChMaterialCableANCF(system)
material.SetDensity(7850)  
material.SetYoungModulus(200e9)  
material.SetBeamRaleyghDamping(0.001)


section = fea.ChBeamSectionCable(material, 0.01)  


element = fea.ChElementCableANCF()
element.SetSection(section)


mesh = fea.ChMesh()
nodes = []
for i in range(num_elements + 1):
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * element_length, 0, 0)))
    nodes.append(node)
    mesh.AddNode(node)

for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(section)
    mesh.AddElement(element)


nodes[0].SetFixed(True)


for node in nodes:
    node.SetMass(0.1)  


system.Add(mesh)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(beam_length / 2, beam_length / 4, -beam_length), chrono.ChVectorD(beam_length / 2, 0, 0))
vis.AddTypicalLights()




while vis.Run():
    system.DoStepDynamics(system.GetStep())

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()


vis.Close()