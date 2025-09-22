import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))





num_nodes = 5
node_spacing = 0.2  
beam_length = node_spacing
beam_radius = 0.01
beam_density = 7800  
beam_material = chrono.ChMaterialBeam()


nodes = []
beams = []


for i in range(num_nodes):
    node_position = chrono.ChVectorD(i * node_spacing, 0, 0)
    node = chrono.ChNodeFEAxyz(chrono.ChVectorD(node_position))
    node.SetMass(beam_density * (4/3)*3.1415*beam_radius**3)  
    system.Add(node)
    nodes.append(node)


for i in range(num_nodes - 1):
    node_a = nodes[i]
    node_b = nodes[i + 1]

    
    section = chrono.ChBeamSectionCircular()
    section.SetRadius(beam_radius)
    section.SetMaterial(beam_material)

    
    beam = chrono.ChElementBeam()
    beam.SetNodes(node_a, node_b)
    beam.SetSection(section)
    beam.SetDensity(beam_density)
    system.Add(beam)
    beams.append(beam)


fixed_node = nodes[0]
fixed_node.SetFixed(True)


system.Initialize()


application = irr.ChIrrApp(system, 'Beam Finite Element Simulation', irr.dimension2d(1024, 768))
application.GetSceneManager().AddCamera(irr.ChCameraSceneNode(application.GetSceneManager(), 
                                    irr.ChVectorD(0.5, 0.2, 1.0),  
                                    irr.ChVectorD(0.5, 0, 0)))   
application.AddTypicalLights()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()