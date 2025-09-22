import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = irr.ChIrrApp(system, 'Beam Finite Element Simulation', irr.dimension2d(1024, 768))
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0, 0, -10))
application.AddTypicalLights()



nodes_positions = [
    (0, 0, 0),
    (1, 0, 0),
    (2, 0, 0),
    (3, 0, 0),
    (4, 0, 0),
]


nodes = []
for pos in nodes_positions:
    node = chrono.ChNodeFEAxyz(chrono.ChFrameD(chrono.ChVectorD(*pos)))
    system.Add(node)
    nodes.append(node)



beams = []
for i in range(len(nodes) - 1):
    beam_element = chrono.ChElementBeam()
    beam_element.SetNodes(nodes[i], nodes[i + 1])
    
    beam_length = (chrono.ChVectorD(*nodes_positions[i + 1]) - chrono.ChVectorD(*nodes_positions[i])).Length()
    
    beam_material = chrono.ChMaterialBeam()
    
    beam_material.SetYoungModulus(2e11)  
    beam_material.SetDensity(7850)       
    beam_material.SetBeamR(0.01)         
    beam_material.SetBeamA(chrono.ChBezierCurve())  
    
    beam_element.SetMaterial(beam_material)
    beam_element.SetInertias(chrono.ChVectorD(1e-6, 1e-6, 1e-6))
    
    
    if not hasattr(system, 'mesh'):
        system.mesh = chrono.ChMesh()
        system.Add(system.mesh)
    system.mesh.AddElement(beam_element)


system.mesh.Build()
system.mesh.SetAutomaticTuning(True)



application.AssetBindAll()
application.AssetUpdateAll()



nodes[0].SetFixed(True)


application.SetTimestep(0.01)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()