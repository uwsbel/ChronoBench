import pychrono.core as chrono
import pychrono.irr as chronoirr


chrono.ChSystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


system = chrono.ChSystem()


beam_length = 1.0
num_beams = 10
beam_radius = 0.05


nodes = []


for i in range(num_beams):
    
    node = chrono.ChNodeFEAxyz(chrono.ChVectorD(i * beam_length, 0, 0))
    system.Add(node)
    nodes.append(node)


for i in range(num_beams - 1):
    beam = chrono.ChElementBeamEuler()
    beam.SetNodes(nodes[i], nodes[i + 1])
    beam.SetSection(chrono.ChBeamSectionEuler(beam_radius))
    beam.SetYoungModulus(2.1e11)  
    beam.SetDensity(7800)  
    system.Add(beam)


irr_app = chronoirr.ChIrrApp(system, "Beam Finite Element Simulation", chronoirr.dimension2d(800, 600))
irr_app.AddLogo()
irr_app.AddSkyBox()
irr_app.AddCamera(chronoirr.vector3df(5, 5, -10), chronoirr.vector3df(0, 0, 0))
irr_app.AddTypicalLights()
irr_app.AssetBindAll()
irr_app.AssetUpdateAll()


while True:
    
    system.DoStepDynamics(0.01)

    
    irr_app.DrawAll()

    
    if irr_app.GetDevice().run():
        irr_app.GetDevice().drop()
        break