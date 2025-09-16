import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


system = ch.ChSystem()


mesh = ch.ChBeamBodyEasy()


beam_section = ch.ChBeamSectionEuler()
beam_section.SetYoungModulus(2e9)  
beam_section.SetArea(0.01)  
beam_section.SetInertiaY(0.0001)  
beam_section.SetInertiaZ(0.0001)  


node_A = ch.ChNodeBase(beam_section)
node_A.SetPos([0, 0, 0])
node_B = ch.ChNodeBase(beam_section)
node_B.SetPos([1, 0, 0])


system.Add(node_A)
system.Add(node_B)


beam = ch.ChBeamEuler(beam_section, node_A, node_B)
beam.SetLength(1.0)


system.Add(beam)


irr.ChIrrApp(system, 'Beam Finite Element Simulation').Run()