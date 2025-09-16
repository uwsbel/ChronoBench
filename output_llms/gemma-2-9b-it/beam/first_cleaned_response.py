import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("..")  


system = chrono.ChSystemNSC()



beam_material = chrono.ChMaterialSurfaceNSC()
beam_material.SetFriction(0.5, 0.5)
beam_material.SetRestitution(0.0)

beam_cross_section = chrono.ChBeamSectionSolidCircle(0.05)  


node1 = chrono.ChNodeBodyEasyMotion(system)
node1.SetPos(chrono.ChVectorD(0, 0, 0))

node2 = chrono.ChNodeBodyEasyMotion(system)
node2.SetPos(chrono.ChVectorD(1, 0, 0))


beam1 = chrono.ChBeam(beam_material, beam_cross_section)
beam1.SetName("Beam1")
beam1.SetBody(node1)
beam1.SetOtherBody(node2)


system.Add(beam1)


vis = chronoirr.ChIrrApp(system, 'Beam Simulation')
vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


vis.Run()