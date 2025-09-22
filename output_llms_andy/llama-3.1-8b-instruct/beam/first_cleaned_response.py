import chrono as ch
import chrono-python-irrlicht as cpi


ch.InitChronics()


sys = ch.ChSystem()



node1 = ch.ChNodeBeam()
node1.SetPos(ch.Vec3(0, 0, 0))
node2 = ch.ChNodeBeam()
node2.SetPos(ch.Vec3(5, 0, 0))
node3 = ch.ChNodeBeam()
node3.SetPos(ch.Vec3(5, 5, 0))


beam1 = ch.ChBeam(node1, node2)
beam1.SetCrossSection(ch.ChBeamCrossSectionBox(0.1, 0.1))
beam1.SetElasticity(200e9)
beam1.SetDensity(7800)

beam2 = ch.ChBeam(node2, node3)
beam2.SetCrossSection(ch.ChBeamCrossSectionBox(0.1, 0.1))
beam2.SetElasticity(200e9)
beam2.SetDensity(7800)


sys.Add(node1)
sys.Add(node2)
sys.Add(node3)
sys.Add(beam1)
sys.Add(beam2)


sys.SetGravity(ch.Vec3(0, -9.81, 0))


vis = cpi.ChIrrApp(sys, "Beam Finite Elements Simulation")


vis.Run()