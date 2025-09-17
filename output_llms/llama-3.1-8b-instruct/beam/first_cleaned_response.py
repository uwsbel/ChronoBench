import chrono as ch
import chrono-python-irrlicht as cpi


ch.InitChronics()


sys = ch.ChSystem()



n1 = ch.ChNodeBeam()
n1.SetPos(ch.Vec(0, 0, 0))
n2 = ch.ChNodeBeam()
n2.SetPos(ch.Vec(2, 0, 0))
n3 = ch.ChNodeBeam()
n3.SetPos(ch.Vec(2, 2, 0))


beam1 = ch.ChBeam()
beam1.SetNode1(n1)
beam1.SetNode2(n2)
beam1.SetCrossSection(ch.ChBeamCrossSectionBox(0.1, 0.1))
beam1.SetMaterialDef(ch.ChMaterialSurfaceElastic(1e9, 1e9, 0.3))
sys.Add(beam1)

beam2 = ch.ChBeam()
beam2.SetNode1(n2)
beam2.SetNode2(n3)
beam2.SetCrossSection(ch.ChBeamCrossSectionBox(0.1, 0.1))
beam2.SetMaterialDef(ch.ChMaterialSurfaceElastic(1e9, 1e9, 0.3))
sys.Add(beam2)


irrlicht = cpi.ChIrrApp(sys, "PyChrono Beam Simulation", cpi.dimension2D)
irrlicht.AddLine(n1.GetPos(), n2.GetPos(), cpi.Color(1, 0, 0))
irrlicht.AddLine(n2.GetPos(), n3.GetPos(), cpi.Color(0, 1, 0))


irrlicht.Run()