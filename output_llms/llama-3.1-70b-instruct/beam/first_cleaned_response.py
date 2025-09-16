import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.SetChronoDataPath('./')


system = chrono.ChSystemNSC()


mesh = fea.ChBeamSectionEulerBernoulliRectangular(0.1, 0.05, 1000, 1000, 1000, 1000)


node1 = fea.ChNodeFEAxyz(chrono.ChVectorD(0, 0, 0))
node2 = fea.ChNodeFEAxyz(chrono.ChVectorD(1, 0, 0))


beam = fea.ChElementBeamEulerBernoulli(node1, node2, mesh)


system.Add(beam)


application = chronoirr.ChVisualSystemIrrlicht()
application.AddTypicalLights()
application.AddSkyBox()
application.AddTypicalCamera(chronoirr.vector3df(1, 1, 1))


application.AddSystem(system)


application.SetTimestep(0.01)
application.SetTryRealtime(True)


while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()