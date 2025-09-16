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
application.AddLogo(0.5, 0.5)


application.SetWindowSize(800, 600)
application.SetWindowTitle('PyChrono Beam Simulation')


application.AddSystem(system)


application.SetCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


dt = 0.01
t_end = 10


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    system.DoStepDynamics(dt)

    
    if system.GetChTime() > t_end:
        break


application.GetDevice().closeDevice()