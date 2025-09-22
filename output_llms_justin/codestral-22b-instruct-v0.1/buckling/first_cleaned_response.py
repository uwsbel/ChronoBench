import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()



beam = fea.ChElementBeamANCF()






my_system.Add(beam)




beam.SetPos(chrono.ChVectorD(0, 0, 0))









application = irr.ChIrrApp(my_system, 'Beam Buckling Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(irr.vector3df(0, 3, -6))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverMaxIterations(100)
my_system.SetSolverWarmStarting(True)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)


application.SetTimestep(0.01)
application.SetTryRealtime(True)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()