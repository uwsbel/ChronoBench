import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')


my_system = chrono.ChSystemNSC()


beam = fea.ChElementBeamANCF()



def custom_motor_function(time):
    
    pass


motor = chrono.ChFunction_Recorder()
motor.AddPoint(0, 0)
motor.AddPoint(1, 1)
motor.AddPoint(2, 0)

motor.SetCustomFunction(custom_motor_function)








app = irr.ChIrrApp(my_system, 'Beam Buckling Simulation', irr.dimension2du(1024, 768))
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0.0, 0.5, -1.0))
app.AddTypicalLights()
app.AssetBindAll()
app.AssetUpdateAll()


my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverMaxIterations(100)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()