import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.scm as scm


chrono.SetChronoDataPath("path/to/chrono/data/")
myapp = chronoirr.ChIrrApp(__file__, "HMMWV on SCM Deformable Terrain", chronoirr.dimension2du(800, 600))
application = veh.ChPart("military")
myapp.AddTypicalSky()
myapp.AddTypicalLights()
myapp.AddCamera(chrono.ChVectorD(0, 1.5, -5))


terrain = scm.ChScmTerrain()
terrain.Initialize(chrono.ChVectorD(0, -0.5, 0), 10, 10, 0.1)
terrain.SetSoilParameters(0.2, 0.1, 0.01, 0.001, 0.1)
terrain.EnableMovingPatch(True)
terrain.SetSinkageVisualization(True)


vehicle = veh.ChPart("HMMWV")
vehicle.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
vehicle.SetChassisOrientation(chrono.Q_from_AngX(0))
vehicle.SetRigidTireModel()
vehicle.AddVisualizationMeshAll()


vehicle.Initialize()


driver = veh.ChIrrNodeDriver(vehicle)
driver.Initialize()


application.SetTimestep(0.02)
application.SetSolverType(chrono.ChSolver.Type_APGD)
application.SetSolverMaxIterations(100)
application.SetSolverTolerance(1e-4)


application.AddSystem(vehicle)
application.AddSystem(terrain)


while myapp.GetDevice().run():
    myapp.BeginScene()
    myapp.DrawAll()
    application.DoStepDynamics(0.02)
    myapp.EndScene()
    myapp.GetDevice().setFrameRate(50)