import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import os


chrono.ChSystemNSC my_system()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


my_application = chronoirr.ChIrrApp(my_system, 'HMMWV on Complex Terrain', chronoirr.dimension2du(1280, 720))
my_application.AddTypicalSky()
my_application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
my_application.AddLightWithShadow(chrono.ChVectorD(1.0, -1.0, 1.0), chrono.ChVectorD(0, 0, 0), 1, 1, 75, 50, 50)
my_application.SetChaseCamera(chrono.ChVectorD(0, 0, 0.5), 6)
my_application.SetSymbolScale(0.02)


vehicle = veh.ChVehicleSystemNSC()
vehicle.SetChassisBody(veh.ChChassis(chrono.ChVectorD(0, 0, 0.5)))


vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1))
vehicle.SetInitOrientation(chrono.Q_from_AngAxis(chrono.ChMatrix33D(1)))
vehicle.SetEngine(veh.ChEngineSimpleMap())
vehicle.SetDrivetrain(veh.ChDrivetrainSimple())
vehicle.SetTireType(veh.ChTireLugre())


my_system.Add(vehicle)


vehicle.SetChassisVisualizationType(veh.ChChassis::VISUALIZATION_MESH)
for wheel in vehicle.GetWheelBodies():
    wheel.SetVisualizationType(veh.ChWheel::VISUALIZATION_MESH)


terrain = fea.ChTerrainNSC()
terrain.SetSystem(my_system)


patch1 = terrain.AddPatch(chrono.ChVectorD(0, -10, 0), chrono.ChVectorD(10, 10, 0), 'texture1.png')
patch2 = terrain.AddPatch(chrono.ChVectorD(10, -10, 0), chrono.ChVectorD(20, 10, 0), 'texture2.png')


bump_mesh = fea.ChMeshFile()
bump_mesh.LoadMesh(os.path.join(chrono.GetChronoDataPath(), 'terrain', 'bump.obj'))
terrain.AddPatch(chrono.ChVectorD(20, -10, 0), bump_mesh)


heightmap = fea.ChHeightMap()
heightmap.LoadTexture(os.path.join(chrono.GetChronoDataPath(), 'terrain', 'heightmap.png'))
terrain.AddPatch(chrono.ChVectorD(30, -10, 0), heightmap)


driver = veh.ChDriver()
driver.Initialize(vehicle)
driver.SetSteeringController(veh.ChFunction_PID(0.1, 0, 0.01))
driver.SetThrottleController(veh.ChFunction_PID(0.2, 0, 0.01))
driver.SetBrakingController(veh.ChFunction_PID(0.1, 0, 0.01))


my_system.SetTimestepperType(chrono.ChTimestepper.HBM)
my_system.SetSolverType(chrono.ChSolver.SOR)
my_system.SetSolverMaxIterations(100)
my_system.SetSolverTolerance(1e-10)


my_application.AssetBindAll()
my_application.AssetUpdateAll()
my_application.AddLogico("Step", chronoirr.ChIrrAppStepHandler(my_system, 0.01))

while my_application.GetDevice().run():
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()
    my_application.EndScene()