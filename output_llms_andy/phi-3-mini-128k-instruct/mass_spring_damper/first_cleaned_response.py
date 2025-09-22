from pychrono.core import *
from pychrono.vehicle import *
from pychrono.vehicle_dynamic import *
from pychrono.irrlicht import *


chrono_conf = Chrono_Config()
chrono_conf.SetPhysicsEngineSync(true)
chrono_conf.SetVerbosityLevelStandAlone(Chrono_LogVerbosityLevel_Error)
chrono_conf.Initialize()


vehicle = Vehicle()


mass = VehicleBody()
mass.SetPos(Vector3(0, 0, 0))


spring_damper = ChLinkTSDA()
spring_damper.SetDamping(1.0)
spring_damper.SetStiffness(100.0)


ground = ChBody()
ground.SetPos(Vector3(0, -1, 0))
spring_damper.AttachTo(mass, ChVector3D(0, 0, 0))


mass.SetVisualShape(ChVisualShapeAppearance(), ChVisualShapePrimitiveBox(ChVector3D(0.1, 0.1, 0.1))
spring_damper.SetVisualShape(ChVisualShapeAppearance(), ChVisualShapePrimitiveBox(ChVector3D(0.05, 0.05, 0.05))


vehicle.Add(mass)
vehicle.Add(spring_damper)


application = IrrlichtApplication(chrono_conf, 'MassSpringDamperSimulation')
application.AddSpatialManager(ChSpatialManager())
application.AddDevice(ChSceneManager())
application.AddDevice(ChWindowManager())
application.SetWindowSize(1280, 720)
application.SetWindowTitle('Mass Spring Damper Simulation')


camera = ChCameraManager()
camera.SetProjectionType(ChProjectionType_Perspective)
camera.SetPosition(ChVector3D(2, 2, 2))
camera.SetTarget(ChVector3D(0, 0, 0))
camera.SetFieldOfView(45)


scene_manager = application.GetSceneManager()


scene_manager.Add(mass)
scene_manager.Add(spring_damper)


chrono_conf.SetSolverType(ChSolver_Exa(1e-6, 1e-6, 1e-6, 1e-6, 1e-6)
chrono_conf.SetSolverTolerance(1e-6)
chrono_conf.SetSolverMaxIterations(1000)
chrono_conf.SetSolverRelativeTolerance(1e-6)
chrono_conf.SetSolverAbsoluteTolerance(1e-6)
chrono_conf.SetSolverMaxIterations(1000)


while application.Run() == Application::Running:
    application.DoStep(0.01, True)
    scene_manager.DrawAll()


chrono_conf.Shutdown()