import chrono
import irrlich
from chrono import ChSystemIrrlicht
from chrono.vehicle import ChVehicleModelGator, ChTireModelTMeasy


chrono.SetChDir(chrono.GetChronoDataFile('..'))
my_system = chrono.ChSystem()



terrain = chrono.ChBodyEasyBox(100, 0.1, 100, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
my_system.AddBody(terrain)


vehicle = ChVehicleModelGator()
vehicle.SetChassisVisualizationType(chrono.ChVisualizationType.MESH)
vehicle.SetSuspensionVisualizationType(chrono.ChVisualizationType.MESH)
vehicle.SetTireVisualizationType(chrono.ChVisualizationType.MESH)
vehicle.SetTireType(ChTireModelTMeasy())
vehicle.Initialize(my_system, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
vehicle.SetContactMethod(chrono.ChContactMethod.NSC)
my_system.Add(vehicle)


vehicle.SetSteering(-0.5)
vehicle.SetThrottle(0.5)
vehicle.SetBrake(0)


my_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))
my_system.SetSolverType(chrono.ChSolver.Type.CSR)
my_system.SetSolverMinIterations(10)
my_system.SetSolverMaxIterations(50)
my_system.SetSolverAccuracy(1e-6)

my_application = ChSystemIrrlicht()
my_application.SetChronoSystem(my_system)
my_application.SetWindowSize(1280, 720)
my_application.SetWindowTitle('Gator Simulation')
my_application.AddTypicalSky()
my_application.AddTypicalLogo()
my_application.AddCamera(chrono.ChVectorD(0, 10, 10), chrono.ChVectorD(0, 0, 0))


def my_custom_update(vehicle, system):
    vehicle.Synchronize(system, chrono.ChFrameD(chrono.Q_from_AngX(chrono.CH_C_PI_2)))
    vehicle.SetSteering(my_application.GetDevice().getJoystickAxis(0) * 0.5 - 0.5)
    vehicle.SetThrottle(my_application.GetDevice().getJoystickAxis(1))
    vehicle.SetBrake(my_application.GetDevice().getJoystickAxis(2))

my_application.AssetBindAll()
my_application.SetCustomUpdateFunction(my_custom_update)
my_application.SetTimestep(1./50)


while my_application.GetDevice().run():
    my_application.DrawAll()
    my_application.BeginScene()
    my_system.DoStepDynamics(my_application.GetTimestep())
    my_application.EndScene()