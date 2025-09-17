import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.terrain as ter
import pychrono.solver as solver


chrono.SetChronoDataPath('../../data/')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.HMMWV()
vehicle.InitSystem(system, veh.ChVehicleModelType.VEHICLE_3D)
vehicle.SetChassisVisualizationType(veh.ChVehicleVisualizationType.MESH)


vehicle.SetInitPosition(chrono.ChVectorD(0, 1, 0))
vehicle.SetInitOrientation(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))


terrain = ter.ChTerrain()
terrain.Initialize(system, "terrain/scmtest")
terrain.SetContactMaterialProperties(0.3, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0)


terrain.EnableMovingPatch(vehicle.GetChassisBody(), 3, 3, 0.2, 0.2)


terrain.EnableSinkagePlotting()


driver = veh.ChDriver()
driver.Initialize(vehicle.GetVehicle(), system)
driver.SetSteeringController(veh.ChDriver::STEERING_CONTROLLER_SIMPLE)
driver.SetThrottleController(veh.ChDriver::THROTTLE_CONTROLLER_SIMPLE)
driver.SetBrakingController(veh.ChDriver::BRAKING_CONTROLLER_SIMPLE)


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('HMMWV on SCM Terrain')
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalLights()
visualization.AddLogo()
visualization.AddLightWithShadow(chrono.ChVectorD(1.0, -1.0, 1.0), chrono.ChVectorD(0, 0, 0), 1, 1.5, 30)


system.SetTimestepperType(chrono.ChTimestepperType.HYBRID)
system.SetSolverType(solver.ChSolverType.BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-10)
system.SetStep(0.01)


run_time = 10  
current_time = 0
while current_time < run_time:
    visualization.Render()
    system.DoStepDynamics(0.01)
    current_time += 0.01


visualization.Close()