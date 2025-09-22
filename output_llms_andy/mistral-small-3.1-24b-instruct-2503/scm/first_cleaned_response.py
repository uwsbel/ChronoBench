import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('../data/')


system = chrono.ChSystemNSC()


vehicle = veh.HMMWV()
vehicle.SetChassisBodyVisualizationType(veh.ChVisualizationType::MESH)
vehicle.SetChassisBodyTextureFilename(veh.ChContactMethod::SMC)
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1))
vehicle.SetInitOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetTireType(veh.ChTireModelType::RIGID)
vehicle.Initialize(system)


terrain = veh.SCMDeformableTerrain()
terrain.SetChassisBodyVisualizationType(veh.ChVisualizationType::MESH)
terrain.SetChassisBodyTextureFilename(veh.ChContactMethod::SMC)
terrain.SetSoilParameters(0.1, 0.3, 0.01, 0.001, 0.0, 0.0)
terrain.SetMovingPatch(True)
terrain.SetSinkageVisualization(True)
terrain.Initialize(system)


driver = veh.ChDriver()
driver.SetVehicle(vehicle)
driver.SetSteeringControllerType(veh.ChDriver::STEERING_CONTROLLER_TYPE_PI)
driver.SetSteeringControllerGain(1.0)
driver.SetThrottleControllerType(veh.ChDriver::THROTTLE_CONTROLLER_TYPE_PI)
driver.SetThrottleControllerGain(1.0)
driver.SetBrakingControllerType(veh.ChDriver::BRAKING_CONTROLLER_TYPE_PI)
driver.SetBrakingControllerGain(1.0)


visualization = chronoirr.ChIrrApp(system, 'HMMWV on SCM Terrain', chronoirr.dimension2du(1280, 720))
visualization.AddTypicalSky()
visualization.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_small.png'))
visualization.AddLightWithShadow(chrono.ChVectorD(1.0, -1.0, 1.0), chrono.ChVectorD(0.7, 0.7, 0.9), 1.0, 4.0, 5.0, 40, 512)
visualization.SetChaseCamera(chrono.ChVectorD(0, 0, 1), 6, 0.5)
visualization.SetSymbolScale(0.02)


visualization.AddVisualSystem(vehicle.GetChassisBody())
visualization.AddVisualSystem(terrain.GetChassisBody())


system.SetTimestepperType(chrono.ChTimestepper::Type::HHT)
system.SetMaxPenetrationRecoverySpeed(2.0)
system.SetMaxIterationsSMC(100)
system.SetIterLCPmaxItersSpeed(100)
system.SetToleranceForWarmStarting(1e-10)
system.SetMaxPenetrationRecoverySpeed(1.0)


visualization.SetTimestep(1.0 / 50)
visualization.SetTryRealTime(True)


while visualization.Run():
    system.DoStepDynamics(1.0 / 50)
    visualization.Render()