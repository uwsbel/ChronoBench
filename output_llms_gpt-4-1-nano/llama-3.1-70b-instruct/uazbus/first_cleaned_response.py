import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()


uazbus = veh.ChUAZBUS()
uazbus.SetChassisFixed(False)
uazbus.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
uazbus.SetInitRotation(chrono.ChQuaternionD(1, 0, 0, 0))
mysystem.Add(uazbus.GetChassisBody())


terrain = chrono.ChBodyEasyBox(mysystem, 10, 10, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, 0, -0.5))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterial().SetFriction(0.9)
terrain.GetMaterial().SetRestitution(0.1)
mysystem.Add(terrain)


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(uazbus)
driver.SetSteeringMode(veh.SteeringMode::STEERING_MODE_DEGREES)
driver.SetSteeringLimit(30 * chrono.CH_C_DEG_TO_RAD)
driver.SetAccelerationMode(veh.AccelerationMode::ACCELERATION_MODE_TORQUE)
driver.SetMaxAcceleration(1000)
driver.SetBrakingMode(veh.BrakingMode::BRAKING_MODE_TORQUE)
driver.SetMaxBrakingTorque(1000)


visual_system = chronoirr.ChVisualSystemIrrlicht()
visual_system.SetWindowSize(800, 600)
visual_system.SetWindowTitle('UAZBUS Simulation')
visual_system.SetCamera(chrono.ChVectorD(0, 0, 1.5), chrono.ChVectorD(0, 0, 0))
visual_system.SetCameraUp(chrono.ChVectorD(0, 1, 0))
visual_system.SetCameraSpeed(0.5)
visual_system.SetCameraRotSpeed(0.1)
visual_system.AddSkyBox()
visual_system.AddTypicalLights()


visual_system.Add(uazbus.GetChassisBody())
visual_system.Add(terrain)


mysystem.SetSolverType(chrono.ChSolver.Type_PSSOR)
mysystem.SetSolverMaxIterations(100)
mysystem.SetSolverTolerance(1e-6)
mysystem.SetTimestep(0.01)


while visual_system.Run():
    
    mysystem.DoStepDynamics(0.01)
    visual_system.BeginScene()
    visual_system.DrawAll()
    visual_system.EndScene()
    
    driver.Advance(0.01)
    
    uazbus.GetChassisBody().Update()
    terrain.Update()