import PyChrono as chrono
import PyChrono.Irrlicht as chronoirr
import PyChrono.Vehicle as vehicle
import PyChrono.Postprocess as postprocess


chrono.Init_Chrono()


vehicle_system = vehicle.ChVehicle()
vehicle_system.Initialize(chrono.GetChronoDataFile('vehicle/hmmwv/hmmwv.tire'), chrono.GetChronoDataFile('vehicle/hmmwv/hmmwv.chassis'))


vehicle_system.SetChassisPosition(chrono.ChVectorD(0, 0, 0.5))
vehicle_system.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))


vehicle_system.SetContactMethod(vehicle.ChContactMethod_FEA)
vehicle_system.SetChassisVisualizationType(vehicle.ChVehicleVisualizationType_PRIMITIVES)


terrain = chrono.ChTerrain()
terrain.SetTexture(chrono.GetChronoDataFile('vehicle/textures/tile4.jpg'), 200, 200)
terrain.Initialize(vehicle_system.GetSystem())


terrain.Set_tx(200)
terrain.Set_tz(200)


driver = vehicle.ChIrrGuiDriver(vehicle_system, terrain)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(-0.2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('ARTcar Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, -30, 2))
vis.AddTypicalLights()


vis.Add(vehicle_system.GetChassis())
vis.Add(terrain)


sim_time = 0
step_size = 0.01
while vis.Run():
    sim_time += step_size
    vehicle_system.Synchronize(step_size)
    terrain.Synchronize(vehicle_system.GetChassis(), vehicle_system.GetSpindles(), step_size)
    driver.Update(step_size)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    chrono.Print("Simulation time: ", sim_time)
    chrono.Print("Vehicle speed: ", vehicle_system.GetChassisSpeed())
    chrono.Print("Steering angle: ", vehicle_system.GetSteeringValue())
    chrono.Print("Throttle value: ", vehicle_system.GetThrottleValue())
    chrono.Print("Brake value: ", vehicle_system.GetBrakeValue())
    chrono.Print("")
    chrono.Sleep(1 / 50)


chrono.Chrono_Exit()