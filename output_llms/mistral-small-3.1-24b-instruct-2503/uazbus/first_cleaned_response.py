import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.postprocess as postprocess


chrono.SetChronoDataPath('../data/')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = chrono.ChBodyEasyBox(50, 0.5, 50, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -0.25, 0))
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurface().SetFriction(0.9)
terrain.GetMaterialSurface().SetRestitution(0.2)
system.Add(terrain)


vehicle = veh.UAZBus(system)
vehicle.Initialize(chrono.ChVectorD(0, 1, 0), chrono.ChQuaternionD(1, 0, 0, 0), chrono.ChVectorD(0, 0, 0))


driver = veh.ChDriver(vehicle.GetVehicle())
driver.SetSteeringController(veh.ChSteeringControllerPID(0.5, 0, 0.1))
driver.SetThrottleController(veh.ChThrottleControllerPID(0.2, 0, 0.01))
driver.SetBrakingController(veh.ChBrakingControllerPID(0.5, 0, 0.1))


vis = chronoirr.ChIrrApp()
vis.AddTypicalSky()
vis.AddTypicalLogo()
vis.AddTypicalCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLight(chrono.ChVectorD(1, 1, 1), chrono.ChColorf(1, 1, 1), 500)
vis.AssetBindAll()
vis.AssetUpdateAll()


vis.AddVisualSystem(vehicle)
vis.AddVisualSystem(terrain)


step_size = 0.01
max_time = 10.0
time = 0.0


postprocess.Init()
postprocess.SetChronoDataPath('../data/')
postprocess.SetOutputDir('UAZBUS_SIMULATION')
postprocess.AddSystem(system)

while time < max_time:
    time += step_size
    system.DoStepDynamics(step_size)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Synchronize(time)
    driver.Advance(step_size)

    
    postprocess.DoStepDynamics(step_size)


postprocess.WritePostProcessors(time)
postprocess.Close()


vis.Exit()