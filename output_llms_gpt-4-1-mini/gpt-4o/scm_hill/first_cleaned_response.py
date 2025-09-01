import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.postprocess as post
import pychrono.utils as utils


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
vehicle.SetDriveType(veh.DrivelineType_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
terrain.Initialize(terrain.GetPlane(), 100, 100, 0.1)


terrain.SetSoilParameters(2e6, 0, 1e4, 3e4, 0.01, 0.01)
terrain.SetBulldozingFlow(True)
terrain.SetBulldozingParameters(55, 1, 5, 6)
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000)


driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), veh.StraightLinePath(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(100, 0, 0)), "path", 10.0)
driver.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Vehicle Simulation')
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(30, 30, 100), 290, chrono.ChColor(0.7, 0.7, 0.7))
vis.AttachVehicle(vehicle.GetVehicle())


step_size = 1e-3
time_end = 20
realtime_timer = chrono.ChRealtimeStepTimer()


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    if time >= time_end:
        break

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    vis.Synchronize(time, driver.GetInputs())

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

print("Simulation finished.")