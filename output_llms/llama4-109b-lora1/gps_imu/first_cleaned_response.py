import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import pychrono.vehicle as veh


chrono.SetChairsThreading(True)
chrono.ChSetDataFilePath(chrono.GetChronoDataPath() + 'vehicle/hmmwv/')


vehicle = veh.HMMWV()
terrain = veh.RigidTerrain()
driver = veh.HMMWV_PWSAIDriver()


vehicle.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.1), chrono.ChQuaterniond(1, 0, 0, 0)))


terrain.Initialize(vehicle.GetSystem())


driver.Initialize(vehicle.GetVehicle(), terrain)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachVehicle(vehicle.GetVehicle())
vis.SetWindowTitle('HMMWV Sensor Test')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)


imu = sens.ChIMU(chrono.ChFrameMovingd(chrono.ChVector3d(0, 0, 1.75), chrono.ChQuaterniond(1, 0, 0, 0)))
gps = sens.ChGPS(chrono.ChFrameMovingd(chrono.ChVector3d(0, 0, 1.75), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.GetSystem().AddSensor(imu)
vehicle.GetSystem().AddSensor(gps)


step_size = 1e-3
t_end = 100
render_step_size = 1 / 60
render_steps = int(render_step_size / step_size)
render_counter = 0


vehicle.GetSystem().Initialize()
terrain.Initialize(vehicle.GetSystem())
driver.Initialize(vehicle.GetVehicle(), terrain)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChCoordsysd(chrono.ChVector3d(2, -6, 1.5), chrono.ChQuaterniond(1, 0, 0, 0)))
vis.AddLightDirectional()


while vis.Run() and vehicle.GetSystem().GetChTime() < t_end:
    
    driver.Synchronize(time=vehicle.GetSystem().GetChTime())
    terrain.Synchronize(vehicle.GetSystem())
    vis.Synchronize(vehicle.GetVehicle(), driver.GetSteering())
    vehicle.Synchronize(time=vehicle.GetSystem().GetChTime(), driver=driver, terrain=terrain)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    vehicle.GetSystem().Update()
    imu.Update()
    gps.Update()

    
    print(vehicle.GetVehicle().GetMass())
    vis.Render()

    
    render_counter += 1
    if render_counter == render_steps:
        vis.Render()
        render_counter = 0