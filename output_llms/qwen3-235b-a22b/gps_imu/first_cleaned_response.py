import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sensor


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.HMMWV_Full()
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0), chrono.QUNIT))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
print("Vehicle mass: ", vehicle.GetMass())


terrain = chrono.ChPlaneTerrain()
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetPlane(chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 0))
terrain.GetMaterialSurfaceNSC().SetFriction(0.9)
terrain.GetMaterialSurfaceNSC().SetRestitution(0.01)
system.Add(terrain)


driver = veh.ChDriver(vehicle)
driver.Initialize()


application = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


imu = sensor.ChIMU()
imu.Attach(vehicle.GetChassisBody())
system.Add(imu)

gps = sensor.ChGPS()
gps.Attach(vehicle.GetChassisBody())
gps.SetReference(chrono.ChVectorD(0, 0, 0))  
system.Add(gps)


step_size = 1e-3
time = 0


while application.GetDevice().run():
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    
    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    
    
    imu_data = imu.GetSensorData()
    gps_data = gps.GetSensorData()
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    system.DoStepDynamics(step_size)
    time += step_size