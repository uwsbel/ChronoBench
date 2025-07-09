import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np


out_dir = "SENSOR_OUTPUT"


if not chrono.QDIR(out_dir):
    chrono.QDIR.mkdir(out_dir)


chrono.SetChronoDataPath("/path/to/chrono/data")


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleX(chrono.CH_C_PI / 20)))
vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetSteeringType(veh.SteeringTypeWV)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChContactMaterialSMC()
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(vehicle.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 5), vehicle.GetChassis().GetPos())
vis.AddTypicalLights()


imu_data = veh.SensorIMU(vehicle.GetChassis(), 0, chrono.ChFrame())
gps_data = veh.SensorGPS(vehicle.GetChassis(), 0, chrono.ChFrame())


time_step = 1e-3
time_end = 10
render_step = 1 / 60


time = 0
render_time = 0
while vis.Run() and time < time_end:
    time = vehicle.GetSystem().GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    vis.Synchronize("HMMWV Visualization", driver_inputs)
    
    
    imu_data.Update(time)
    gps_data.Update(time)
    
    
    vehicle.Advance(time_step)
    terrain.Advance(time_step)
    vis.Advance(time_step)
    
    
    if time >= render_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_time += render_step
    
    
    if time % 1 == 0:
        print(f"Time: {time}, Vehicle Mass: {vehicle.GetVehicle().GetMass()}")


del vis