import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemSMC()


initLoc = chrono.ChVectorD(0, 0, 1.0)
initRot = chrono.QUNIT
vehicle = veh.HMMWV_Full(system)
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.Initialize()


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.CSYSNORM, 200, 200)
patch.SetContactFrictionCoefficient(0.9)
patch.SetRestitutionCoefficient(0.01)
patch.SetMaterialSurface(chrono.ChMaterialSurfaceSMC())
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


path = veh.StraightLinePath(chrono.ChVectorD(0, 0, 1.0), chrono.ChVectorD(100, 0, 1.0), 1)
target_speed = 10.0  
driver = veh.ChPathFollowerDriver(vehicle, path, "my_path", target_speed)
driver.Initialize()


vis = veh.ChWheeledVehicleIrrApp(vehicle, 'HMMWV Sensor Demo', irr.dimension2du(1024,768))
vis.SetSkyBox()
vis.AddTypicalLights()
vis.AddTypicalLogo()
vis.AddTypicalCamera(chrono.ChVectorD(0, 2, 1.75), chrono.ChVectorD(0, 0, 1.0))
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.AssetBindAll()
vis.AssetUpdateAll()


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChVectorF(1, 1, 1), 500)


imu_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.2))
imu = sens.ChIMUSensor(
    vehicle.GetChassisBody(),  
    imu_offset,                
    100.0,                     
    chrono.ChVectorD(0),       
    sens.IMUNoiseModel_NONE
)
imu.PushFilter(sens.ChFilterIMUUpdate())
manager.AddSensor(imu)


gps_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.5))
gps = sens.ChGPSSensor(
    vehicle.GetChassisBody(),
    gps_offset,
    10.0,                      
    chrono.ChVectorD(0),       
    sens.GPSNoiseModel_NONE
)
gps.PushFilter(sens.ChFilterGPSUpdate())
manager.AddSensor(gps)


step_size = 2e-3
render_steps = int(1.0 / (60 * step_size))  
sim_time = 0
frame = 0

print("Vehicle mass:", vehicle.GetVehicleMass(), "kg")

while vis.Run():
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    vehicle.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize("HMMWV Demo", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    manager.Update()

    
    imu_data = imu.GetMostRecentBuffer()
    if imu_data:
        acc = imu_data.Acc
        ang_vel = imu_data.AngVel
        print(f"[{sim_time:.2f}s] IMU Acc: {acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f} | AngVel: {ang_vel.x:.2f}, {ang_vel.y:.2f}, {ang_vel.z:.2f}")

    
    gps_data = gps.GetMostRecentBuffer()
    if gps_data:
        pos = gps_data.Position
        print(f"[{sim_time:.2f}s] GPS Position: {pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}")

    
    if frame % render_steps == 0:
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()

    
    sim_time += step_size
    frame += 1

    
    if sim_time > 10.0:
        break


print("Final vehicle mass:", vehicle.GetVehicleMass(), "kg")