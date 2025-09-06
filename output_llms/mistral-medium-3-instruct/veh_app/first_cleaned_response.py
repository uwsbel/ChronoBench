import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


vehicle_system = veh.VehicleSystem()
vehicle_system.SetChTimeStep(1e-3)  


terrain = veh.RigidTerrain(vehicle_system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)


patch = terrain.AddPatch(chrono.ChCoordinatesysD(chrono.VECT_X, chrono.QUNIT),
                         100, 100,  
                         0, 0,      
                         True)      


gator = veh.Gator(vehicle_system)
gator.SetContactFrictionCoefficient(0.9)
gator.SetContactRestitutionCoefficient(0.1)
gator.SetContactMaterialProperties(2e7, 0.3)


gator.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
gator.SetInitFwdVel(5)  


gator.Initialize()


gator.GetChassisBody().SetVisualizationType(irr.VisualizationType_MESH)
gator.GetChassisBody().GetVisualModel().SetMeshFile(veh.GetDataFile("gator/chassis.obj"))
gator.GetChassisBody().GetVisualModel().SetTextureFile(veh.GetDataFile("gator/chassis.png"))

for i in range(gator.GetNumberWheels()):
    wheel = gator.GetWheelBody(i)
    wheel.SetVisualizationType(irr.VisualizationType_MESH)
    wheel.GetVisualModel().SetMeshFile(veh.GetDataFile("gator/wheel.obj"))
    wheel.GetVisualModel().SetTextureFile(veh.GetDataFile("gator/wheel.png"))


driver = veh.ChPathFollowerDriver(gator, "my_path", "path.plt")  
driver.SetSteeringGain(0.5)
driver.SetThrottleGain(0.5)
driver.SetTargetSpeed(5)
driver.Initialize()


sensor_manager = veh.ChSensorManager(vehicle_system)
sensor_manager.SetSynchronizationMode(chrono.ChSensorManager.SyncMode_FIXED_STEP)


light1 = chrono.ChPointPointLight()
light1.SetLightPos(chrono.ChVectorD(1, 1, 5))
light1.SetLightDir(chrono.ChVectorD(-1, -1, -5))
sensor_manager.AddLight(light1)

light2 = chrono.ChPointPointLight()
light2.SetLightPos(chrono.ChVectorD(-1, 1, 5))
light2.SetLightDir(chrono.ChVectorD(1, -1, -5))
sensor_manager.AddLight(light2)


camera = chrono.ChCameraSensor(gator.GetChassisBody(),  
                              10,                       
                              chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 1.5),  
                                             chrono.Q_from_AngAxis(chrono.PI/4, chrono.VECT_Z)),  
                              640, 480)                
camera.SetName("main_camera")
camera.SetLensModel(chrono.ChLensModelType::PINHOLE)
camera.SetFocalLength(0.05)  
sensor_manager.AddSensor(camera)


vis = veh.ChVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddLogo()
vis.AttachVehicle(gator.GetChassisBody())


time_step = 1e-3
sim_time = 10.0  

while vehicle_system.GetChTime() < sim_time:
    
    time = vehicle_system.GetChTime()

    
    driver.Synchronize(time)
    driver.Advance(time_step)

    
    terrain.Synchronize(time)
    terrain.Advance(time_step)

    
    gator.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    gator.Advance(time_step)

    
    sensor_manager.Synchronize(time)
    sensor_manager.Advance(time_step)

    
    vis.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    vis.Advance(time_step)

    
    sensor_manager.Render()

    
    if int(time * 10) % 10 == 0:  
        camera.SaveLastImage("camera_frame_" + str(int(time*10)) + ".png")

    
    vis.Render()

print("Simulation completed successfully.")