import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import os
import math




chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
veh.SetDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'vehicle', ''))
sens.SetSensorDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'sensor', ''))





step_size = 2e-3  


t_end = 100


out_dir = "GATOR_SENSOR_DEMO"
if not os.path.exists(out_dir):
    os.makedirs(out_dir)
if not os.path.exists(os.path.join(out_dir, "images")):
    os.makedirs(os.path.join(out_dir, "images"))





print("Creating Chrono system...")

system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)




print("Creating Gator vehicle...")

initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_NSC) 
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(veh.TireModelType_TMEASY) 
gator.SetTireStepSize(step_size)
gator.SetPowertrainType(veh.PowertrainModelType_SIMPLE) 
gator.Initialize()


gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH) 
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

vehicle = gator.GetVehicle()
chassis_body = vehicle.GetChassisBody()




print("Creating rigid terrain...")
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.1), chrono.QUNIT),  
                         200.0, 200.0) 
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




print("Creating Irrlicht visualization...")

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator Simulation with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5) 
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AttachVehicle(vehicle)




print("Creating interactive driver...")
driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(step_size / steering_time)
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)
driver.Initialize()
driver.SetVehicle(vehicle) 




print("Creating sensor manager and sensors...")
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetVerbose(False)



background = sens.Background()
background.mode = sens.BackgroundMode_ENVIRONMENT_MAP
background.env_tex = sens.GetDataFile("sensor/textures/sky_2_4k.hdr") 
sensor_manager.SetBackground(background)




cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.8, 0, 0.5), 
    chrono.Q_from_AngAxis(chrono.CH_PI / 20, chrono.ChVector3d(0, 1, 0)) 
)
update_rate = 30 

camera = sens.ChCameraSensor(
    chassis_body,       
    update_rate,        
    cam_offset_pose,    
    1280,               
    720,                
    chrono.CH_PI / 3,   
    0, 
    sens.CameraLensModelType_PINHOLE, 
    False 
)
camera.SetName("VehicleCamera")



camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View", False))





sensor_manager.AddSensor(camera)





light_intensity = 2.0 
light_radius = 10.0 


light1_offset_pose = chrono.ChFramed(chrono.ChVector3d(2.0, 1.5, 1.0)) 
point_light1 = sens.ChPointLightSensor(
    chassis_body,           
    100,                    
    light1_offset_pose,     
    chrono.ChColor(light_intensity, light_intensity, light_intensity), 
    light_radius            
)
point_light1.SetName("SensorPointLight1")
sensor_manager.AddSensor(point_light1)


light2_offset_pose = chrono.ChFramed(chrono.ChVector3d(2.0, -1.5, 1.0)) 
point_light2 = sens.ChPointLightSensor(
    chassis_body,
    100,
    light2_offset_pose,
    chrono.ChColor(light_intensity, light_intensity, light_intensity),
    light_radius
)
point_light2.SetName("SensorPointLight2")
sensor_manager.AddSensor(point_light2)





print(f"Running simulation for {t_end} seconds...")
realtime_timer = chrono.ChRealtimeStepTimer()
realtime_timer.SetRealtimeStepUnscaled(1.0) 




while vis.Run():
    time = system.GetChTime()
    if time >= t_end:
        break

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    
    system.DoStepDynamics(step_size)

    
    
    sensor_manager.Update()
    
    
    
    
    
    
    
    

    
    vis.BeginScene()
    vis.Render()
    
    
    vis.EndScene()
    
    
    realtime_timer.Spin(step_size)

print("Simulation finished.")