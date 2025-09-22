import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "/vehicle/")


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3D(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50


vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVector3d(0,0,0), chrono.QUNIT),
    terrainLength, terrainWidth)


texture_path = "terrain/textures/grass.jpg"  
patch.SetTexture(veh.GetDataFile(texture_path), 200, 200)


patch.SetColor(chrono.ChColor(0.2, 0.8, 0.2))  

terrain.Initialize()




sensor_manager = irr.ChSensorManager()


light1 = irr.ChLightingPoint()
light1.SetPosition(0, 20, 0)
light1.SetIntensity(8.0)  
sensor_manager.Add(light1)

light2 = irr.ChLightingPoint()
light2.SetPosition(20, 20, 20)
light2.SetIntensity(6.0)
sensor_manager.Add(light2)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()


vis.AttachSensorManager(sensor_manager)


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()


vis.AttachVehicle(vehicle.GetVehicle())



chassis_body = vehicle.GetVehicle().GetChassis()
camera_position = chrono.ChVectorD(0.3, 0, 1.0)  


camera_sensor = sensor_manager.AddSensor(chrono.ChSensor(
    chassis_body,  
    chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 1.0)),  
    30.0,  
    1920,  
    1080   
))

camera_sensor.SetTypes(chrono.ChSensorType_CAMERA)


visual_filter = camera_sensor.GetDisplayCamera()


driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()


step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        
        vis.EndScene()
        render_frame += 1

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    camera_sensor.Update()

    
    
    

    
    step_number += 1

    
    realtime_timer.Spin(step_size)