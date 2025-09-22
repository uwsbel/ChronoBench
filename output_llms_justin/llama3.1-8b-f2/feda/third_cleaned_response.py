import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
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
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())



driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


manager = veh.ChSensorManager(vehicle.GetSystem())


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 1.5), chrono.QUNIT)
camera = veh.ChCameraSensor(
    vehicle.GetChassisBody(),              
    update_rate,                           
    offset_pose,                           
    image_width,                           
    image_height,                          
    fov                                    
)
camera.SetName("Chassis Camera")
manager.AddSensor(camera)


filter = veh.ChFilterVisualize(image_width, image_height, "Chassis Camera POV")
camera.PushFilter(filter)


manager.Initialize()


while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    manager.Synchronize(time, driver_inputs)  

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    manager.Advance(step_size)  

    
    light1 = irr.ChPointLightSceneNode(vis.GetSceneManager())
    light1.setPosition(irr.vector3df(300, 300, 1000))
    light1.setDiffuseColor(irr.SColor(0, 255, 255, 255))
    light1.setSpecularColor(irr.SColor(0, 255, 255, 255))
    vis.GetScene().addLight(light1)

    light2 = irr.ChPointLightSceneNode(vis.GetSceneManager())
    light2.setPosition(irr.vector3df(900, 1500, 1000))
    light2.setDiffuseColor(irr.SColor(0, 255, 255, 255))
    light2.setSpecularColor(irr.SColor(0, 255, 255, 255))
    vis.GetScene().addLight(light2)

    light3 = irr.ChPointLightSceneNode(vis.GetSceneManager())
    light3.setPosition(irr.vector3df(1500, 300, 1000))
    light3.setDiffuseColor(irr.SColor(0, 255, 255, 255))
    light3.setSpecularColor(irr.SColor(0, 255, 255, 255))
    vis.GetScene().addLight(light3)

    light4 = irr.ChPointLightSceneNode(vis.GetSceneManager())
    light4.setPosition(irr.vector3df(0, 1800, 1000))
    light4.setDiffuseColor(irr.SColor(0, 255, 255, 255))
    light4.setSpecularColor(irr.SColor(0, 255, 255, 255))
    vis.GetScene().addLight(light4)

    light5 = irr.ChPointLightSceneNode(vis.GetSceneManager())
    light5.setPosition(irr.vector3df(1200, 0, 1000))
    light5.setDiffuseColor(irr.SColor(0, 255, 255, 255))
    light5.setSpecularColor(irr.SColor(0, 255, 255, 255))
    vis.GetScene().addLight(light5)

    light6 = irr.ChPointLightSceneNode(vis.GetSceneManager())
    light6.setPosition(irr.vector3df(2000, 1200, 1000))
    light6.setDiffuseColor(irr.SColor(0, 255, 255, 255))
    light6.setSpecularColor(irr.SColor(0, 255, 255, 255))
    vis.GetScene().addLight(light6)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)