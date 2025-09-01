import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-20, 0, 1.5) 
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  





vehicle = veh.MAN_5t() 
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





terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                        200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200) 
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()


height_map = chrono.ChHeightField()
height_map.SetFileName(chrono.GetChronoDataPath() + 'height_maps/test64.bmp')


terrain_size_x = 100.0
terrain_size_y = 100.0


height_range = height_map.GetHeightRange()
height_scale = 5.0 / height_range


terrain_patch = terrain.AddPatch(patch_mat, 
                                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                                terrain_size_x, terrain_size_y,
                                0.1, height_scale) 


terrain_patch.SetHeightField(height_map)
terrain_patch.SetTexture(chrono.GetChronoDataPath() + "textures/grass.jpg")

terrain.Initialize()





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 5t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(-3.0, 0.0, 1.7), 6.0, 0.5)
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

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)