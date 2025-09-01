import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os 



chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '')) 
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))


initLoc = chrono.ChVector3d(0, 0, 0.5) 
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY 


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1) 


contact_method = chrono.ChContactMethod_NSC
contact_vis = False 


step_size = 2e-3 
tire_step_size = 1e-3 


render_step_size = 1.0 / 50  


vehicle = veh.Gator()
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
patch_mat.SetYoungModulus(2e7) 

terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_length = 40.0  
patch_width = 15.0   
terrain_base_z = -0.2 


p1_center_x = patch_length / 2.0
patch1_csys = chrono.ChCoordsysd(chrono.ChVector3d(p1_center_x, 0, terrain_base_z), chrono.QUNIT)
patch1 = terrain.AddPatch(patch_mat, patch1_csys, patch_length, patch_width)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), patch_length / 2, patch_width / 2) 
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


p2_center_x = patch_length * 1.5
patch2_csys = chrono.ChCoordsysd(chrono.ChVector3d(p2_center_x, 0, terrain_base_z), chrono.QUNIT)
patch2 = terrain.AddPatch(patch_mat, patch2_csys, patch_length, patch_width)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), patch_length / 2, patch_width / 2)
patch2.SetColor(chrono.ChColor(0.6, 0.6, 0.6))


bump_x_dim = 0.8  
bump_y_dim = patch_width * 0.6 
bump_z_dim = 0.25 
bump_center_x = p2_center_x 
bump_center_y = 0
bump_center_z = terrain_base_z + bump_z_dim / 2.0

bump = chrono.ChBodyEasyBox(bump_x_dim, bump_y_dim, bump_z_dim,
                             1000,    
                             True,    
                             True,    
                             patch_mat) 
bump.SetPos(chrono.ChVector3d(bump_center_x, bump_center_y, bump_center_z))
bump.SetBodyFixed(True)
bump.GetVisualShape(0).SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg")) 
vehicle.GetSystem().Add(bump)



hm_length = 50.0  
hm_width = patch_width   
hm_min_height = 0.0 
hm_max_height = 2.0 
p3_center_x = patch_length * 2.0 + hm_length / 2.0




patch3_csys = chrono.ChCoordsysd(chrono.ChVector3d(p3_center_x, 0, terrain_base_z), chrono.QUNIT)
heightmap_file = veh.GetDataFile("terrain/height_map.bmp") 
patch3 = terrain.AddPatch(patch_mat,
                          patch3_csys,
                          heightmap_file,    
                          "heightmap_mesh",  
                          hm_length,         
                          hm_width,          
                          hm_min_height,     
                          hm_max_height,     
                          0,                 
                          0.0)               





patch3.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), hm_length / 5, hm_width / 5) 
patch3.SetColor(chrono.ChColor(0.5, 0.7, 0.5)) 


p4_center_x = patch_length * 2.0 + hm_length + patch_length / 2.0
patch4_csys = chrono.ChCoordsysd(chrono.ChVector3d(p4_center_x, 0, terrain_base_z), chrono.QUNIT)
patch4 = terrain.AddPatch(patch_mat, patch4_csys, patch_length, patch_width)
patch4.SetTexture(veh.GetDataFile("terrain/textures/rock.jpg"), patch_length / 3, patch_width / 2)
patch4.SetColor(chrono.ChColor(0.5, 0.4, 0.3))

terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle on Diverse Terrain')
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

print("Chrono data path: " + chrono.GetChronoDataPath())
print("Vehicle data path: " + veh.GetDataPath())


while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
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

del vehicle
del driver