import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(6, -70, 0.5)
initRot = chrono.QuatFromAngleZ(1.57)


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



vehicle = veh.HMMWV_Full() 
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


highway_coordsys = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT)
highway_coll_mesh = chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj')



patch_highway = terrain.AddPatch(patch_mat,
                                 highway_coordsys,
                                 highway_coll_mesh,
                                 "highway_mesh",  
                                 0.01,            
                                 False)           


highway_vis_mesh_file = veh.GetDataFile("terrain/meshes/Highway_vis.obj")
vis_mesh_highway = chrono.ChTriangleMeshConnected()
vis_mesh_highway.LoadWavefrontMesh(highway_vis_mesh_file, True, True) 

tri_mesh_shape_highway = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape_highway.SetMesh(vis_mesh_highway)
tri_mesh_shape_highway.SetMutable(False)
patch_highway.GetGroundBody().AddVisualShape(tri_mesh_shape_highway)



bump_pos_vec = chrono.ChVector3d(0, -42, 0)
bump_coordsys = chrono.ChCoordsysd(bump_pos_vec, chrono.QUNIT)


bump_mesh_file = veh.GetDataFile("terrain/meshes/bump.obj")


patch_bump = terrain.AddPatch(patch_mat,
                              bump_coordsys,
                              bump_mesh_file,
                              "bump_mesh",    
                              0.0,            
                              False)          


vis_mesh_bump = chrono.ChTriangleMeshConnected()
vis_mesh_bump.LoadWavefrontMesh(bump_mesh_file, True, True) 

bump_tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
bump_tri_mesh_shape.SetMesh(vis_mesh_bump)
bump_tri_mesh_shape.SetMutable(False)


bump_tri_mesh_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.8))


bump_texture_file = veh.GetDataFile("terrain/textures/dirt.jpg")
bump_tri_mesh_shape.SetTexture(bump_texture_file, 6.0, 6.0) 


patch_bump.GetGroundBody().AddVisualShape(bump_tri_mesh_shape)



terrain.Initialize()



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Modified Terrain Demo')
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