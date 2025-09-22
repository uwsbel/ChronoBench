import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os 



if 'CHRONO_DATA_DIR' not in os.environ:
    chrono.SetChronoDataPath(chrono.GetChronoDataPath()) 

veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle', ''))


initLoc = chrono.ChVector3d(-15, 0, 2.7) 
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH



chassis_collision_type = veh.CollisionType_NONE


terrainLength = 100.0  
terrainWidth = 100.0   

hmap_file = veh.GetDataFile("terrain/height_maps/slope.png")
hmap_hMin = 0.0  
hmap_hMax = 2.0  
scm_div_x = 100   
scm_div_y = 100   


trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1) 


contact_method = chrono.ChContactMethod_SMC



step_size = 5e-4
tire_step_size = step_size 


render_step_size = 1.0 / 50  


vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN) 
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)     
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)





vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())


terrain.SetSoilParameters(
    2e6,  
    0,    
    1.1,  
    0,    
    30,   
    0.01, 
    4e7,  
    3e4   
)


terrain.SetWaitonData(False) 


terrain.Initialize(
    hmap_file,      
    terrainLength,  
    terrainWidth,   
    hmap_hMin,      
    hmap_hMax,      
    scm_div_x,      
    scm_div_y       
)




scm_visual_asset_found = False
if terrain.GetGroundBody().GetVisualModel() and terrain.GetGroundBody().GetVisualModel().GetShapes():
    
    mesh_shape_asset_tuple = terrain.GetGroundBody().GetVisualModel().GetShapes()[0]
    if mesh_shape_asset_tuple:
        mesh_shape_asset = mesh_shape_asset_tuple[0] 
        
        if isinstance(mesh_shape_asset, chrono.ChTriangleMeshShape):
            
            texture_repeats_u = terrainLength / 2.0
            texture_repeats_v = terrainWidth / 2.0
            mesh_shape_asset.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), texture_repeats_u, texture_repeats_v)
            scm_visual_asset_found = True
            print("SCM terrain texture set to dirt.jpg")

if not scm_visual_asset_found:
    print("Warning: Could not set texture on SCM terrain visual asset. Using color.")
    terrain.SetColor(chrono.ChColor(0.47, 0.39, 0.26))  



vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 SCM Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5) 
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


vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)





print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


step_number = 0



while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()
    
    driver_inputs.m_throttle = 0.8

    
    driver.Synchronize(time)
    terrain.Synchronize(time) 
    vehicle.Synchronize(time, driver_inputs) 
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size) 
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1