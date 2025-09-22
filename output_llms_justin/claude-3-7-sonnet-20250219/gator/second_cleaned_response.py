import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np

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

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType_BULLET)


terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


patch1 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(-25, 25, 0), chrono.QUNIT), 
    50, 50)
patch1.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 50, 50)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


patch2 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(25, 25, 0), chrono.QUNIT), 
    50, 50)
patch2.SetTexture(veh.GetDataFile("terrain/textures/rock.jpg"), 50, 50)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.7))


bump_pos = chrono.ChVector3d(15, 15, 0)
bump_rad = 3.0
bump_height = 0.3
patch2.AddBump(bump_pos.x, bump_pos.y, bump_rad, bump_height)


patch3 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(-25, -25, 0), chrono.QUNIT), 
    50, 50)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
patch3.SetColor(chrono.ChColor(0.9, 0.9, 0.6))



grid_size = 50
height_map = np.zeros((grid_size, grid_size))


for i in range(grid_size):
    for j in range(grid_size):
        
        x = i - grid_size/2
        y = j - grid_size/2
        
        
        height_map[i, j] = 0.2 * (i + j) / grid_size
        
        
        if (i % 10 == 0 and j % 10 == 0):
            height_map[i, j] += 0.1

patch4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, -25, 0), chrono.QUNIT),
    50, 50, height_map, 0.0, 1.0)
patch4.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 50, 50)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.3))


terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
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