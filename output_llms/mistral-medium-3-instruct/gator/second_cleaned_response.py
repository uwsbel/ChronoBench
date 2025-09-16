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

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())


def create_height_map():
    size = 100
    height_map = np.zeros((size, size), dtype=np.float32)

    
    for i in range(size):
        for j in range(size):
            if j < 50:
                height_map[i, j] = 0.0
            else:
                height_map[i, j] = 0.5 * (j - 50) / 50  

    
    for i in range(size):
        for j in range(size):
            dist = np.sqrt((i-50)**2 + (j-50)**2)
            if dist < 10:
                height_map[i, j] += 0.3 * (1 - dist/10)

    return height_map

height_map = create_height_map()



patch1 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-50, -50, 0), chrono.QUNIT),
    50, 50)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile1.jpg"), 20, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


patch2 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(50, -50, 0), chrono.QUNIT),
    50, 50)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile2.jpg"), 20, 20)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.4))


patch3 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-50, 50, 0), chrono.QUNIT),
    50, 50)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile3.jpg"), 20, 20)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.3))
patch3.SetHeightMap(height_map, 50, 50, 0.5)  


patch4 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(50, 50, 0), chrono.QUNIT),
    50, 50)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.2))


bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.9)
bump_mat.SetRestitution(0.01)
bump = terrain.AddPatch(bump_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(70, 70, 0), chrono.QUNIT),
    10, 10)
bump.SetHeightMap(height_map[30:60, 30:60], 10, 10, 0.3)  

terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle with multi-texture terrain')
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