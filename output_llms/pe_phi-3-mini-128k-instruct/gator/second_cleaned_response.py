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

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_materials = [
    chrono.ChContactMaterialNSC(),  
    chrono.ChContactMaterialNSC(),  
    chrono.ChContactMaterialNSC(),  
    chrono.ChContactMaterialNSC()  
]
patch_textures = [
    veh.GetDataFile("terrain/textures/tile1.jpg"),
    veh.GetDataFile("terrain/textures/tile2.jpg"),
    veh.GetDataFile("terrain/textures/tile3.jpg"),
    veh.GetDataFile("terrain/textures/bump.jpg")
]
patch_height_map = veh.GetDataFile("terrain/height_map.jpg")


terrain = veh.RigidTerrain(vehicle.GetSystem())
for i in range(4):
    patch_mat = patch_materials[i]
    patch_texture = patch_textures[i]
    terrain.AddPatch(patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        terrainLength, terrainWidth)
    terrain.AddPatch(patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(terrainWidth / 2, 0, 0), chrono.QUNIT),
        terrainLength, terrainWidth / 2,
        patch_height_map)  


for i in range(4):
    terrain.GetPatch(i).SetTexture(patch_texture, 200, 200)

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


print(f"VEHICLE MASS: {vehicle.GetVehicle().GetMass()}")