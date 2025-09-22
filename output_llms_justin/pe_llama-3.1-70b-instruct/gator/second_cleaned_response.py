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


terrain1 = veh.RigidTerrain(vehicle.GetSystem())
patch1 = terrain1.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength/2, terrainWidth/2)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain2 = veh.RigidTerrain(vehicle.GetSystem())
patch2 = terrain2.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(terrainLength/2, 0, 0), chrono.QUNIT), 
    terrainLength/2, terrainWidth/2)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile5.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.4))

terrain3 = veh.RigidTerrain(vehicle.GetSystem())
patch3 = terrain3.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, terrainWidth/2, 0), chrono.QUNIT), 
    terrainLength/2, terrainWidth/2)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile6.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.3))

terrain4 = veh.RigidTerrain(vehicle.GetSystem())
patch4 = terrain4.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(terrainLength/2, terrainWidth/2, 0), chrono.QUNIT), 
    terrainLength/2, terrainWidth/2)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile7.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.2))


height_map = chrono HeightsField()
height_map.LoadFromFile(chrono.GetChronoDataFile("terrain/heightmaps/hm1.jpg"))
patch1.SetHeightfield(height_map)
patch2.SetHeightfield(height_map)
patch3.SetHeightfield(height_map)
patch4.SetHeightfield(height_map)


bump = chrono ChBump()
bump.SetSize(chrono.ChVector3d(10, 10, 5))
bump.SetPosition(chrono.ChVector3d(terrainLength/4, terrainWidth/4, 0))
patch1.AddBump(bump)
patch2.AddBump(bump)
patch3.AddBump(bump)
patch4.AddBump(bump)

terrain1.Initialize()
terrain2.Initialize()
terrain3.Initialize()
terrain4.Initialize()






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
    terrain1.Synchronize(time)
    terrain2.Synchronize(time)
    terrain3.Synchronize(time)
    terrain4.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain1)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain1.Advance(step_size)
    terrain2.Advance(step_size)
    terrain3.Advance(step_size)
    terrain4.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)