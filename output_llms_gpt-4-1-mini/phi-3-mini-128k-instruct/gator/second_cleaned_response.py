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


terrain_patches = [
    veh.RigidTerrain(vehicle.GetSystem()),
    veh.RigidTerrain(vehicle.GetSystem()),
    veh.RigidTerrain(vehicle.GetSystem()),
    veh.RigidTerrain(vehicle.GetSystem())
]


for i, patch_mat in enumerate(patch_matrices):
    patch_mat.SetHeightMap(chrono.ChHeightMap(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(terrainLength, terrainWidth, terrainHeight), 100, 100, 100, 100, 0.01))
    patch = terrain_patches[i].AddPatch(patch_mat, 
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
        terrainLength, terrainWidth)
    patch.SetTexture(veh.GetDataFile(f"terrain/textures/tile{i}.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.AddBump(chrono.ChVector3d(0, 0, 0.1), 0.05)

terrain_patches[0].Initialize()
terrain_patches[1].Initialize()
terrain_patches[2].Initialize()
terrain_patches[3].Initialize()


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
    terrain_patches[0].Synchronize(time)
    terrain_patches[1].Synchronize(time)
    terrain_patches[2].Synchronize(time)
    terrain_patches[3].Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain_patches)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain_patches[0].Advance(step_size)
    terrain_patches[1].Advance(step_size)
    terrain_patches[2].Advance(step_size)
    terrain_patches[3].Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)