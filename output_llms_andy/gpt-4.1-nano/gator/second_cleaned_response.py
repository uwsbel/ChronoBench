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


patch_mat_nsc = chrono.ChContactMaterialNSC()
patch_mat_nsc.SetFriction(0.9)
patch_mat_nsc.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_params = [
    {
        'center': chrono.ChVector3d(0, 0, terrainHeight),
        'size_x': terrainLength / 2,
        'size_y': terrainWidth / 2,
        'texture': 'terrain/textures/tile4.jpg',
        'has_height_map': False,
        'height_map_file': None,
        'add_bump': True
    },
    {
        'center': chrono.ChVector3d(terrainLength / 2, 0, terrainHeight),
        'size_x': terrainLength / 2,
        'size_y': terrainWidth / 2,
        'texture': 'terrain/textures/tile2.jpg',
        'has_height_map': False,
        'height_map_file': None,
        'add_bump': False
    },
    {
        'center': chrono.ChVector3d(0, terrainWidth / 2, terrainHeight),
        'size_x': terrainLength / 2,
        'size_y': terrainWidth / 2,
        'texture': 'terrain/textures/tile3.jpg',
        'has_height_map': True,
        'height_map_file': 'terrain/maps/height_map.png',  
        'add_bump': True
    },
    {
        'center': chrono.ChVector3d(terrainLength / 2, terrainWidth / 2, terrainHeight),
        'size_x': terrainLength / 2,
        'size_y': terrainWidth / 2,
        'texture': 'terrain/textures/tile1.jpg',
        'has_height_map': False,
        'height_map_file': None,
        'add_bump': False
    },
]

patches = []
for params in patch_params:
    patch = terrain.AddPatch(
        patch_mat_nsc,
        chrono.ChCoordsysd(params['center'], chrono.QUNIT),
        params['size_x'],
        params['size_y']
    )
    patch.SetTexture(veh.GetDataFile(params['texture']), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    if params['has_height_map']:
        
        height_map = chrono.ChHeightMapTerrain(
            veh.GetDataFile(params['height_map_file'])
        )
        
        terrain.RemovePatch(patch)
        height_patch = terrain.AddHeightMap(
            height_map,
            chrono.ChCoordsysd(params['center'], chrono.QUNIT),
            params['size_x'],
            params['size_y']
        )
        height_patch.SetTexture(veh.GetDataFile(params['texture']), 200, 200)
        height_patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        patches.append(height_patch)
    else:
        patches.append(patch)
    
    if params['add_bump']:
        
        bump_size = 2.0
        bump_height = 0.2
        bump_pos = chrono.ChVector3d(
            params['center'].x + (params['size_x'] / 4),
            params['center'].y + (params['size_y'] / 4),
            terrainHeight + bump_height / 2
        )
        bump = terrain.GetSystem().NewBody()
        bump_shape = chrono.ChBoxShape()
        bump_shape.GetBoxGeometry().Size = chrono.ChVector3d(bump_size/2, bump_size/2, bump_height/2)
        bump.SetPos(bump_pos)
        bump.SetBodyFixed(True)
        bump.GetCollisionModel().ClearModel()
        bump.GetCollisionModel().AddBox(bump_shape, bump_shape.GetBoxGeometry().Size)
        bump.GetCollisionModel().BuildModel()
        bump.SetMaterialSurface(patch_mat_nsc)
        terrain.GetSystem().AddBody(bump)


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