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


def generate_flat_bump_patch(size_x, size_y, bump_radius, bump_height):
    height_map = []
    for i in range(size_x):
        row = []
        for j in range(size_y):
            dx = (i - size_x / 2) / (size_x / 2)
            dy = (j - size_y / 2) / (size_y / 2)
            distance = math.sqrt(dx**2 + dy**2)
            if distance < bump_radius:
                bump = bump_height * (1 - distance / bump_radius)
            else:
                bump = 0
            row.append(bump)
        height_map.append(row)
    return height_map

def generate_slope_bump_patch(size_x, size_y, slope_angle, bump_radius, bump_height):
    height_map = []
    for i in range(size_x):
        row = []
        for j in range(size_y):
            
            slope_height = (i / (size_x - 1)) * math.tan(slope_angle) * size_y
            dx = (i - size_x / 2) / (size_x / 2)
            dy = (j - size_y / 2) / (size_y / 2)
            distance = math.sqrt(dx**2 + dy**2)
            if distance < bump_radius:
                bump = bump_height * (1 - distance / bump_radius)
            else:
                bump = 0
            row.append(slope_height + bump)
        height_map.append(row)
    return height_map


terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)



slope_angle = math.radians(10)  
bump_radius = 0.2
bump_height = 0.1
slope_bump_map = generate_slope_bump_patch(100, 100, slope_angle, bump_radius, bump_height)


bump_map1 = generate_flat_bump_patch(100, 100, bump_radius, bump_height)


bump_map2 = generate_flat_bump_patch(100, 100, bump_radius, bump_height)


bump_map3 = generate_flat_bump_patch(100, 100, bump_radius, bump_height)


patch1 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 50, 50, slope_bump_map)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile1.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

patch2 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(50, 0, 0), chrono.QUNIT), 50, 50, bump_map1)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile2.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.4))

patch3 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 50, 0), chrono.QUNIT), 50, 50, bump_map2)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile3.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.3))

patch4 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(50, 50, 0), chrono.QUNIT), 50, 50, bump_map3)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.2))

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