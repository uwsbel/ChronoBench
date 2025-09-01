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
terrainLength = 50.0   
terrainWidth = 50.0    


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


terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_mat1 = chrono.ChContactMaterialNSC()
patch_mat1.SetFriction(0.9)
patch_mat1.SetRestitution(0.01)
patch1 = terrain.AddPatch(patch_mat1, 
    chrono.ChCoordsysd(chrono.ChVector3d(-25, -25, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch1.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.5, 0.8, 0.5))


patch_mat2 = chrono.ChContactMaterialNSC()
patch_mat2.SetFriction(0.8)
patch_mat2.SetRestitution(0.02)
patch2 = terrain.AddPatch(patch_mat2, 
    chrono.ChCoordsysd(chrono.ChVector3d(25, -25, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


patch_mat3 = chrono.ChContactMaterialNSC()
patch_mat3.SetFriction(0.7)
patch_mat3.SetRestitution(0.01)


heightmap_file = "terrain/height_maps/bump64.bmp"  
try:
    patch3 = terrain.AddPatch(patch_mat3,
        chrono.ChCoordsysd(chrono.ChVector3d(-25, 25, 0), chrono.QUNIT),
        heightmap_file, terrainLength, terrainWidth, 0.0, 3.0)  
    patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
    patch3.SetColor(chrono.ChColor(0.7, 0.5, 0.3))
except:
    
    ramp_angle = chrono.CH_PI / 12  
    patch3 = terrain.AddPatch(patch_mat3,
        chrono.ChCoordsysd(chrono.ChVector3d(-25, 25, 1.5), 
                          chrono.QuatFromAngleY(ramp_angle)),
        terrainLength, terrainWidth)
    patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
    patch3.SetColor(chrono.ChColor(0.7, 0.5, 0.3))


patch_mat4 = chrono.ChContactMaterialNSC()
patch_mat4.SetFriction(0.8)
patch_mat4.SetRestitution(0.01)
patch4 = terrain.AddPatch(patch_mat4, 
    chrono.ChCoordsysd(chrono.ChVector3d(25, 25, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch4.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.6, 0.6, 0.6))



bump_material = chrono.ChContactMaterialNSC()
bump_material.SetFriction(0.8)
bump_material.SetRestitution(0.3)


for i in range(3):
    for j in range(3):
        
        bump_pos2 = chrono.ChVector3d(15 + i*5, -35 + j*10, 0.2)
        bump2 = chrono.ChBodyEasyCylinder(bump_material, 1.0, 0.4, 1000, True, True)
        bump2.SetPos(bump_pos2)
        bump2.SetFixed(True)
        vehicle.GetSystem().Add(bump2)
        
        
        bump_pos4 = chrono.ChVector3d(15 + i*5, 15 + j*10, 0.2)
        bump4 = chrono.ChBodyEasyCylinder(bump_material, 1.0, 0.4, 1000, True, True)
        bump4.SetPos(bump_pos4)
        bump4.SetFixed(True)
        vehicle.GetSystem().Add(bump4)


for i in range(2):
    for j in range(2):
        bump_pos1 = chrono.ChVector3d(-35 + i*15, -35 + j*15, 0.15)
        bump1 = chrono.ChBodyEasyBox(bump_material, 2.0, 2.0, 0.3, 1000, True, True)
        bump1.SetPos(bump_pos1)
        bump1.SetFixed(True)
        vehicle.GetSystem().Add(bump1)

terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle - Multi-terrain Test')
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