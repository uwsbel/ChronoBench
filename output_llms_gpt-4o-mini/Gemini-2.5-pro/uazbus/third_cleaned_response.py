import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH



chassis_collision_type = veh.CollisionType_PRIMITIVES



tire_model = veh.TireModelType_RIGID


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1) 


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size 


render_step_size = 1.0 / 50  



vehicle = veh.UAZBUS() 
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







patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight - 0.001), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()



mysystem = vehicle.GetSystem()
box_contact_mat = chrono.ChContactMaterialNSC()
box_contact_mat.SetFriction(0.7) 
box_contact_mat.SetRestitution(0.1) 


box_half_dx = 0.5 / 2.0
box_half_dy = 5.0 / 2.0
box_half_dz = 0.2 / 2.0
box_density = 1000 

box_body = chrono.ChBodyEasyBox(box_half_dx, box_half_dy, box_half_dz,
                                box_density, True, True, box_contact_mat)
box_body.SetPos(chrono.ChVector3d(5, 0, box_half_dz + terrainHeight)) 
box_body.SetBodyFixed(True) 


if box_body.GetVisualModel() and box_body.GetVisualModel().GetShapes():
    visual_shape_ptr = box_body.GetVisualModel().GetShapes()[0].first 
    if visual_shape_ptr:
        visual_shape_ptr.SetColor(chrono.ChColor(0.8, 0.2, 0.2)) 

mysystem.Add(box_body)



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo with Obstacle')
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

    
    driver_inputs.m_throttle = 0.5
    driver_inputs.m_steering = 0.0
    driver_inputs.m_braking = 0.0

    
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