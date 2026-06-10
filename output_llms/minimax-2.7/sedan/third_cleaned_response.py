import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.postprocess as postproc
import math

print(veh)


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(5.0, 0.0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 200.0   


trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-4
tire_step_size = step_size


render_step_size = 1.0 / 100  


reference_speed = 15.0  


Kp = 1.0
Ki = 0.1
Kd = 0.05
integral_error = 0.0
previous_error = 0.0






vehicle = veh.BMW_E90()
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


try:
    highway_mesh = chrono.ChTriangleMeshConnected()
    highway_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/highway.obj"))
    
    patch = terrain.AddMesh(highway_mesh, patch_mat, 
                            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
except:
    
    patch = terrain.AddPlane(patch_mat, 
                             chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = irr.ChIrrApp(vehicle.GetSystem(), 'Sedan Demo', irr.dimension2du(1280, 1024))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddLightWithShadow(irr.vector3df(50, 100, 50), irr.vector3df(0, 0, 0), 100, 0.5, 100, 120)
vis.AddCamera(irr.vector3df(0, 5, -10), irr.vector3df(0, 0, 0))
vis.AddTypicalLights()
vis.SetChaseCamera(trackPoint, 6.0, 0.5)


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 5.0  
throttle_time = 1.0  
braking_time = 0.3   

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)


driver.SetThrottle(0.0)
driver.Initialize()






print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
print("REFERENCE SPEED: ", reference_speed, " m/s")


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()
        render_frame += 1

    
    current_speed = vehicle.GetVehicle().GetSpeed()
    speed_error = reference_speed - current_speed

    
    integral_error += speed_error * step_size
    derivative_error = (speed_error - previous_error) / step_size if step_size > 0 else 0.0
    throttle_output = Kp * speed_error + Ki * integral_error + Kd * derivative_error
    
    
    throttle_output = max(0.0, min(1.0, throttle_output))
    
    previous_error = speed_error

    
    driver_inputs = driver.GetInputs()
    driver_inputs.throttle = throttle_output
    driver.SetThrottle(throttle_output)

    
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