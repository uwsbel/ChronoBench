import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


circular_radius = 50.0  
circular_center = chrono.ChVector3d(0, 0, 0.5)  
path_points = []  

def generateCircularPath():
    angle = 0
    while True:
        x = circular_radius * math.cos(angle)
        y = circular_radius * math.sin(angle)
        z = 0.5  
        point = chrono.ChVector3d(x, y, z)
        path_points.append(point)
        angle += 0.1  
        if angle > 2 * math.pi:
            break

generateCircularPath()


vehicle = veh.HMMWV_Full()  
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


path_follower = veh.ChPathFollowerVehicleSystem()
path_follower.SetVehicle(vehicle)
path_follower.SetPathPoints(path_points)
path_follower.SetPathIndex(0)  
path_follower.SetMaxSteeringAngle(1.0)
path_follower.SetMaxSpeed(5.0)  
path_follower.Initialize()


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Follower Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


start_point = path_points[0]
end_point = path_points[-1]
start_sphere = vis.AddSphere(start_point, 0.5, chrono.ChColor(0, 1, 0))
end_sphere = vis.AddSphere(end_point, 0.5, chrono.ChColor(1, 0, 0))


path_follower_controller = veh.ChPathFollowerPIDController()
path_follower_controller.SetProportionalGain(0.1)
path_follower_controller.SetIntegralGain(0.1)
path_follower_controller.SetDerivativeGain(0.05)
path_follower_controller.Initialize()


driver = None  


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   


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

    
    vehicle.Synchronize(time)
    terrain.Synchronize(time)
    path_follower.Synchronize(time)
    vis.Synchronize(time)

    
    driver.Advance(step_size)  
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    path_follower.Advance(step_size)

    
    current_pos = vehicle.GetVehicle().GetPosition()
    current_angle = path_follower.GetCurrentAngle()

    
    
    

    
    steering_angle = path_follower_controller.GetSteeringAngle(current_pos, current_angle)
    steering_delta = steering_angle * (render_step_size / 2)  

    
    vehicle.GetVehicle().SetSteering(steering_delta)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)