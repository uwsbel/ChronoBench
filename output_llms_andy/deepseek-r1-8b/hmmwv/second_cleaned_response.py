import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_PRIMITIVES


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


circular_path_radius = 50.0  
circular_path_center = chrono.ChVector3d(100.0, 0, 0)  
circular_path_driving_radius = 20.0  
circular_path_segments = 4  
circular_path_speed = 0.5  


path_follower = None


pid_gains = [0.1, 0.2, 0.02]  


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(True)  
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()


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


path_follower = veh.ChPathFollower(vehicle.GetVehicle())
path_follower.SetPathType(veh.PathType_CIRCULAR)
path_follower.SetPathCenter(circular_path_center)
path_follower.SetPathRadius(circular_path_radius)
path_follower.SetDrivingRadius(circular_path_driving_radius)
path_follower.SetPathSegmentCount(circular_path_segments)
path_follower.SetPathFollowingSpeed(circular_path_speed)
path_follower.Initialize()


start_point = path_follower.GetInitialPosition()
end_point = path_follower.GetTargetPosition()
start_sphere = vis.AddSphere(chrono.ChColor(1.0, 0.0, 0.0), start_point, 0.5, True, 0, 1)
end_sphere = vis.AddSphere(chrono.ChColor(0.0, 0.0, 1.0), end_point, 0.5, True, 0, 1)


path_trail = vis.AddLineList()
path_trail.SetPointSize(0.5)
path_trail.SetColor(chrono.ChColor(0.5, 0.5, 0.5))


controller = veh.ChPIDController()
controller.SetGains(pid_gains)
controller.SetPIDInputDimension(3)  
controller.SetPIDOutputDimension(1)  
controller.Initialize()


driver = None  


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

    
    current_pos = vehicle.GetVehicle().GetChassis().GetPosition()
    target_pos = path_follower.GetTargetPosition()

    
    if driver is not None:
        driverinputs = driver.GetInputs()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driverinputs, terrain)
        vis.Synchronize(time, driverinputs)
        driver.Advance(step_size)
    else:
        
        current_pos = vehicle.GetVehicle().GetChassis().GetPosition()
        target_pos = path_follower.GetTargetPosition()
        
        
        error = (target_pos - current_pos).GetX()  
        
        
        if abs(error) > 1e-8:
            steering_angle = controller.GetPIDOutput([error, 0, 0])
        else:
            steering_angle = 0
            
        
        path_follower.SetSteeringAngle(steering_angle)
        path_follower.Advance(step_size)
        
        
        if step_number % 50 == 0:
            current_angle = math.radians(step_number)
            target_pos = circular_path_center + chrono.ChVector3d(
                circular_path_radius * math.cos(current_angle),
                circular_path_radius * math.sin(current_angle),
                0
            )
            path_follower.SetTargetPosition(target_pos)
            path_follower.SetInitialPosition(current_pos)

    step_number += 1
    realtime_timer.Spin(step_size)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())