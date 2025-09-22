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


path_radius = 50.0     
path_center = chrono.ChVector3d(0, 0, 0.5)  
current_path_angle = 0.0
target_path_angle = 0.0


class PathFollower:
    def __init__(self, vehicle, path_radius, path_center):
        self.vehicle = vehicle
        self.path_radius = path_radius
        self.path_center = path_center
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.max_speed = 5.0  
        self.p_gain = 3.0    
        self.d_gain = 0.1   
        self.i_gain = 0.0   
        
    def Update(self, time, current_pos, current_rot):
        
        target_pos = self.path_center + chrono.ChVector3d(
            self.path_radius * math.cos(self.target_path_angle),
            self.path_radius * math.sin(self.target_path_angle),
            0.5
        )
        
        
        error_pos = target_pos - current_pos
        error_rot = current_rot.Inverse() * (self.path_center.Inverse() * target_pos).GetRot()
        
        
        current_rot_angle = math.atan2(error_pos.y, error_pos.x)
        target_rot_angle = 0.0  
        
        
        steering_error = current_rot_angle - target_rot_angle
        
        
        steering_force = self.p_gain * steering_error + self.d_gain * math.degrees(steering_error) + self.i_gain * (steering_error)
        self.vehicle.SetSteeringForce(steering_force)
        
        
        current_speed = self.vehicle.GetVehicle().GetSpeed()
        if current_speed < self.max_speed:
            throttle_force = self.max_speed - current_speed
            self.vehicle.SetThrottleForce(throttle_force)


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)

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
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


path_follower = PathFollower(vehicle, path_radius, path_center)
path_follower.current_path_angle = current_path_angle
path_follower.target_path_angle = target_path_angle


sentinel_sphere = vis.AddSphere(chrono.ChColor(1.0, 0.0, 0.0), 0.5, 128, 128, 128)
target_sphere = vis.AddSphere(chrono.ChColor(0.0, 1.0, 0.0), 0.5, 128, 128, 128)


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

    
    current_pos = vehicle.GetVehicle().GetPosition()
    current_rot = vehicle.GetVehicle().GetRotation().Inverse()
    path_follower.Update(time, current_pos, current_rot)

    
    sentinel_pos = current_pos
    target_pos = path_follower.target_pos
    sentinel_sphere.SetPosition(sentinel_pos)
    target_sphere.SetPosition(target_pos)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)