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


class PathFollower:
    def __init__(self, vehicle, radius):
        self.vehicle = vehicle
        self.radius = radius
        self.center = chrono.ChVector3d(0, 0, 0)
        self.target_point = chrono.ChVector3d(radius, 0, 0)
        self.sentinel_point = chrono.ChVector3d(radius * math.cos(math.pi / 2), radius * math.sin(math.pi / 2), 0)

        self.Kp = 0.5  
        self.Ki = 0.1  
        self.Kd = 0.1  

        self.error_integral = 0
        self.previous_error = 0

    def update(self, time):
        
        vehicle_pos = self.vehicle.GetVehicle().GetPos()

        
        distance = math.sqrt((vehicle_pos.x() - self.target_point.x()) ** 2 + (vehicle_pos.y() - self.target_point.y()) ** 2)

        
        error = distance - self.radius

        
        self.error_integral += error * 0.01

        
        derivative = (error - self.previous_error) / 0.01

        
        self.previous_error = error

        
        steering_input = self.Kp * error + self.Ki * self.error_integral + self.Kd * derivative

        
        steering_input = max(-1, min(steering_input, 1))

        
        return veh.DriverInputs(steering_input, 0.3, 0)

path_follower = PathFollower(vehicle, 20)


sentinel_sphere = chrono.ChBodyEasySphere(0.2, 1000, chrono.ChMaterialNSC())
sentinel_sphere.SetPos(path_follower.sentinel_point)
sentinel_sphere.SetColor(chrono.ChColor(1, 0, 0))
vehicle.GetSystem().AddBody(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.2, 1000, chrono.ChMaterialNSC())
target_sphere.SetPos(path_follower.target_point)
target_sphere.SetColor(chrono.ChColor(0, 1, 0))
vehicle.GetSystem().AddBody(target_sphere)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = path_follower.update(time)

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)