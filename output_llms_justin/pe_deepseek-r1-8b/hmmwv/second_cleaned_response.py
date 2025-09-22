import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())


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


path_radius = 50.0  
path_angle = 0.0   
current_position = chrono.ChVector3d(0, 0, 0.5)
target_position = None

class PathFollower:
    def __init__(self, vehicle, path_radius):
        self.vehicle = vehicle
        self.path_radius = path_radius
        self.path_angle = 0.0
        self.current_pos = chrono.ChVector3d(0, 0, 0.5)
        self.target_pos = None
        self.pid_steer = PIDController(10.0, 0.1, 0.0)
        self.pid_throttle = PIDController(10.0, 0.1, 0.0)
    
    def Update(self, time_step):
        global path_radius, path_angle
        self.path_angle += time_step
        self.current_pos = self.vehicle.GetVehicle().GetPos()
        self.target_pos = self.current_pos + chrono.ChVector3d(math.cos(self.path_angle) * self.path_radius, math.sin(self.path_angle) * self.path_radius, 0.0)
    
    def GetDirection(self):
        dir_vec = self.target_pos - self.current_pos
        return dir_vec.Normalize()
    
    def GetSteering(self, direction):
        steer_angle = self.GetDirection().GetZ()  
        steer_output = self.pid_steer.Calculate(steer_angle)
        return -steer_output  
    
    def GetThrottle(self):
        distance = self.GetDirection().GetLength()
        if distance > 0:
            throttle_output = self.pid_throttle.Calculate(1.0 / distance)
            return max(0, min(1.0, throttle_output))
        return 0.0


path_follower = PathFollower(vehicle, path_radius)


sentinel_sphere = chrono.ChVisualShapeSphere(0.2)
target_sphere = chrono.ChVisualShapeSphere(0.2)
sentinel_pos = trackPoint
sentinel_sphere.SetPos(sentinel_pos)
target_sphere.SetPos(path_follower.target_pos)
vis.AddVisualShape(sentinel_sphere)
vis.AddVisualShape(target_sphere)


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

    
    path_follower.Update(step_size)
    vehicle.Synchronize(step_size)

    
    direction = path_follower.GetDirection()
    steer = path_follower.GetSteering(direction)
    throttle = path_follower.GetThrottle()

    
    vehicle.GetVehicle().GetChassis().AddForce(chrono.ChVector3d(direction.GetX(), direction.GetY(), 0.0) * 1000.0)
    vehicle.GetVehicle().GetEngine().SetThrottle(throttle)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)