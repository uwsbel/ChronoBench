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
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
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


path_radius = 20.0
path_center = chrono.ChVector3d(0, 0, 0)
sentinel = chrono.ChVector3d(path_center.x() + path_radius, 0, 0.1)
target = chrono.ChVector3d(path_center.x() + path_radius * math.cos(math.pi / 2), path_radius * math.sin(math.pi / 2), 0.1)


sentinel_sphere = chrono.ChBodyEasySphere(0.1, 1000, chrono.ChMaterialNSC())
sentinel_sphere.SetPos(sentinel)
sentinel_sphere.SetNoMass()
vis.AttachBody(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.1, 1000, chrono.ChMaterialNSC())
target_sphere.SetPos(target)
target_sphere.SetNoMass()
vis.AttachBody(target_sphere)


Kp = 0.5
Ki = 0.1
Kd = 0.1

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_prev = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.error_prev) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.error_prev = error
        return output

pid_controller = PIDController(Kp, Ki, Kd)


driver = veh.ChPathFollowerDriver(vehicle.GetVehicle())
driver.SetPathRadius(path_radius)
driver.SetPathCenter(path_center)
driver.SetThrottle(0.3)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % math.ceil(render_step_size / step_size) == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    vehicle_pos = vehicle.GetVehicle().GetPos()
    steering_error = math.atan2(vehicle_pos.y() - path_center.y(), vehicle_pos.x() - path_center.x()) - vehicle.GetVehicle().GetHeading()

    
    steering_output = pid_controller.update(steering_error, step_size)

    
    driver_inputs.steering = steering_output

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)