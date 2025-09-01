import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def Update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output


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
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChDriver(vehicle.GetVehicle())


R = 20.0  
Kp = 0.1
Ki = 0.01
Kd = 0.005
pid_controller = PIDController(Kp, Ki, Kd)


path_sphere1 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
path_sphere1.SetPos(chrono.ChVector3d(R, 0, 0.5))
path_sphere1.SetFixed(True)
path_sphere1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/red.png"))
vehicle.GetSystem().Add(path_sphere1)

path_sphere2 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
path_sphere2.SetPos(chrono.ChVector3d(0, R, 0.5))
path_sphere2.SetFixed(True)
path_sphere2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/red.png"))
vehicle.GetSystem().Add(path_sphere2)


sentinel_sphere = chrono.ChBodyEasySphere(0.3, 1000, True, True)
sentinel_sphere.SetFixed(True)
sentinel_sphere.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/green.png"))
vehicle.GetSystem().Add(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.3, 1000, True, True)
target_sphere.SetFixed(True)
target_sphere.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
vehicle.GetSystem().Add(target_sphere)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
render_steps = math.ceil(render_step_size / step_size)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    current_pos = vehicle.GetVehicle().GetPos()
    x = current_pos.x
    y = current_pos.y

    
    distance = math.sqrt(x**2 + y**2)
    error = distance - R  

    
    steering_cmd = pid_controller.Update(error, step_size)
    steering_cmd = max(-1.0, min(1.0, steering_cmd))  

    
    if distance > 0:
        sentinel_x = R * x / distance
        sentinel_y = R * y / distance
    else:
        sentinel_x, sentinel_y = R, 0
    sentinel_sphere.SetPos(chrono.ChVector3d(sentinel_x, sentinel_y, 0.5))

    angle = math.atan2(y, x)
    target_angle = angle + math.radians(30)  
    target_x = R * math.cos(target_angle)
    target_y = R * math.sin(target_angle)
    target_sphere.SetPos(chrono.ChVector3d(target_x, target_y, 0.5))

    
    driver.SetThrottle(0.3)  
    driver.SetSteering(steering_cmd)
    driver.SetBraking(0.0)

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(step_size)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)