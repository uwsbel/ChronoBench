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


path_radius = 20.0
sentinel_point = chrono.ChVector3d(path_radius, 0, 0.5)
target_point = chrono.ChVector3d(path_radius * math.cos(math.pi / 2), path_radius * math.sin(math.pi / 2), 0.5)


sentinel_sphere = chrono.ChSphere()
sentinel_sphere.SetRadius(0.5)
sentinel_body = chrono.ChBody()
sentinel_body.SetPos(sentinel_point)
sentinel_body.AddAsset(sentinel_sphere)
sentinel_body.AddAsset(chrono.ChVisualMaterial(chrono.ChColor(1, 0, 0)))
vehicle.GetSystem().Add(sentinel_body)

target_sphere = chrono.ChSphere()
target_sphere.SetRadius(0.5)
target_body = chrono.ChBody()
target_body.SetPos(target_point)
target_body.AddAsset(target_sphere)
target_body.AddAsset(chrono.ChVisualMaterial(chrono.ChColor(0, 1, 0)))
vehicle.GetSystem().Add(target_body)


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_integral = 0
        self.error_previous = 0

    def update(self, error, step_size):
        self.error_integral += error * step_size
        derivative = (error - self.error_previous) / step_size
        self.error_previous = error
        return self.Kp * error + self.Ki * self.error_integral + self.Kd * derivative

pid_controller = PIDController(Kp=10, Ki=0.1, Kd=0.1)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
time = 0
throttle = 0.3  

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    vehicle_pos = vehicle.GetVehicle().GetPos()

    
    vehicle_heading = vehicle.GetVehicle().GetHeading()
    error = math.atan2(target_point(1) - vehicle_pos(1), target_point(0) - vehicle_pos(0)) - vehicle_heading
    if error > math.pi:
        error -= 2 * math.pi
    elif error < -math.pi:
        error += 2 * math.pi

    
    steering = pid_controller.update(error, step_size)

    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = throttle
    driver_inputs.m_steering = steering

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)