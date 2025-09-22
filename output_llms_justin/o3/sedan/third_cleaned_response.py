import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')




target_speed       = 20.0          
step_size          = 5.0e-4        
tire_step_size     = step_size
render_step_size   = 1.0 / 100.0   
steering_time      = 5.0           




initLoc = chrono.ChVector3d(-10.0, 0.0, 0.5)                 
initRot = chrono.Q_from_AngZ(math.radians(10.0))             




vehicle = veh.Sedan()                                        
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vis_type = veh.VisualizationType_MESH
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(
    chrono.ChCollisionSystem.Type_BULLET)




patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

highway_mesh = veh.GetDataFile("meshes/terrain/Highway.obj")

patch = terrain.AddPatch(patch_mat, highway_mesh, 'Highway')
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




trackPoint = chrono.ChVector3d(0.0, 0.0, 1.8)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan – PID speed control')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)


driver.SetThrottleDelta(0.0)
driver.SetBrakingDelta(0.0)
driver.Initialize()




class SimplePID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def advance(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

pid = SimplePID(kp=0.4, ki=0.1, kd=0.0, dt=step_size)




print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

realtime_timer = chrono.ChRealtimeStepTimer()
render_steps   = math.ceil(render_step_size / step_size)
step_number    = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    
    
    inputs = driver.GetInputs()          

    
    
    
    current_speed = vehicle.GetVehicle().GetSpeed()
    speed_err     = target_speed - current_speed
    pid_out       = pid.advance(speed_err)

    throttle = max(0.0, min(pid_out, 1.0))
    brake    = 0.0
    if pid_out < 0.0:
        throttle = 0.0
        brake    = min(-pid_out, 1.0)

    inputs.m_throttle = throttle
    inputs.m_braking  = brake

    
    
    
    driver.Synchronize(time)                  
    terrain.Synchronize(time)
    vehicle.Synchronize(time, inputs, terrain)
    vis.Synchronize(time, inputs)

    
    
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    realtime_timer.Spin(step_size)
    step_number += 1