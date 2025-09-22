import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





CIRCLE_RADIUS      = 30.0           
CIRCLE_CENTER      = chrono.ChVector3d(0.0, 0.0, 0.0)
INIT_LOC           = chrono.ChVector3d(CIRCLE_CENTER.x + CIRCLE_RADIUS,
                                      CIRCLE_CENTER.y,
                                      0.50)
INIT_ROT           = chrono.QUNIT                           


TERRAIN_LENGTH     = 200.0          
TERRAIN_WIDTH      = 200.0


STEP_SIZE          = 1e-3
RENDER_FPS         = 50
RENDER_STEP_SIZE   = 1.0 / RENDER_FPS


LOOK_AHEAD_DIST    =  6.0           
PID_KP             =  2.0
PID_KI             =  0.0
PID_KD             =  0.2
CONST_THROTTLE     =  0.30




class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_term       = 0.0
        self.prev_error   = 0.0

    def advance(self, error, dt):
        self.i_term     += self.ki * error * dt
        d_term           = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error  = error
        return self.kp * error + self.i_term + d_term




contact_method = chrono.ChContactMethod_NSC
vehicle        = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)


yaw_init = math.pi/2.0
INIT_ROT = chrono.ChQuaterniond(chrono.ChQuaterniond.Euler123_to_Quat(yaw_init, 0, 0))

vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(STEP_SIZE)
vehicle.Initialize()

vis_type = veh.VisualizationType_PRIMITIVES
for part in ["Chassis", "Suspension", "Steering", "Wheel", "Tire"]:
    getattr(vehicle, f"Set{part}VisualizationType")(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




patch_mat  = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch   = terrain.AddPatch(patch_mat,
                           chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                           TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 400, 400)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV – Circular-Path Autonomous Demo')
vis.SetWindowSize(1280, 768)
vis.AddSkyBox()
vis.AddLightDirectional()
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AttachVehicle(vehicle.GetVehicle())
vis.Initialize()




def make_helper_sphere(radius, color):
    body = chrono.ChBodyEasySphere(radius,        
                                   1000,          
                                   True, True)    
    body.SetBodyFixed(True)
    body.GetVisualShape(0).SetColor(color)
    vehicle.GetSystem().Add(body)
    return body

sentinel_sphere = make_helper_sphere(0.25, chrono.ChColor(0, 1, 0))  
target_sphere   = make_helper_sphere(0.25, chrono.ChColor(1, 0, 0))  




pid       = PID(PID_KP, PID_KI, PID_KD)
max_input = 1.0                           




def get_forward_dir(chassis):
    
    return chassis.GetRot().Rotate(chrono.ChVector3d(1, 0, 0))

def clamp(val, lo, hi):
    return max(lo, min(hi, val))




render_steps   = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)
step_number    = 0
realtime_timer = chrono.ChRealtimeStepTimer()

print("VEHICLE TOTAL MASS:", vehicle.GetVehicle().GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    
    
    chassis      = vehicle.GetVehicle().GetChassis()
    pos          = chassis.GetPos()

    fwd_dir      = get_forward_dir(chassis)
    sentinel_pos = pos + LOOK_AHEAD_DIST * fwd_dir
    sentinel_pos.z = 0.05                          

    
    v            = chrono.ChVector3d(sentinel_pos.x - CIRCLE_CENTER.x,
                                     sentinel_pos.y - CIRCLE_CENTER.y, 0)
    if v.Length() < 1e-6:
        v = chrono.ChVector3d(1, 0, 0)             
    v.Normalize()
    target_pos   = chrono.ChVector3d(CIRCLE_CENTER.x + CIRCLE_RADIUS * v.x,
                                     CIRCLE_CENTER.y + CIRCLE_RADIUS * v.y,
                                     0.05)

    
    sentinel_sphere.SetPos(sentinel_pos)
    target_sphere.SetPos(target_pos)

    
    
    
    
    tangent      = chrono.ChVector3d(-v.y, v.x, 0)
    tangent.Normalize()

    
    dot          = chrono.Vdot(fwd_dir, tangent)
    dot          = clamp(dot, -1.0, 1.0)
    cross_z      = fwd_dir.x * tangent.y - fwd_dir.y * tangent.x
    angle_error  = math.atan2(cross_z, dot)        

    steering_raw = pid.advance(angle_error, STEP_SIZE)
    steering     = clamp(steering_raw, -max_input, max_input)

    
    
    
    inputs                  = veh.DriverInputs()   
    inputs.m_steering       = steering
    inputs.m_throttle       = CONST_THROTTLE
    inputs.m_braking        = 0.0

    
    
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, inputs, terrain)
    vis.Synchronize(time, inputs)

    terrain.Advance(STEP_SIZE)
    vehicle.Advance(STEP_SIZE)
    vis.Advance(STEP_SIZE)

    step_number += 1
    realtime_timer.Spin(STEP_SIZE)