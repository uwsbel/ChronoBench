import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Path and Controller Implementation
path_radius = 50.0
path_center = chrono.ChVector3d(0, 0, 0.5)
sentinel_point = chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z)
target_point = chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z)

# PID controller gains
Kp = 0.1
Ki = 0.01
Kd = 0.001

# PID controller
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_prev = 0
        self.error_integral = 0

    def Update(self, error):
        self.error_integral += error * step_size
        error_derivative = (error - self.error_prev) / step_size
        self.error_prev = error
        return self.Kp * error + self.Ki * self.error_integral + self.Kd * error_derivative

pid_controller = PIDController(Kp, Ki, Kd)

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update sentinel and target points
    sentinel_point.x = path_center.x + path_radius * math.cos(0.1 * time)
    sentinel_point.y = path_center.y + path_radius * math.sin(0.1 * time)
    target_point.x = path_center.x + path_radius * math.cos(0.1 * time + 0.1)
    target_point.y = path_center.y + path_radius * math.sin(0.1 * time + 0.1)

    # Visualize sentinel and target points
    vis.DrawSphere(sentinel_point, 1.0, chrono.ChColor(1, 0, 0), vis.GetDevice().GetVideoDriver(), vis.GetDevice().GetSceneManager())
    vis.DrawSphere(target_point, 1.0, chrono.ChColor(0, 1, 0), vis.GetDevice().GetVideoDriver(), vis.GetDevice().GetSceneManager())

    # Calculate error
    vehicle_position = vehicle.GetVehicle().GetPos()
    error = math.atan2(target_point.y - vehicle_position.y, target_point.x - vehicle_position.x) - vehicle.GetVehicle().GetRot().GetAngle()

    # PID controller update
    steering_input = pid_controller.Update(error)

    # Throttle input
    throttle_input = 0.3

    # Update vehicle inputs
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.SetSteering(steering_input)
    driver_inputs.SetThrottle(throttle_input)

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)