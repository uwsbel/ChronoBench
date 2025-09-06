import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths (update these if needed)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (adjusted)
initLoc = chrono.ChVector3d(0, 0, 0.6)  # Slightly higher initial position
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # No rotation

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes (decreased for finer control)
step_size = 5e-4
tire_step_size = step_size
render_step_size = 1.0 / 100  # Increased FPS to 100

# Create the vehicle system
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Visualization settings
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Create RigidTerrain with mesh
terrain = veh.RigidTerrain(vehicle.GetSystem())
mesh_patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# Load highway mesh (example - adjust path as needed)
highway_mesh = chrono.ChTriangleMeshConnected()
highway_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("terrain/meshes/highway.obj"))
mesh_patch.SetMesh(highway_mesh)
mesh_patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/highway.jpg"), 200, 200)
mesh_patch.SetColor(chrono.ChColor(0.6, 0.6, 0.6))  # Gray color for highway
terrain.Initialize()

# Create Irrlicht visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan with PID Control')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set steering response time to 5 seconds (increased from 1.0)
steering_time = 5.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# PID controller parameters for throttle control
class SpeedPIDController:
    def __init__(self, Kp, Ki, Kd, target_speed):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target_speed = target_speed
        self.prev_error = 0
        self.integral = 0
        
    def Update(self, current_speed, dt):
        error = self.target_speed - current_speed
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        
        # Clamp output between 0 and 1
        return max(0.0, min(1.0, output))

# Initialize PID controller (target speed = 20 m/s)
pid_controller = SpeedPIDController(Kp=0.8, Ki=0.01, Kd=0.05, target_speed=20.0)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Implement PID throttle control based on speed error
    current_speed = vehicle.GetVehicleSpeed()
    pid_throttle = pid_controller.Update(current_speed, step_size)
    
    # Override throttle input from PID controller
    driver_inputs.m_throttle = pid_throttle

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)