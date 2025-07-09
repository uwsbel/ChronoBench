import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set the data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrain_model = veh.RigidTerrain.HIGHWAY  # Set to HIGHWAY for highway-like surfaces
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 0.005  # Decreased for finer control
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 0.01  # Increased for smoother rendering

# PID controller parameters
Kp = 0.1
Ki = 0.1
Kd = 0.1
target_speed = 10.0  # m/s
current_speed = 0.0

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Create the vehicle
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

# Initialize vehicle components
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
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

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set time responses for steering and throttle
steering_time = render_step_size * 5  # 5 seconds
throttle_time = render_step_size * 5  # 5 seconds
braking_time = render_step_size * 0.3  # 0.3 seconds
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

# Initialize PID controller
pid = PID(Kp, Ki, Kd)

# Initialize simulation variables
speed = 0.0
throttle = 0.0
speed_display = 0.0

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integrator = 0.0
        self.last_error = 0.0
    
    def calculate(self, error, dt):
        self.integrator += self.Ki * error * dt
        output = self.Kp * error + self.Kd * (error - self.last_error) / dt
        self.last_error = error
        return output

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene every render_step_size
    if step_number % render_steps == 0:
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

    # Calculate current speed
    current_speed = vehicle.GetVehicle().GetSpeedX()  # Assuming speed is in X direction

    # PID control for throttle
    speed_error = target_speed - current_speed
    if speed_error > 1.0:
        speed_error = 1.0
    elif speed_error < -1.0:
        speed_error = -1.0

    if speed_error != 0:
        throttle = pid.calculate(speed_error, step_size)
        if throttle > 1.0:
            throttle = 1.0
        elif throttle < 0.0:
            throttle = 0.0

    # Update vehicle with PID throttle
    vehicle.Advance(step_size, throttle=throttle)

    # Display speed
    speed_display = "{:.2f} m/s".format(current_speed)
    vis.AddText(chrono.ChVector3d(5, 0, 0), speed_display, chrono.ChColor(1, 0, 0))

    # Increment step number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)