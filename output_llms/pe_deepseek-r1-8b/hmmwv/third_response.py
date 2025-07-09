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
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
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

# Custom Driver Class
class MyDriver(veh.ChDriver):
    def __init__(self, delay=0.5):
        super().__init__()
        self.delay = delay
        self.last_time = 0.0
        self.throttle = 0.0
        self.steering = 0.0
        self.braking = 0.0
        self.throttle_time = 0.2  # time to reach 0.7 from 0
        self.steering_start_time = 2.0  # start steering at 2 seconds

    def Synchronize(self, time):
        # Apply delays
        if time - self.last_time > self.delay:
            self.last_time = time

        # Update inputs based on simulation time
        elapsed = time - self.last_time
        self.last_time = time

        # Throttle control
        if elapsed > self.throttle_time:
            self.throttle = 0.7
        else:
            self.throttle = min(0.7, elapsed / self.throttle_time * 0.7)

        # Steering control
        if elapsed > self.steering_start_time:
            # Sinusoidal steering pattern
            phase = (elapsed - self.steering_start_time) % 2.0
            self.steering = 0.2 * chrono.ChMath.sin(phase * 1.0)
        else:
            self.steering = 0.0

        # Braking control (Hold brake while not accelerating)
        if self.throttle < 0.1:
            self.braking = 0.3  # maximum braking
        else:
            self.braking = 0.0

        # Apply inputs
        self.input.throttle = self.throttle
        self.input.steering = self.steering
        self.input.braking = self.braking

driver = MyDriver(delay=0.5)

# Simulation end condition
simulation_end_time = 4.0

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
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

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    vehicle.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Check if simulation time has reached the end condition
    if time >= simulation_end_time:
        break

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())