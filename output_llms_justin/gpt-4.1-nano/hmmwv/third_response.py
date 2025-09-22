import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size
render_step_size = 1.0 / 50  # 50 FPS

# Create vehicle
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

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Custom Driver Class Implementation
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay=0.5):
        super().__init__(vehicle)
        self.delay = delay
        self.current_time = 0.0  # track simulation time

    def Synchronize(self, time, inputs, terrain):
        self.current_time = time
        # Calculate delay-adjusted input
        delayed_time = max(0.0, self.current_time - self.delay)

        # Throttle: gradually increase to 0.7 after 0.2 seconds
        if delayed_time < 0.2:
            throttle = 0.0
        elif delayed_time < 0.5:
            # Linear increase from 0 to 0.7 between 0.2s and 0.5s
            throttle = 0.7 * ((delayed_time - 0.2) / (0.3))
        else:
            throttle = 0.7

        # Steering: sinusoidal pattern starting at 2 seconds
        if delayed_time < 2:
            steering = 0.0
        else:
            # sinusoidal steering pattern with amplitude 0.5
            steering = 0.5 * math.sin((delayed_time - 2) * math.pi)

        # Braking remains zero
        brake = 0.0

        # Set the driver controls
        self.SetThrottle(throttle)
        self.SetSteering(steering)
        self.SetBraking(brake)

# Initialize custom driver with delay 0.5
driver = MyDriver(vehicle, delay=0.5)

# Set up driver input parameters
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Calculate number of steps between renders
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Simulation loop with end condition at 4 seconds
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # End simulation if time >= 4.0 seconds
    if time >= 4.0:
        break

    # Render scene at specified intervals
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs (not used here, since MyDriver overrides Synchronize)
    driver_inputs = driver.GetInputs()

    # Synchronize all modules
    driver.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment step
    step_number += 1

    # Real-time interface
    realtime_timer.Spin(step_size)