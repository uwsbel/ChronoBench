import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# =============================================================================
# Custom Driver Class Implementation
# =============================================================================
class MyDriver(veh.ChDriver):
    """
    Custom driver class that inherits from veh.ChDriver.
    Controls throttle, steering, and braking based on simulation time.
    """
    def __init__(self, vehicle, delay=0.5):
        """
        Initialize the custom driver.
        
        Parameters:
        -----------
        vehicle : ChWheeledVehicle
            The vehicle to control
        delay : float
            Delay in seconds before driver inputs are applied
        """
        super().__init__(vehicle)
        self.delay = delay
    
    def Synchronize(self, time):
        """
        Override Synchronize method to implement custom driver behavior.
        
        - Throttle gradually increases to 0.7 after 0.2 seconds (from delay end)
        - Steering uses sinusoidal pattern starting at 2 seconds
        """
        # Apply delay - no inputs before delay is over
        if time < self.delay:
            self.throttle = 0.0
            self.steering = 0.0
            self.braking = 0.0
            return
        
        # Throttle: gradually increasing to 0.7 after 0.2 seconds (from delay end)
        throttle_target_time = self.delay + 0.2  # 0.5 + 0.2 = 0.7 seconds
        if time < throttle_target_time:
            # Linear increase from 0 to 0.7
            self.throttle = 0.7 * ((time - self.delay) / 0.2)
        else:
            # Throttle stays at 0.7 after 0.2 seconds from delay end
            self.throttle = 0.7
        
        # Steering: sinusoidal pattern starting at 2 seconds
        if time >= 2.0:
            # Sinusoidal steering with amplitude 0.3 and period of 2 seconds
            self.steering = 0.3 * math.sin(math.pi * (time - 2.0))
        else:
            self.steering = 0.0
        
        # Braking: no braking in this implementation
        self.braking = 0.0


# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Chassis point tracked by the camera
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
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced() could be another choice here
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth
)
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


# =============================================================================
# Create custom driver system (replacing ChInteractiveDriverIRR)
# =============================================================================
driver = MyDriver(vehicle.GetVehicle(), delay=0.5)

# Set the time response for steering and throttle inputs
steering_time = 1.0    # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0    # time to go from 0 to +1
braking_time = 0.3     # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# =============================================================================
# Main simulation loop
# =============================================================================
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # ---------------------------------------------------------------------
    # Simulation End Condition: Stop when simulation time reaches 4 seconds
    # ---------------------------------------------------------------------
    if time >= 4.0:
        print(f"Simulation ended at t = {time:.2f} seconds")
        break
    
    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Increment frame number
    step_number += 1
    
    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

print("Simulation complete.")