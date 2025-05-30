import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

class SCMTerrainParameters:
    """Class to manage SCM terrain parameters"""
    
    def __init__(self):
        self.Kphi = 0
        self.Kc = 0
        self.n = 0
        self.cohesive_limit = 0
        self.friction_limit = 0
        self.shear_coeff = 0
        self.elastic_stiffness = 0
        self.damping = 0
        
    def configure(self, config="mid"):
        """Configure terrain parameters using predefined settings"""
        if config == "soft":
            self.set_soft_terrain()
        elif config == "mid":
            self.set_mid_terrain()
        elif config == "hard":
            self.set_hard_terrain()
        else:
            raise ValueError("Invalid terrain configuration")
            
    def set_soft_terrain(self):
        """Set parameters for soft terrain"""
        self.Kphi = 1e6
        self.Kc = 0
        self.n = 1.5
        self.cohesive_limit = 0
        self.friction_limit = 20
        self.shear_coeff = 0.01
        self.elastic_stiffness = 1e7
        self.damping = 1e3
        
    def set_mid_terrain(self):
        """Set parameters for medium terrain"""
        self.Kphi = 2e6
        self.Kc = 0
        self.n = 1.1
        self.cohesive_limit = 0
        self.friction_limit = 30
        self.shear_coeff = 0.01
        self.elastic_stiffness = 2e8
        self.damping = 3e4
        
    def set_hard_terrain(self):
        """Set parameters for hard terrain"""
        self.Kphi = 5e6
        self.Kc = 0
        self.n = 0.8
        self.cohesive_limit = 0
        self.friction_limit = 40
        self.shear_coeff = 0.005
        self.elastic_stiffness = 5e8
        self.damping = 1e5
        
    def configure_custom(self, Kphi, Kc, n, cohesive_limit, friction_limit, shear_coeff, elastic_stiffness, damping):
        """Set custom terrain parameters"""
        self.Kphi = Kphi
        self.Kc = Kc
        self.n = n
        self.cohesive_limit = cohesive_limit
        self.friction_limit = friction_limit
        self.shear_coeff = shear_coeff
        self.elastic_stiffness = elastic_stiffness
        self.damping = damping

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize

vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced()  could be another choice here
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

# Create terrain parameters object and configure
terrain_params = SCMTerrainParameters()
terrain_params.configure("mid")  # Choose between "soft", "mid", or "hard"

# Create the SCM deformable terrain patch
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    terrain_params.Kphi,
    terrain_params.Kc,
    terrain_params.n,
    terrain_params.cohesive_limit,
    terrain_params.friction_limit,
    terrain_params.shear_coeff,
    terrain_params.elastic_stiffness,
    terrain_params.damping
)

# Optionally, enable moving patch feature (single patch around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain.Initialize(20, 20, 0.02)

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

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

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