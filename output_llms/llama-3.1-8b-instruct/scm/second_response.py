import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

class SCM_Terrain_Params:
    def __init__(self, 
                 bekker_kphi, 
                 bekker_kc, 
                 janosi_n_exponent, 
                 mohr_cohesive_limit, 
                 mohr_friction_limit, 
                 janosi_shear_coefficient, 
                 elastic_stiffness, 
                 damping):
        self.bekker_kphi = bekker_kphi
        self.bekker_kc = bekker_kc
        self.janosi_n_exponent = janosi_n_exponent
        self.mohr_cohesive_limit = mohr_cohesive_limit
        self.mohr_friction_limit = mohr_friction_limit
        self.janosi_shear_coefficient = janosi_shear_coefficient
        self.elastic_stiffness = elastic_stiffness
        self.damping = damping

class Terrain_Params_Config:
    def __init__(self, 
                 name, 
                 params):
        self.name = name
        self.params = params

    def get_params(self):
        return self.params

class SCM_Terrain:
    def __init__(self, system):
        self.system = system
        self.params = None

    def set_soil_parameters(self, params):
        self.params = params

    def initialize(self, length, width, mesh_resolution):
        # Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
        self.system.InitializeSCMTerrain(length, width, mesh_resolution)

    def add_moving_patch(self, body, position, size):
        # Optionally, enable moving patch feature (single patch around vehicle chassis)
        self.system.AddMovingPatch(body, position, size)

    def set_plot_type(self, plot_type, plot_min, plot_max):
        # Set plot type for SCM (false color plotting)
        self.system.SetSCMPLOT(plot_type, plot_min, plot_max)

def get_terrain_params_config(name):
    if name == "soft":
        return Terrain_Params_Config(
            name,
            SCM_Terrain_Params(
                2e6,   # Bekker Kphi
                0,     # Bekker Kc
                1.1,   # Bekker n exponent
                0,     # Mohr cohesive limit (Pa)
                30,    # Mohr friction limit (degrees)
                0.01,  # Janosi shear coefficient (m)
                2e8,   # Elastic stiffness (Pa/m), before plastic yield
                3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
            )
        )
    elif name == "mid":
        return Terrain_Params_Config(
            name,
            SCM_Terrain_Params(
                1e6,   # Bekker Kphi
                0,     # Bekker Kc
                1.1,   # Bekker n exponent
                0,     # Mohr cohesive limit (Pa)
                30,    # Mohr friction limit (degrees)
                0.01,  # Janosi shear coefficient (m)
                2e8,   # Elastic stiffness (Pa/m), before plastic yield
                3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
            )
        )
    elif name == "hard":
        return Terrain_Params_Config(
            name,
            SCM_Terrain_Params(
                1e5,   # Bekker Kphi
                0,     # Bekker Kc
                1.1,   # Bekker n exponent
                0,     # Mohr cohesive limit (Pa)
                30,    # Mohr friction limit (degrees)
                0.01,  # Janosi shear coefficient (m)
                2e8,   # Elastic stiffness (Pa/m), before plastic yield
                3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
            )
        )
    else:
        raise ValueError("Invalid terrain parameters configuration")

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
# terrain_model = veh.RigidTerrain.BOX
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

# Create the SCM terrain
terrain = SCM_Terrain(vehicle.GetSystem())
terrain.set_soil_parameters(get_terrain_params_config("mid").get_params())
terrain.initialize(terrainLength, terrainWidth, 0.02)
terrain.add_moving_patch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.set_plot_type(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

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
    terrain.system.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.system.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)