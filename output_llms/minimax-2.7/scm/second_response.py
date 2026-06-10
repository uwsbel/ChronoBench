import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


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

# Terrain configuration preset: "soft", "mid", or "hard"
terrain_config = "mid"

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


# =============================================================================
# SCM Terrain Parameters Class
# =============================================================================
class SCMTerrainParameters:
    """
    Class to manage and set SCM terrain parameters with predefined configurations.
    
    This class encapsulates the soil parameters for the SCM (Stress Contact Model)
    deformable terrain and provides predefined configurations for different
    terrain types.
    """
    
    # Predefined terrain configurations
    CONFIGS = {
        "soft": {
            "Kphi": 1e6,         # Bekker Kphi (soil stiffness modulus)
            "Kc": 0,             # Bekker Kc (cohesive modulus)
            "n": 1.1,            # Bekker n exponent (soil hardness exponent)
            "cohesion": 0,       # Mohr cohesive limit (Pa)
            "friction": 20,     # Mohr friction limit (degrees)
            "Janosi": 0.02,      # Janosi shear coefficient (m)
            "elastic": 5e7,      # Elastic stiffness (Pa/m), before plastic yield
            "damping": 1e3       # Damping (Pa s/m), proportional to negative vertical speed
        },
        "mid": {
            "Kphi": 2e6,         # Bekker Kphi
            "Kc": 0,             # Bekker Kc
            "n": 1.1,            # Bekker n exponent
            "cohesion": 0,       # Mohr cohesive limit (Pa)
            "friction": 30,      # Mohr friction limit (degrees)
            "Janosi": 0.01,      # Janosi shear coefficient (m)
            "elastic": 2e8,       # Elastic stiffness (Pa/m)
            "damping": 3e4        # Damping (Pa s/m)
        },
        "hard": {
            "Kphi": 5e6,         # Bekker Kphi
            "Kc": 0,             # Bekker Kc
            "n": 1.1,            # Bekker n exponent
            "cohesion": 0,       # Mohr cohesive limit (Pa)
            "friction": 40,      # Mohr friction limit (degrees)
            "Janosi": 0.005,     # Janosi shear coefficient (m)
            "elastic": 5e8,      # Elastic stiffness (Pa/m)
            "damping": 5e4        # Damping (Pa s/m)
        }
    }
    
    def __init__(self, config_name="mid"):
        """
        Initialize terrain parameters with a predefined configuration.
        
        Args:
            config_name: Name of the configuration ("soft", "mid", or "hard")
        """
        if config_name not in self.CONFIGS:
            available = ", ".join(self.CONFIGS.keys())
            raise ValueError(
                f"Unknown terrain configuration: '{config_name}'. "
                f"Available configurations: {available}"
            )
        
        self.config_name = config_name
        self.params = self.CONFIGS[config_name].copy()
    
    def set_parameters(self, terrain):
        """
        Apply these parameters to a terrain object.
        
        Args:
            terrain: SCMTerrain object to configure
        """
        terrain.SetSoilParameters(
            self.params["Kphi"],
            self.params["Kc"],
            self.params["n"],
            self.params["cohesion"],
            self.params["friction"],
            self.params["Janosi"],
            self.params["elastic"],
            self.params["damping"]
        )
    
    def get_parameter(self, param_name):
        """
        Get a specific parameter value.
        
        Args:
            param_name: Name of the parameter
            
        Returns:
            The value of the specified parameter
        """
        if param_name not in self.params:
            raise KeyError(f"Unknown parameter: {param_name}")
        return self.params[param_name]
    
    @classmethod
    def get_available_configs(cls):
        """
        Get list of available configuration names.
        
        Returns:
            List of available configuration names
        """
        return list(cls.CONFIGS.keys())


# =============================================================================
# Create the HMMWV vehicle, set parameters, and initialize
# =============================================================================

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

# =============================================================================
# Create the SCM deformable terrain patch
# =============================================================================

terrain = veh.SCMTerrain(vehicle.GetSystem())

# Use the new SCMTerrainParameters class to set soil parameters
terrain_params = SCMTerrainParameters(terrain_config)
terrain_params.set_parameters(terrain)

# Optionally, enable moving patch feature (single patch around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain.Initialize(20, 20, 0.02)

# =============================================================================
# Create the vehicle Irrlicht interface
# =============================================================================

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo('logo_pychrono_alpha.png')  # Fixed: just use filename
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# =============================================================================
# Create the driver system
# =============================================================================

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
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
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

terrain_params = SCMTerrainParameters(terrain_config)
  terrain_params.set_parameters(terrain)