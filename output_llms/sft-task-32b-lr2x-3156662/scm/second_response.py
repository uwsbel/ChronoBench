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


class SCMTerrainConfig:
    """Class to encapsulate SCM terrain parameters with predefined configurations."""
    
    def __init__(self, config_name="mid"):
        """Initialize with a predefined configuration: 'soft', 'mid', or 'hard'."""
        if config_name == "soft":
            self.Kphi = 1e6    # Bekker Kphi (Pa)
            self.Kc = 0        # Bekker Kc (Pa)
            self.n = 1.1       # Bekker n exponent
            self.phi = 20      # Mohr friction angle (degrees)
            self.c = 0         # Mohr cohesive limit (Pa)
            self.k = 1e8       # Elastic stiffness (Pa/m)
            self.damping = 5e4 # Damping coefficient (Pa s/m)
        elif config_name == "mid":
            self.Kphi = 2e6    # Bekker Kphi (Pa)
            self.Kc = 0        # Bekker Kc (Pa)
            self.n = 1.1       # Bekker n exponent
            self.phi = 30      # Mohr friction angle (degrees)
            self.c = 0         # Mohr cohesive limit (Pa)
            self.k = 2e8       # Elastic stiffness (Pa/m)
            self.damping = 3e4 # Damping coefficient (Pa s/m)
        elif config_name == "hard":
            self.Kphi = 3e6    # Bekker Kphi (Pa)
            self.Kc = 0        # Bekker Kc (Pa)
            self.n = 1.1       # Bekker n exponent
            self.phi = 40      # Mohr friction angle (degrees)
            self.c = 0         # Mohr cohesive limit (Pa)
            self.k = 3e8       # Elastic stiffness (Pa/m)
            self.damping = 1e4 # Damping coefficient (Pa s/m)
        else:
            raise ValueError(f"Unknown terrain configuration: {config_name}. Use 'soft', 'mid', or 'hard'.")

    def get_parameters(self):
        """Return the terrain parameters as a tuple for SetSoilParameters."""
        return (self.Kphi, self.Kc, self.n, self.phi, self.c, self.k, self.damping)


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


# Create and configure the SCM deformable terrain using the new class
terrain = veh.SCMTerrain(vehicle.GetSystem())

# Initialize terrain with a predefined configuration (e.g., "mid")
terrain_config = SCMTerrainConfig("mid")
terrain.SetSoilParameters(*terrain_config.get_parameters())

# Optionally, enable moving patch feature (single patch around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution)
terrain.Initialize(20, 20, 0.02)


# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV SCM Terrain Demo')
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
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)