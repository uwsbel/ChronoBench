import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# New class to encapsulate SCM terrain parameters
class SCMConfig:
    def __init__(self, config_name):
        self.config_name = config_name
        if config_name == 'soft':
            self.Kphi = 1e5
            self.Kc = 0.0
            self.n = 0.5
            self.cohesive = 0.0
            self.friction = 20  # degrees
            self.shear_coeff = 0.02
            self.elastic = 1e7
            self.damping = 1e3
        elif config_name == 'mid':
            self.Kphi = 1e6
            self.Kc = 0.0
            self.n = 0.8
            self.cohesive = 0.0
            self.friction = 25
            self.shear_coeff = 0.015
            self.elastic = 5e7
            self.damping = 5e3
        elif config_name == 'hard':
            self.Kphi = 2e6
            self.Kc = 0.0
            self.n = 1.1
            self.cohesive = 0.0
            self.friction = 30
            self.shear_coeff = 0.01
            self.elastic = 2e8
            self.damping = 3e4
        else:
            raise ValueError("Invalid terrain configuration name")

    def apply_to_terrain(self, terrain):
        terrain.SetSoilParameters(
            self.Kphi,
            self.Kc,
            self.n,
            self.cohesive,
            self.friction,
            self.shear_coeff,
            self.elastic,
            self.damping
        )

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + '/vehicle/')  # Added missing slash

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Terrain configuration (soft, mid, hard)
terrain_config = 'hard'

# Rigid terrain parameters (original setup replaced by SCMConfig)
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle
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

# Removed conflicting collision system setup (now handled by contact method)
# vehicle.GetSystem().SetCollisionSystemType(...) was removed

# Create SCM terrain with configuration
terrain = veh.SCMTerrain(vehicle.GetSystem())
config = SCMConfig(terrain_config)
config.apply_to_terrain(terrain)

# Optional moving patch
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize terrain with mesh grid
terrain.Initialize(20, 20, 0.02)

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Driver setup
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop variables
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

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