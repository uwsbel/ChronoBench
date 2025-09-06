import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Encapsulated SCM Terrain Configuration
class SCMTerrainConfig:
    PRESETS = {
        "soft": (2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4),
        "mid": (4e6, 0, 1.1, 0, 35, 0.01, 4e8, 3e4),
        "hard": (6e6, 0, 1.1, 0, 40, 0.01, 6e8, 3e4)
    }
    
    def __init__(self, preset="mid"):
        self.Kphi, self.Kc, self.n, self.coh, self.phi, self.shear, self.K, self.damping = self.PRESETS[preset]
    
    def apply(self, terrain):
        terrain.SetSoilParameters(
            self.Kphi, self.Kc, self.n, 
            self.coh, self.phi, self.shear, 
            self.K, self.damping
        )

# Initialize Chrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation parameters
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID
contact_method = chrono.ChContactMethod_SMC
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # 50 FPS
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Initialize vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization modes
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Initialize visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create and configure SCM terrain with encapsulated parameters
terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
config = SCMTerrainConfig("soft")  # Can be "soft", "mid", or "hard"
config.apply(terrain)

# Enable moving patch around vehicle
terrain.AddMovingPatch(
    vehicle.GetChassisBody(), 
    chrono.ChVector3d(0, 0, 0), 
    chrono.ChVector3d(5, 3, 1)
)

# Set visualization for sinkage
terrain.SetPlotType(
    veh.SCMDeformableTerrain.PLOT_SINKAGE, 
    0, 0.1
)

# Initialize terrain
terrain.Initialize(20, 20, 0.02)

# Initialize driver
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Update systems
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)