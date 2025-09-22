import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# New class to manage SCM terrain parameters
class SCMTerrainParams:
    def __init__(self):
        self.params = {
            "soft": {
                "Bekker_Kphi": 2e6,
                "Bekker_Kc": 0,
                "Bekker_n": 1.1,
                "Mohr_cohesion": 0,
                "Mohr_friction": 30,
                "Janosi_shear": 0.01,
                "elastic_stiffness": 2e8,
                "damping": 3e4
            },
            "mid": {
                "Bekker_Kphi": 4e6,
                "Bekker_Kc": 0,
                "Bekker_n": 1.2,
                "Mohr_cohesion": 1000,
                "Mohr_friction": 35,
                "Janosi_shear": 0.015,
                "elastic_stiffness": 4e8,
                "damping": 6e4
            },
            "hard": {
                "Bekker_Kphi": 8e6,
                "Bekker_Kc": 0,
                "Bekker_n": 1.3,
                "Mohr_cohesion": 2000,
                "Mohr_friction": 40,
                "Janosi_shear": 0.02,
                "elastic_stiffness": 8e8,
                "damping": 1.2e5
            }
        }

    def set_params(self, terrain, config):
        if config in self.params:
            params = self.params[config]
            terrain.SetSoilParameters(
                params["Bekker_Kphi"],
                params["Bekker_Kc"],
                params["Bekker_n"],
                params["Mohr_cohesion"],
                params["Mohr_friction"],
                params["Janosi_shear"],
                params["elastic_stiffness"],
                params["damping"]
            )
        else:
            raise ValueError("Invalid terrain configuration")

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain parameters
terrainHeight = 0 
terrainLength = 100.0 
terrainWidth = 100.0 

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50 

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

# Create the SCM deformable terrain patch
terrain = veh.SCMTerrain(vehicle.GetSystem())

# Initialize SCM terrain parameters using the new class
scm_params = SCMTerrainParams()
scm_params.set_params(terrain, "soft")  # Choose "soft", "mid", or "hard"

# Optionally, enable moving patch feature
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain
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

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0 
throttle_time = 1.0 
braking_time = 0.3 
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop
render_steps = int(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
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

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# The corrected and modified script is provided above.