import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
from math import ceil

class SCMTerrainParams:
    def __init__(self, config="mid"):
        self.config = config
        self.params = self.get_params()

    def get_params(self):
        if self.config == "soft":
            return {
                'Bekker_Kphi': 2e6,
                'Bekker_Kc': 0,
                'Bekker_n': 1.1,
                'Mohr_cohesion': 0,
                'Mohr_friction': 30,
                'Janosi_shear': 0.01,
                'elastic_stiffness': 2e8,
                'damping': 3e4
            }
        elif self.config == "mid":
            return {
                'Bekker_Kphi': 5e6,
                'Bekker_Kc': 0,
                'Bekker_n': 1.0,
                'Mohr_cohesion': 1e3,
                'Mohr_friction': 35,
                'Janosi_shear': 0.01,
                'elastic_stiffness': 2e8,
                'damping': 3e4
            }
        elif self.config == "hard":
            return {
                'Bekker_Kphi': 1e7,
                'Bekker_Kc': 0,
                'Bekker_n': 0.9,
                'Mohr_cohesion': 2e3,
                'Mohr_friction': 40,
                'Janosi_shear': 0.01,
                'elastic_stiffness': 2e8,
                'damping': 3e4
            }
        else:
            raise ValueError("Invalid configuration. Choose from 'soft', 'mid', 'hard'.")

    def set_terrain_params(self, terrain):
        params = self.params
        terrain.SetSoilParameters(
            params['Bekker_Kphi'],
            params['Bekker_Kc'],
            params['Bekker_n'],
            params['Mohr_cohesion'],
            params['Mohr_friction'],
            params['Janosi_shear'],
            params['elastic_stiffness'],
            params['damping']
        )

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
terrainHeight = 0  # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0  # size in Y direction

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

# Initialize terrain parameters using the new class
terrain_params = SCMTerrainParams("mid")
terrain_params.set_terrain_params(terrain)

terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
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
braking_time = 0.3  # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# ---------------
# Simulation loop
# ---------------
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if step_number % render_steps == 0:
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

# To get the modified code, the above Python code is already provided.