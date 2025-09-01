import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Encapsulate SCM Terrain Parameters
class SCMTerrainParameters:
    def __init__(self, soil_parameters, soil_parameters_2, soil_parameters_3, soil_parameters_4, soil_parameters_5, soil_parameters_6, soil_parameters_7, soil_parameters_8):
        self.soil_parameters = soil_parameters
        self.soil_parameters_2 = soil_parameters_2
        self.soil_parameters_3 = soil_parameters_3
        self.soil_parameters_4 = soil_parameters_4
        self.soil_parameters_5 = soil_parameters_5
        self.soil_parameters_6 = soil_parameters_6
        self.soil_parameters_7 = soil_parameters_7
        self.soil_parameters_8 = soil_parameters_8

# Initialize Terrain Parameters
def initialize_terrain_parameters(terrain_type):
    if terrain_type == "soft":
        soil_parameters = (2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
    elif terrain_type == "mid":
        soil_parameters = (1e6, 0, 1.5, 0, 45, 0.02, 1e7, 5e4)
    elif terrain_type == "hard":
        soil_parameters = (5e6, 0, 2.0, 0, 60, 0.03, 1e8, 1e5)
    else:
        raise ValueError("Invalid terrain type.")
    return soil_parameters

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(chrono.ChCollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-8, 0, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(step_size)

vehicle.Initialize()

# Create the SCM terrain patch
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(initialize_terrain_parameters("soft"))
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