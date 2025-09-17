import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

class SCMTerrainParameters:
    def __init__(self, soil_parameters, soil_parameters_2, soil_parameters_3, soil_parameters_4, soil_parameters_5, soil_parameters_6):
        self.soil_parameters = soil_parameters
        self.soil_parameters_2 = soil_parameters_2
        self.soil_parameters_3 = soil_parameters_3
        self.soil_parameters_4 = soil_parameters_4
        self.soil_parameters_5 = soil_parameters_5
        self.soil_parameters_6 = soil_parameters_6

    def set_soil_parameters(self, soil_parameters):
        self.soil_parameters = soil_parameters

    def set_soil_parameters_2(self, soil_parameters_2):
        self.soil_parameters_2 = soil_parameters_2

    def set_soil_parameters_3(self, soil_parameters_3):
        self.soil_parameters_3 = soil_parameters_3

    def set_soil_parameters_4(self, soil_parameters_4):
        self.soil_parameters_4 = soil_parameters_4

    def set_soil_parameters_5(self, soil_parameters_5):
        self.soil_parameters_5 = soil_parameters_5

    def set_soil_parameters_6(self, soil_parameters_6):
        self.soil_parameters_6 = soil_parameters_6

    def get_soil_parameters(self):
        return self.soil_parameters

    def get_soil_parameters_2(self):
        return self.soil_parameters_2

    def get_soil_parameters_3(self):
        return self.soil_parameters_3

    def get_soil_parameters_4(self):
        return self.soil_parameters_4

    def get_soil_parameters_5(self):
        return self.soil_parameters_5

    def get_soil_parameters_6(self):
        return self.soil_parameters_6


# ... (rest of the code)
# Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   # Bekker Kphi
                            0,     # Bekker Kc
                            1.1,   # Bekker n exponent
                            0,     # Mohr cohesive limit (Pa)
                            30,    # Mohr friction limit (degrees)
                            0.01,  # Janosi shear coefficient (m)
                            2e8,   # Elastic stiffness (Pa/m), before plastic yield
                            3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

# Create the SCM terrain parameters object
terrain_params = SCMTerrainParameters(
    soil_parameters=2e6,  # Bekker Kphi
    soil_parameters_2=0,   # Bekker Kc
    soil_parameters_3=1.1,  # Bekker n exponent
    soil_parameters_4=0,   # Mohr cohesive limit (Pa)
    soil_parameters_5=30,   # Mohr friction limit (degrees)
    soil_parameters_6=0.01,  # Janosi shear coefficient (m)
)

# Set the SCM terrain parameters
terrain.SetSoilParameters(terrain_params.get_soil_parameters())
terrain.SetSoilParameters_2(terrain_params.get_soil_parameters_2())
terrain.SetSoilParameters_3(terrain_params.get_soil_parameters_3())
terrain.SetSoilParameters_4(terrain_params.get_soil_parameters_4())
terrain.SetSoilParameters_5(terrain_params.get_soil_parameters_5())
terrain.SetSoilParameters_6(terrain_params.get_soil_parameters_6())

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