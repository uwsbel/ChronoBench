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

# Create the SCM deformable terrain patch
class SCMTerrainParams:
    def __init__(self, Kphi=0, Kc=0, n=0, cohesion=0, friction=0, psi=0, E=0, nu=0):
        self.Kphi = Kphi  # Bekker Kphi parameter
        self.Kc = Kc      # Bekker Kc parameter
        self.n = n        # Bekker n parameter
        self.cohesion = cohesion  # Mohr cohesive limit (Pa)
        self.friction = friction  # Mohr friction limit (degrees)
        self.psi = psi    # Janosi shear coefficient (m)
        self.E = E        # Elastic stiffness (Pa/m), before plastic yield
        self.nu = nu      # Damping (Pa s/m), proportional to negative vertical speed (optional)

    def SetSCMParameters(self, terrain, param_type):
        # Set SCM parameters based on the selected type
        if param_type == "soft":
            self.Kphi = 0.5e6
            self.Kc = 0
            self.n = 1.1
            self.cohesion = 0
            self.friction = 20
            self.psi = 0.1
            self.E = 2e7
            self.nu = 3e4
        elif param_type == "mid":
            self.Kphi = 2e6
            self.Kc = 0
            self.n = 1.1
            self.cohesion = 0
            self.friction = 30
            self.psi = 0.03
            self.E = 2e8
            self.nu = 3e4
        elif param_type == "hard":
            self.Kphi = 53.1e6
            self.Kc = 1.5e6
            self.n = 1.37
            self.cohesion = 1.5e6
            self.friction = 31.0
            self.psi = 0.017
            self.E = 3.22e9
            self.nu = 2e5
        else:
            print(f"SCM parameter type [{param_type}] not recognized. Using default values.")
        terrain.SetSoilParameters(self.Kphi,   # Bekker Kphi
                                self.Kc,     # Bekker Kc
                                self.n,      # Bekker n exponent
                                self.cohesion,     # Mohr cohesive limit (Pa)
                                self.friction,    # Mohr friction limit (degrees)
                                self.psi,  # Janosi shear coefficient (m)
                                self.E,     # Elastic stiffness (Pa/m), before plastic yield
                                self.nu    # Damping (Pa s/m), proportional to negative vertical speed (optional)
        )


terrain = veh.SCMTerrain(vehicle.GetSystem())
params = SCMTerrainParams()
params.SetSCMParameters(terrain, "mid")
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