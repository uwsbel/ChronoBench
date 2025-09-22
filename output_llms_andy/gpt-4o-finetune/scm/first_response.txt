import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain parameters
terrainHeight = 0      # Terrain height
terrainLength = 100.0  # Size in X direction
terrainWidth = 100.0   # Size in Y direction

# Poon chassis tracked by camera
trackPoint = chrono.ChVector3d(-15.0, 8.0, 5.8)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full() # veh.HMMWV_Raw()  could be another choice here
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
# Optionally, enable moving patch:
terrain.SetMovingPatch(veh.GetDataFile("vehicle/terrain/patch.bmp"), 40.0, 40.0)

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)

# Initialize the terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain.Initialize(40, 40, 0.02)

# Set the soil parameters by thickness layers (from the contact surface down)
# The top layer is the only one that really matters for pressure response.
# Units: all SI units (N, m, s, ...)
terrain.SetSoilParameters(2e6,   # Bekker Kphi (primary shear parameter)  [Pa/m^(n+1)]
                            0,     # Bekker Kc (secondary shear parameter) [Pa/m^(n+1)]
                            1.1,   # Bekker n exponent
                            0,     # Mohr cohesive limit (usually = Kc) [Pa]
                            30,    # Mohr friction limit (in degrees)
                            2e8,   # Janosi shear coefficient (shear stress relaxation factor) [Pa/m]
                            0.04)  # Elastic stiffness [m/N]

terrain.SetBulldozingFlow(True)            # Enable/disable the bulldozing effect
terrain.SetBulldozingParameters(55,  # Angle [deg] beyond which the soil cannot be lifted any more
                                 1,   # Slow angle (below this angle, the velocity multiplier is 1)
                                 10,  # Fast angle (above this angle, the velocity multiplier is set to some value)
                                 0.1, # Drift coefficient (usually small, e.g. 0.1)
                                 0.9) # Fast coefficient (usually close to 1, e.g. 0.9)

# Set the visualization mesh scale (before calling terrain -> data flush)
terrain.SetMeshResolution(0.05)

# Create the vehicle Irrlicht interface

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.vehicle)


# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # Time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # Time to go from 0 to +1
braking_time = 0.3   # Time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Solver and integrator settings
# ------------------------------
# Use a more accurate solver for the multibody system
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
step_number = 0
render_frame = 0

# Solver and integrator settings
# Use a more accurate solver for the vehicle
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
step_number = 0
render_frame = 0

# ---------------
# Simulation loop
# ---------------

# Inter-module communication data
# --------------------------------
# Driver inputs
driver_inputs = chrono.ChDriverInputs()

# Set real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()
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