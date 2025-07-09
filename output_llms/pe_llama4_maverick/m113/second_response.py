import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Correctly set the Chrono data path
chrono.SetChronoDataPath('/path/to/chrono/data')  # Update this path
veh.SetDataPath('/path/to/chrono/data/vehicle/')  # Update this path

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 0.0)  # Changed initial location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 5e-4
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the MAN vehicle, set parameters, and initialize
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the SCM deformable terrain
terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e7,  # Bekker Kphi (Pa/m)
                          0,      # Bekker Kc (Pa/m^n)
                          1.0,    # Bekker n exponent
                          0,      # Mohr cohesive limit (Pa)
                          30,     # Mohr friction limit (degrees)
                          0.01,   # Janosi shear coefficient (m)
                          2e8,    # Elastic stiffness (Pa/m), before plastic yield
                          3e4     # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

# Initialize SCM terrain using a height map
terrain.Initialize(chrono.GetChronoDataFile("vehicle/terrain/height_maps/test64.bmp"),  # Height map
                   128,         # Height map resolution (number of pixels along y axis)
                   128,         # Height map resolution (number of pixels along x axis)
                   0.0, 2.0,    # Location (min and max height)
                   2.0, 5.0     # Size (X and Y dimensions)
)

# Set the SCM terrain texture
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 16, 16)

# Create the vehicle Irrlicht interface
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.1), 9.0, 1.5)
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

# Solver and integrator settings
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)

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
    # Hard-code throttle value to 0.8
    driver_inputs.m_throttle = 0.8

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1