import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Correct and set the Chrono data path
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
terrain.SetSoilParameters(2e7,  # Bekker Kphi
                           0,      # Bekker Kc
                           1.1,    # Bekker n exponent
                           0,      # Mohr cohesive limit (Pa)
                           30,     # Mohr friction limit (degrees)
                           0.01,   # Janosi shear coefficient (m)
                           2e8,    # Elastic stiffness (Pa/m), before plastic yield
                           3e4     # Damping (Pa s/m), proportional to negative vertical speed (optional)
                          )

# Initialize SCM terrain using a height map
terrain.Initialize(chrono.GetChronoDataFile('vehicle/terrain/height_maps/test64.bmp'),  # height map
                   128,         # height scale
                   0,           # phase shift
                   2.0,         # terrain base height
                   50           # mesh refinement (default = 1)
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

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs (hard-coded throttle value)
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.8  # Hard-coded throttle value

    # Update modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1