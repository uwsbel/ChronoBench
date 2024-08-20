import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
# Modified: Changed initial X location to -50
initLoc = chrono.ChVector3d(-50, 0, 0.5) 
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
# Modified: Increased terrain length to 200
terrainLength = 200.0  # size in X direction 
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the FEDA vehicle, set parameters, and initialize
vehicle = veh.FEDA()
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


# --- Driver System Modification ---
# Create the path-follower driver system
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), 
                                  "my_path.txt",
                                  "my_path_follower",
                                  10.0) # target speed

# Create the path (Double Lane Change - ISO 3888-2)
# You might need to adjust these values based on your specific needs
path_width = 3.5
path_length = 80.0
s_start1 = 15.0
s_start2 = s_start1 + path_length + 2.5 * path_width
s_end = s_start2 + path_length

# Function to describe the double lane change path
def path_function(s):
    if s < s_start1:
        return 0.0
    elif s < s_start2:
        ds = s - s_start1
        return path_width * math.sin(math.pi * ds / (2.5 * path_width))
    elif s <= s_end:
        ds = s - s_start2
        return -path_width * math.sin(math.pi * ds / (2.5 * path_width))
    else:
        return 0.0

# Generate path points
num_points = 200
path_points = []
for i in range(num_points + 1):
    s = i * (s_end + 10.0) / num_points
    path_points.append(chrono.ChVectorD(s, path_function(s), 0.1))

# Create the path using ChBezierCurve
path = chrono.ChBezierCurve()
path.SetupFromPoints(path_points)

# Set the path for the path follower driver
driver.GetSteeringController().SetLookAheadDistance(5)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)  # Adjust gains if needed
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)      # Adjust gains if needed

driver.Initialize()

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

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

    # Get driver inputs (Not needed for path follower)
    # driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain) # Use driver.GetInputs() here
    vis.Synchronize(time, driver.GetInputs()) # Use driver.GetInputs() here

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)