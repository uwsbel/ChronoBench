import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid body inertia estimation method
inertia_method = veh.InertiaEstimationMethod_MESH_PRIMITIVES

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 15

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directories
out_dir = "./POOPoon"

# =============================================================================

#print ( "Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n")

# --------------
# Create systems
# --------------

# Create the vehicle, set parameters, and initialize
poon = veh.PoonChassis()
poon.SetContactMethod(chrono.ChContactMethod_NSC)
poon.SetChassisCollisionType(chassis_collision_type)
poon.SetChassisFixed(False)
poon.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
poon.SetTireType(tire_model)
poon.SetTireStepSize(tire_step_size)
poon.SetInertiaEstimationMethod(inertia_method)
poon.Initialize()

poon.SetChassisVisualizationType(chassis_vis_type)
poon.SetSuspensionVisualizationType(suspension_vis_type)
poon.SetSteeringVisualizationType(steering_vis_type)
poon.SetWheelVisualizationType(wheel_vis_type)

poon.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the tire visualization system
tire_vis = veh.TireVisualization()
tire_vis.SetLineThickness(0.02)
tire_vis.SetHubRadius(0.01)
tire_vis.Initialize()
poon.GetChassis().AddVisualization(tire_vis)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Poon chassis')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(poon.GetVehicle())

# Create the interactive driver system
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
print( "VEHICLE MASS: ",  pois.GetMass())

# Number of simulation steps between miscellaneous events
render_steps = m.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
step_number = 0
render_frame = 0

poon.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = pooch.GetSystem().GetChTime()

    # End simulation
    if (time >= tend):
        break

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
    vis.Synchronize(time, driver_inputs)
    poon.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    vis.Advance(step_size)
    poon.Advance(step_size)

    # Increment frame number
    step_number += 1