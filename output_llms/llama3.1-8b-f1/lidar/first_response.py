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

# Create the Poon vehicle, set parameters, and initialize
poon = veh.Poon()
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

print("Vehicle mass:               " + str(poon.GetVehicle().GetMass()))
print("Driveline type: " + poon.GetVehicle().GetDriveline().GetTemplateName())
print("Tire model:     " + poon.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")

# Create the tire tracker
#tracker = veh.ChTireTracker()
#tracker.Initialize(poon.GetVehicle())

# Create the ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetFixed(True)
poisson_ratio = 0.03
ground.SetYoungModulus(2e7*(1-poisson_ratio)/poisson_ratio, True, True)
world.Add(ground)

# ---------------
# Create the vehicle Irrlicht interface
# ---------------

# Create the driver system
driver = veh.ChInteractiveDriverIRR()
driver.SetWindowTitle('Poon')
driver.SetWindowSize(1280, 1024)
driver.SetChaseCamera(trackPoint, 6.0, 0.5)
driver.Initialize()
driver.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
driver.AddLightDirectional()
driver.AddSkyBox()
driver.AttachVehicle(poon.GetVehicle())

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  poon.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = m.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
step_number = 0
render_frame = 0

maxTrackingValue = 0

poon.GetVehicle().EnableRealtime(True)

while driver.Run() :

    time = poon.GetSystem().GetChTime()

    # End simulation
    if (time >= tend):
        break

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        driver.BeginScene()
        driver.Render()
        driver.EndScene()
        render_frame += 1
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    ground.Synchronize(time)
    poon.Synchronize(time, driver_inputs, ground)
    #tracker.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    ground.Advance(step_size)
    poon.Advance(step_size)
    #tracker.Advance(step_size)

    # Increment frame number
    step_number += 1