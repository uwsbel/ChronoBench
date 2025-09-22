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
#  - NONE: don't set inertia (use identity matrix)
#  - MOMENTS_OF_INERTIA: set inertia from moments of inertia
#  - MASS_DUMP: set inertia from a DUMP file
estimation_method = veh.InertiaEstimationMethod_NONE

# Output directory
out_dir = "./"

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# =============================================================================

#print ( "Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n")

# --------------
# Create systems
# --------------

# Create the vehicle, set parameters, and initialize
truck = veh.Truck(veh.DataFile("truck/truck.json"), 
                  tire_model, 
                  tire_step_size)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
truck.SetTireRollingResistance(0.06)
truck.SetTireViscosity(0.3)
truck.Initialize()

truck.SetChassisVisualizationType(chassis_vis_type)
truck.SetSuspensionVisualizationType(suspension_vis_type)
truck.SetSteeringVisualizationType(steering_vis_type)
truck.SetWheelVisualizationType(wheel_vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the ground
ground = chrono.ChBody()
ground.SetFixed(True)
truck.GetSystem().Add(ground)

# Create the tire visualization system
tire_vis = veh.ChTireVisualizationSystemPRIMITIVES()
tire_vis.SetMaxRadius(0.1)
tire_vis.SetLineThickness(0.01)
tire_vis.SetInitPosition(chrono.ChVector3d(0, 0, 0))
truck.GetSystem().Add(tire_vis)

# ------------------
# Create the Irrlicht
# ------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('DAF CF 75.340')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetVehicle())

# ---------------
# Simulation loop
# ---------------

truck.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = truck.GetSystem().GetChTime()

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Collect output data from modules (for inter-module communication)
    driver_inputs = truck.GetDriver().GetInputs()

    # Update modules (process inputs from other modules)
    # (this may exchange data with other modules)
    truck.Synchronize(time)
    ground.Synchronize(time)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    truck.Advance(step_size)
    ground.Advance(step_size)
    vis.Advance(step_size)

    # Advance state of entire system (containing both vehicle and terrain)
    # This is the point where the physics engine's state is updated based on the forces applied
    # during this timestep by all modules (e.g. force from driver inputs, collision forces, ...)
    truck.GetSystem().DoStepDynamics(step_size)