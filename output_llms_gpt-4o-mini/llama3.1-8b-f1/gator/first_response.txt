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
#  - MOMENTS_OF_INERTIA: set inertia from moments of inertia only
#  - FULL: set inertia from all 9 elements
estimation_method = veh.RigidBodyEstimationMethod_NONE

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 15

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# =============================================================================

#print ( "Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n")

# --------------
# Create systems
# --------------

# Create the vehicle, set parameters, and initialize
driver = veh.ChDriver()
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.2)
driver.Initialize()

tireL = veh.ChTMeasyTire(tire_model, veh.TIRE_RIGID_MESH_FILE, veh.TIRE_PAC02_MESH_FILE, veh.TIRE_PAC02_DATA_FILE)
tireR = veh.ChTMeasyTire(tire_model, veh.TIRE_RIGID_MESH_FILE, veh.TIRE_PAC02_MESH_FILE, veh.TIRE_PAC02_DATA_FILE)
tireAssemblyL = veh.ChLinkLockridgeTire(tireL, veh.CH_ASSEMBLY_TYPE_PRIMITIVES)
tireAssemblyR = veh.ChLinkLockridgeTire(tireR, veh.CH_ASSEMBLY_TYPE_PRIMITIVES)
tireAssemblyL.Initialize(veh.mChCoordsysd(chrono.ChVector3d(0, 0.4, 0), chrono.QUNIT))
tireAssemblyR.Initialize(veh.mChCoordsysd(chrono.ChVector3d(0, -0.4, 0), chrono.QUNIT))
tireAssemblyL.SetTireStepSize(tire_step_size)
tireAssemblyR.SetTireStepSize(tire_step_size)

suspensionL = veh.ChLinkSuspensionSimple()
suspensionR = veh.ChLinkSuspensionSimple()
suspensionL.Initialize(veh.mChCoordsysd(chrono.ChVector3d(0, 0.4, 0), chrono.QUNIT), 2.0, 0.2, 10.0)
suspensionR.Initialize(veh.mChCoordsysd(chrono.ChVector3d(0, -0.4, 0), chrono.QUNIT), 2.0, 0.2, 10.0)
suspensionL.SetSpringRestLength(0.2)
suspensionL.SetShockOringRestLength(0.05)
suspensionR.SetSpringRestLength(0.2)
suspensionR.SetShockOringRestLength(0.05)
suspensionL.Initialize(veh.GetChassis())
suspensionR.Initialize(veh.GetChassis())

wheelL = veh.ChBodyEasyBox(0.1, 0.1, 0.3, 1000, True, True)
wheelR = veh.ChBodyEasyBox(0.1, 0.1, 0.3, 1000, True, True)
wheelL.SetPos(veh.GetSuspensionEndPos(suspensionL, True))
wheelL.SetRot(chrono.QUNIT)
wheelR.SetPos(veh.GetSuspensionEndPos(suspensionR, False))
wheelR.SetRot(chrono.QUNIT)
wheelL.Initialize()
wheelR.Initialize()
wheelL.SetFixed(True)
wheelR.SetFixed(True)

powertrain = veh.ChPowertrain()
powertrain.Initialize(veh.GetChassis(), wheelL.GetBody(), 2.0, 0.2)
powertrain.Initialize(veh.GetChassis(), wheelR.GetBody(), 2.0, 0.2)

vehicle = veh.ChVehicleRigidBody()
vehicle.Initialize(veh.BodyFromFrame(1000, initLoc, initRot), True)

vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetSuspensionVisualizationType(suspension_vis_type)
vehicle.SetSteeringVisualizationType(steering_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)
vehicle.Initialize()

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ------------------
# Create the Irrlicht
# ------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Rigid Body Poon chassis')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ------------------
# Simulation loop
# ------------------

vehicle.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Collect output data from modules (for inter-module communication)
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    tireAssemblyL.Synchronize(time)
    tireAssemblyR.Synchronize(time)
    suspensionL.Synchronize(time)
    suspensionR.Synchronize(time)
    wheelL.Synchronize(time)
    wheelR.Synchronize(time)
    powertrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, tireAssemblyL, tireAssemblyR, suspensionL, suspensionR, wheelL, wheelR, powertrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    tireAssemblyL.Advance(step_size, tireL.GetTireForce())
    tireAssemblyR.Advance(step_size, tireR.GetTireForce())
    suspensionL.Advance(step_size)
    suspensionR.Advance(step_size)
    wheelL.Advance(step_size)
    wheelR.Advance(step_size)
    powertrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)