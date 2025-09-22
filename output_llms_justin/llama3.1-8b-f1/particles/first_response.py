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
#  - FULL_MATRIX: set full 3x3 inertia matrix
inertia_estimation = veh.InertiaEstimationType_NONE

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
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.3)
driver.Initialize()

tireL = veh.ChTMeasyTire(tire_model, veh.TIRE_RADIUS, True,  veh.COLLISION_TYPE_NONE, 0.1, 0.03)
tireR = veh.ChTMeasyTire(tire_model, veh.TIRE_RADIUS, False, veh.COLLISION_TYPE_NONE, 0.1, 0.03)

assem_susp = veh.ChAssemblySuspension(tireL, tireR, veh.CSV_TYPE_SIMPLE_PARabolic, 0.04, False, 0, 0)
assem_susp.Initialize()

suspF = veh.ChSuspensionFront(0.2, assem_susp, veh.CSA_TYPE_LINEAR, 0, 0)
suspR = veh.ChSuspensionRear(0.2, assem_susp, veh.CSA_TYPE_LINLEAF, 0, 0.04)
suspF.Initialize()
suspR.Initialize()

chassis = veh.ChChassis(veh.CHA_TYPE_PRIMITIVES, chassis_vis_type, chassis_collision_type)
chassis.Initialize(chrono.Bodyd(initLoc, initRot), veh.CHassisMass, inertia_estimation, veh.MOMENTS_OF_INERTIA)

mastoL = veh.ChSteeringAssemblyFront(0.2, assem_susp, veh.MAST_TYPE_PRIMITIVES, steering_vis_type)
mastoR = veh.ChSteeringAssemblyFront(0.2, assem_susp, veh.MAST_TYPE_PRIMITIVES, steering_vis_type)
mastoL.Initialize()
mastoR.Initialize()

wheelL = veh.ChWheel(0.2, assem_susp, wheel_vis_type, True)
wheelR = veh.ChWheel(0.2, assem_susp, wheel_vis_type, False)
wheelL.Initialize()
wheelR.Initialize()

vehicle = veh.ChVehicle(chassis, suspF, suspR, mastoL, mastoR, wheelL, wheelR)
vehicle.Initialize()

# Create the driver system
driver_system = chrono.ChSystemNSC()
driver_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
driver_system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
driver_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
driver_system.SetMaxPenetrationRecoverySpeed(4.0)
driver_system.Initialize()

driver_system.AddLightDirectional()
driver_system.AddSkyBox()
driver_system.GetChassis().SetFixed(True)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Scania HMMWV')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Create the interactive driver system
driver_irr = veh.ChInteractiveDriverIRR(vis)
driver_irr.SetSteeringDelta(0.05)
driver_irr.SetThrottleDelta(0.02)
driver_irr.SetBrakingDelta(0.1)
driver_irr.Initialize()

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetChassis().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = m.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
step_number = 0
render_frame = 0

vehicle.EnableRealtime(True)

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

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
    vehicle.Synchronize(time, driver_inputs, tire_step_size)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1