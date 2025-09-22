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
out_dir = "./POON_CHASSIS"


# =============================================================================

#print ( "Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n")

# --------------
# Create systems
# --------------

# Create the vehicle, set parameters, and initialize
car = veh.PoonChassis()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(chassis_collision_type)
car.SetChassisFixed(False)
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
car.SetTireType(tire_model)
car.SetTireStepSize(tire_step_size)
car.SetDriveline8WD(True)
car.Initialize()

car.SetChassisVisualizationType(chassis_vis_type)
car.SetSuspensionVisualizationType(suspension_vis_type)
car.SetSteeringVisualizationType(steering_vis_type)
car.SetWheelVisualizationType(wheel_vis_type)
car.SetTireVisualizationType(veh.VisualizationType_MESH)

car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the wheel system
wheel_system = veh.WheelSystem()
wheel_system.SetContactMethod(chrono.ChContactMethod_NSC)
wheel_system.SetWheelDiameter(0.8)
wheel_system.SetMaxMotorVoltageRatio(0.03)
wheel_system.SetMinMotorVoltageRatio(-0.03)
wheel_system.Initialize(car.GetSystem())

# Create and initialize four wheels
wheels = []
for axle in [True, False]:
    for side in [True, False]:
        wheel = wheel_system.AddWheel(car.GetChassisBody(), 
                  chrono.ChVector3d(0.0, 0.0, 0.0 if axle else -0.225, ), 
                  chrono.ChQuaterniond(1, 0, 0, 0), 
                  5.0, axle, side)
        wheel.Initialize()
        wheels.append(wheel)

# Create the steering system
steering_system = veh.SteeringSystem()
steering_system.SetContactMethod(chrono.ChContactMethod_NSC)
steering_system.SetSteeringAngle(0.0)
steering_system.SetSteeringDiameter(0.2)
steering_system.Initialize(car.GetChassisBody(), 
                            chrono.ChVector3d(0.0, 0.0, 0.0), 
                            chrono.ChQuaterniond(1, 0, 0, 0), 
                            True, False)

# Create and initialize two front steering arms
steering_arms = []
steering_pitman_arms = []
for side in [True, False]:
    steering_arm = steering_system.AddSteeringArm(car.GetChassisBody(), 
                                                   chrono.ChVector3d(0.0, 0.0, 0.0, ), 
                                                   chrono.ChQuaterniond(1, 0, 0, 0), 
                                                   True, side)
    steering_arm.Initialize()
    steering_arms.append(steering_arm)
    
    pitman_arm = steering_system.AddSteeringPitmanArm(wheels[1 + bool(side)], 
                                                       chrono.ChVector3d(0.0, 0.0, 0.0, ), 
                                                       chrono.ChQuaterniond(1, 0, 0, 0), )
    pitman_arm.Initialize()
    steering_pitman_arms.append(pitman_arm)

# Create the tire system
tire_system = veh.TireSystem()
tire_system.SetTireType(tire_model)
tire_system.SetContactMethod(chrono.ChContactMethod_NSC)
tire_system.Initialize(car.GetSystem())

# Create and initialize four tires
tires = []
for wheel in wheels:
    tire = tire_system.AddTire(wheel, 
                               inertia_method, 
                               True, False)
    tire.Initialize()
    tires.append(tire)

# Create the suspension system
suspension_system = veh.SuspensionSystem()
suspension_system.SetContactMethod(chrono.ChContactMethod_NSC)
suspension_system.SetRestLength(1.75 - 0.2)
suspension_system.SetSpringStiffness(20000)
suspension_system.SetDamping(150)
suspension_system.Initialize(car.GetChassisBody(), 
                              wheel_system.GetWheelBody(0, True, False), 
                              chrono.ChVector3d(0.0, 0.0, 0.0), 
                              chrono.ChQuaterniond(1, 0, 0, 0), 
                              True, False)

# Create and initialize two front suspensions
suspension_upper = []
suspension_lower = []
for side in [True, False]:
    suspension = suspension_system.AddSuspension(car.GetChassisBody(), 
                                                 wheel_system.GetWheelBody(1 + bool(side), True, False), 
                                                 chrono.ChVector3d(0.0, 0.0, 0.0, ), 
                                                 chrono.ChQuaterniond(1, 0, 0, 0), 
                                                 True, side)
    suspension.Initialize()
    suspension_upper.append(suspension)
    suspension_lower.append(suspension)

# Create the Irrlicht run-time visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Poon chassis')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(car.GetVehicle())

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR()
driver.SetChaseCamera(trackPoint, 6.0, 0.5)
driver.Initialize()
driver.SetSteeringDelta(0.06)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.SetSteeringKey(IRR.KEY_A, IRR.KEY_D)
driver.SetThrottleKey(IRR.KEY_W)
driver.SetBrakingKey(IRR.KEY_S)
driver.SetInitializeHook(vis.Initialize)
driver.SetRunHook(vis.Run)
driver.SetTerminateHook(vis.Terminate)

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

car.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = car.GetSystem().GetChTime()

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Collect output data from modules (for inter-module communication)
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    suspension_system.Synchronize(time)
    steering_system.Synchronize(time)
    wheel_system.Synchronize(time)
    car.Synchronize(time, driver_inputs, suspension_system, steering_system, wheel_system, vis)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    suspension_system.Advance(step_size)
    steering_system.Advance(step_size)
    wheel_system.Advance(step_size)
    car.Advance(step_size)
    vis.Advance(step_size)