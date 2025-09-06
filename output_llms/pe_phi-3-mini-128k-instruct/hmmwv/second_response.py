import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set the path to Chrono's data directory
chrono.SetChronoDataPath('/path/to/data')

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced() could be another choice here
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(0, 0, 0.5))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(0.01)  # Changed to a more reasonable step size for tire model

vehicle.Initialize()

# Set visualization types for various vehicle parts
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
terrain_length = 200.0  # Increased terrain length as per instructions
terrain_height = 0
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the path follower controller
sentinel_point = chrono.ChVector3d(0, 0, 0.5)
target_point = chrono.ChVector3d(0, 0, -0.5)

# PID controller gains
Kp = 1.0
Ki = 0.1
Kd = 0.05

# Create the driver system with PID controller
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / steering_time)
driver.SetThrottleDelta(1.0 / throttle_time)
driver.SetBrakingDelta(1.0 / braking_time)

# Initialize the PID controller
pid_controller = veh.ChPIDControllerIRR(Kp, Ki, Kd)

# Create visualization for the sentinel and target points
sentinel_visual = veh.ChVisualShapeSphere(0.1)
target_visual = veh.ChVisualShapeSphere(0.1)

# Create a circular path
circle_path = chrono.ChVector3d(0, 0, 0)
radius = 5.0
path_points = []
for i in range(0, 360, 10):
    angle = math.radians(i)
    path_points.append(sentinel_point + chrono.ChVector3d(radius * math.cos(angle), radius * math.sin(angle), 0.5))

# Initialize the path follower
path_follower = veh.ChPathFollowerIRR(vehicle, circle_path, sentinel_point, target_point, pid_controller)

# Create the driver system with path follower
driver.SetPathFollower(path_follower)

# Simulation loop
step_size = 1e-3
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs (only for path follower, not interactive driver)
    # driver_inputs = driver.GetInputs()  # Removed as we are using a path follower

    # Update modules (process inputs from other modules)
    path_follower.Synchronize(time)
    vehicle.Synchronize(time, None, terrain)  # No driver inputs as we are using a path follower
    vis.Synchronize(time, None)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    # Update the PID controller
    pid_controller.Update()

    # Update visualization for sentinel and target points
    sentinel_visual.SetPosition(path_points[step_number % len(path_points)])
    target_visual.SetPosition(path_points[(step_number + 1) % len(path_points)])

    # Update the path follower
    path_follower.Update()

# Output the vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())