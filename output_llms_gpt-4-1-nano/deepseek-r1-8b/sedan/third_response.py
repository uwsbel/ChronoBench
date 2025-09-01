import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Initial vehicle location and orientation adjusted to (0, 0, 1)
initLoc = chrono.ChVector3d(0, 0, 1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Increased steering response time to 5 seconds
steering_time = 5.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver_max_speed = 50.0  # Maximum speed reference (m/s)

# Decreased simulation and render step sizes for finer control
step_size = 1e-4  # Simulation step size
render_step_size = 1.0 / 50  # FPS = 50

# Reference speed input for controlling the vehicle's speed
speed_ref = 50.0  # Reference speed (m/s)

# PID controller parameters
PID = veh.ChPIDParams()
PID.SetPID(PID.PID_GAMMA, PID.PID_KP, PID.PID_KI)  # Example PID parameters
pid = veh.ChPIDController(PID)

# Create the Sedan vehicle, set parameters, and initialize
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(step_size)

vehicle.Initialize()

# Initialize PID controller for throttle control
vehicle.GetVehicle().SetPIDController(pid, "throttle")

# Create the terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Create highway mesh for terrain
highway_mesh = terrain.AddHighwayMesh(
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth,
    "highway/straight/straight_road_1 texture texture.png"
)
highway_mesh.SetMaterial(chrono.ChContactMaterialNSC())
highway_mesh.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Add reference speed input
driver.SetReferenceSpeed(speed_ref)

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Calculate current speed for PID control
    current_speed = vehicle.GetVehicle().GetSpeed()
    speed_error = speed_ref - current_speed

    # Throttle control using PID
    pid.Update(speed_error)
    throttle_input = pid.Output()

    # Limit throttle input between 0 and 1
    throttle_input = max(0, min(1, throttle_input))

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Apply PID-controlled throttle
    driver.SetThrottleInput(throttle_input)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)