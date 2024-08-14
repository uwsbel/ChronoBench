import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0
terrainLength = 200.0  # Increased from 100.0 to 200.0
terrainWidth = 200.0   # Increased to match length

# Point tracked by the camera
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create the terrain
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.VNULL, chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the path and path follower
radius = 50.0
center = chrono.ChVectorD(0, 0, 0)
path = veh.ChBezierCurve()
path.AddPoint(center + chrono.ChVectorD(radius, 0, 0))
path.AddPoint(center + chrono.ChVectorD(0, radius, 0))
path.AddPoint(center + chrono.ChVectorD(-radius, 0, 0))
path.AddPoint(center + chrono.ChVectorD(0, -radius, 0))
path.SetClosed(True)

driver = veh.ChPathFollowerDriver(vehicle.GetVehicle())
driver.SetPath(path)
driver.SetDesiredSpeed(10)
driver.SetMaxSteeringAngle(0.4)
driver.SetSteeringPID(0.4, 0.0, 0.1)  # Adjust PID gains as needed
driver.SetThrottleConstant(0.3)
driver.Initialize()

# Create visualization spheres for path
ball1 = chrono.ChBodyEasySphere(0.5, 1000, True, False)
ball1.SetPos(path.GetPoint(0))
ball1.SetBodyFixed(True)
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBodyEasySphere(0.5, 1000, True, False)
ball2.SetPos(path.GetPoint(path.GetNumPoints() // 2))
ball2.SetBodyFixed(True)
vehicle.GetSystem().Add(ball2)

# Create visualization spheres for sentinel and target points
sentinel_sphere = chrono.ChBodyEasySphere(0.5, 1000, True, False)
vehicle.GetSystem().Add(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.5, 1000, True, False)
vehicle.GetSystem().Add(target_sphere)

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update visualization for sentinel and target points
    sentinel_pos = driver.GetSteeringController().GetSentinelLocation()
    target_pos = driver.GetSteeringController().GetTargetLocation()
    sentinel_sphere.SetPos(sentinel_pos)
    target_sphere.SetPos(target_pos)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)