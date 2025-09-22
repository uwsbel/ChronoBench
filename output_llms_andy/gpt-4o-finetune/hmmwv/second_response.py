import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
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


# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced()  could be another choice here
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
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
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


# Create the path and controller
path = veh.ChLinePath()
radius = 20
for i in range(100) :
    theta = 2 * math.pi * i / 100.0
    path.AddPoint(veh.ChPathPoint(chrono.ChVector3d(radius * math.cos(theta), radius * math.sin(theta), 0), chrono.QUNIT, 0))

path.SetClosed(True)

# Visualize path using two balls
point1 = chrono.ChVector3d()
point2 = chrono.ChVector3d()
for i in range(path.GetNumPoints() - 1) :
    path.EvalPoint(i + 0.5, point1)
    path.EvalPointDerivative(i + 0.5, point2)
    dir_vec = abs(point2 - point1)
    mball = chrono.ChVisualShapeBall(0.2)
    mball.SetMutable(False)
    mball.SetPos(point1 + dir_vec * 0.05)
    vis.GetScene().AddVisualShape(mball)
    mball = chrono.ChVisualShapeBall(0.2)
    mball.SetMutable(False)
    mball.SetPos(point1 + dir_vec * 0.95)
    vis.GetScene().AddVisualShape(mball)

# Create PID path follower
controller = veh.ChPathFollowerPID()
end_speed = 0
wnd1 = 10
wnd2 = 10
controller.SetPath(path)
controller.SetLookAheadDistance(2)
controller.SetGains(wnd1, wnd2, wnd1, wnd2, wnd2)
controller.Initialize()

# Initialize constant throttle value
constant_throttle = 0.3

# Initialize sentinel and target points
sentinel = chrono.ChVector3d()
target = chrono.ChVector3d()
# Visualization of the controller points
msent = chrono.ChVisualShapeSphere(0.1)
msent.SetMutable(False)
vis.GetScene().AddVisualShape(msent)
mtarg = chrono.ChVisualShapeSphere(0.1)
mtarg.SetMutable(False)
vis.GetScene().AddVisualShape(mtarg)

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
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

    # Get driver inputs
    # Set constant throttle value
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = constant_throttle

    # Get the current vehicle state
    veh_state = veh.ChVehicleRootState()

    # Synchronize controller
    controller.Synchronize(time, veh_state, driver_inputs)
    driver_inputs.m_steering = controller.GetSteering()

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)