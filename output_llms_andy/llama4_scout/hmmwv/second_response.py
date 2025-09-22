import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data path
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
terrainHeight = 0  # terrain height
terrainLength = 200.0  # size in X direction
terrainWidth = 100.0  # size in Y direction

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
vehicle = veh.HMMWV_Full()
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
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
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

# Path and controller implementation
class PathFollower:
    def __init__(self, vehicle, vis):
        self.vehicle = vehicle
        self.vis = vis
        self.target_speed = 10.0  # m/s
        self.throttle_value = 0.3
        self.steering_pid = chrono.ChPidController(0.5, 0.1, 0.01)

    def update(self, time):
        # Get current vehicle position and speed
        pos = self.vehicle.GetVehicle().GetPos()
        speed = self.vehicle.GetVehicle().GetSpeed()

        # Calculate target position (circular path)
        radius = 20.0
        target_x = radius * math.cos(0.1 * time)
        target_y = radius * math.sin(0.1 * time)
        target_pos = chrono.ChVector3d(target_x, target_y, pos.z())

        # Calculate steering input using PID controller
        steering_input = self.steering_pid.Update(pos.x() - target_x, pos.y() - target_y, render_step_size)

        # Limit steering input to [-1, 1]
        steering_input = max(-1.0, min(steering_input, 1.0))

        # Create driver inputs
        driver_inputs = veh.DriverInputs()
        driver_inputs.SetThrottle(self.throttle_value)
        driver_inputs.SetSteering(steering_input)
        driver_inputs.SetBraking(0.0)

        return driver_inputs

# Create path follower
path_follower = PathFollower(vehicle, vis)

# Create sentinel and target points
sentinel_pos = chrono.ChVector3d(-10.0, 0.0, 1.0)
target_pos = chrono.ChVector3d(0.0, 0.0, 1.0)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()

        # Visualize sentinel and target points
        vis.DrawSphere(sentinel_pos, 0.1, chrono.ChColor(1.0, 0.0, 0.0))
        vis.DrawSphere(target_pos, 0.1, chrono.ChColor(0.0, 1.0, 0.0))

        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update path follower
    driver_inputs = path_follower.update(time)

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

    # Output vehicle mass
    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())