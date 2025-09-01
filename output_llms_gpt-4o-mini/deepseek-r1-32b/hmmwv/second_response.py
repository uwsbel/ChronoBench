import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set up Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Vehicle parameters
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
vis_type = veh.VisualizationType_PRIMITIVES
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY
terrainHeight = 0
terrainLength = 200.0  # Increased from 100.0
terrainWidth = 100.0
step_size = 1e-3
render_step_size = 1.0 / 50
contact_method = chrono.ChContactMethod_NSC

# PID controller gains
Kp = 0.5
Ki = 0.1
Kd = 0.2

# Circular path parameters
path_radius = 50.0
path_center = chrono.ChVector3d(50, 50, 0)

class ChPathFollowerDriver:
    def __init__(self, vehicle, terrain):
        self.vehicle = vehicle
        self.terrain = terrain
        self.last_error = 0
        self.integral = 0
        self.timestamp = 0

    def get_steering(self, time):
        # Get current vehicle position
        vehicle_pos = self.vehicle.GetVehicle().GetChassisBody().GetPos()
        
        # Calculate desired position on circular path
        theta = (time / 10) % (2 * math.pi)
        desired_pos = chrono.ChVector3d(
            path_center.x + path_radius * math.cos(theta),
            path_center.y + path_radius * math.sin(theta),
            vehicle_pos.z
        )
        
        # Calculate error
        error = desired_pos - vehicle_pos
        error.z = 0  # Only consider X and Y
        steering = Kp * error.y + Ki * self.integral + Kd * (error.y - self.last_error)
        
        # Update integral and last error
        self.integral += error.y * step_size
        self.last_error = error.y
        
        return steering

# Create vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following')
vis.SetWindowSize(1280, 1024)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create path visualization spheres
sphere1 = chrono.ChSphereShape()
sphere1.SetRadius(0.5)
sphere1.SetPos(path_center + chrono.ChVector3d(path_radius, 0, 0))
sphere1.SetColor(chrono.ChColor(1, 0, 0))
chrono.GetSystem().Add(sphere1)

sphere2 = chrono.ChSphereShape()
sphere2.SetRadius(0.5)
sphere2.SetPos(path_center + chrono.ChVector3d(0, path_radius, 0))
sphere2.SetColor(chrono.ChColor(0, 1, 0))
chrono.GetSystem().Add(sphere2)

# Create driver
driver = ChPathFollowerDriver(vehicle, terrain)

# Simulation setup
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    steering = driver.get_steering(time)
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.3  # Constant throttle
    driver_inputs.m_steering = steering

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update path visualization
    theta = (time / 10) % (2 * math.pi)
    sphere1.SetPos(path_center + chrono.ChVector3d(path_radius * math.cos(theta), path_radius * math.sin(theta), 0))
    sphere2.SetPos(path_center + chrono.ChVector3d(path_radius * math.cos(theta + math.pi/2), path_radius * math.sin(theta + math.pi/2), 0))

    step_number += 1
    realtime_timer.Spin(step_size)