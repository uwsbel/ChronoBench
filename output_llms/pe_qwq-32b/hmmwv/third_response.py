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

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

# Contact method and visualization
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Rendering settings
render_step_size = 1.0 / 50  # FPS = 50

# Custom driver class implementation
class MyDriver(veh.ChDriver):
    def __init__(self, delay):
        super().__init__()
        self.delay = delay

    def Synchronize(self, time):
        super().Synchronize(time)
        time_delayed = max(time - self.delay, 0.0)
        
        # Throttle control: increases to 0.7 after 0.2 seconds
        if time_delayed < 0.2:
            throttle = 0.0
        else:
            elapsed = time_delayed - 0.2
            if elapsed < 0.5:
                throttle = 0.7 * (elapsed / 0.5)
            else:
                throttle = 0.7
        # Steering control: sinusoidal starting at 2 seconds
        if time_delayed < 2.0:
            steering = 0.0
        else:
            elapsed_steering = time_delayed - 2.0
            steering = 0.5 * math.sin(2 * math.pi * elapsed_steering / 2)  # 1Hz sine wave
        braking = 0.0
        
        self.SetThrottle(throttle)
        self.SetSteering(steering)
        self.SetBraking(braking)

# Create the HMMWV vehicle
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

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = irr.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional(chrono.ChVectorD(0, 0, -1), chrono.ChColor(1, 1, 1), 2)
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Custom driver initialization with delay of 0.5 seconds
driver = MyDriver(delay=0.5)
driver.Initialize()

# Simulation control parameters
print("VEHICLE MASS: ", vehicle.GetMass())
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Simulation end condition
    if time >= 4.0:
        break
    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Update driver inputs
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)