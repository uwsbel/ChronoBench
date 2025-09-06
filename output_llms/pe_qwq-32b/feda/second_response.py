import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Correct data path setup (removed redundant SetChronoDataPath call)
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (corrected to ChVectorD and moved to -50,0,0.5)
initLoc = chrono.ChVectorD(-50, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain parameters (increased length to 200)
terrainHeight = 0
terrainLength = 200.0
terrainWidth = 100.0

trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

contact_method = chrono.ChContactMethod_NSC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size

render_step_size = 1.0 / 50

# Create the vehicle (fixed chassis_collision_type and corrected vector types)
vehicle = veh.FEDA()
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

# Corrected collision system type setup
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with increased length
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup (corrected vector types)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)  # Removed GetVehicle() call

# Path-follower driver setup
path = chrono.ChPath()
path.AddPoint(chrono.ChVectorD(-50, 0, 0.5))  # Start
path.AddPoint(chrono.ChVectorD(0, 0, 0.5))    # Straight
path.AddPoint(chrono.ChVectorD(50, 1.5, 0.5)) # Left lane change
path.AddPoint(chrono.ChVectorD(100, 0, 0.5))  # Center
path.AddPoint(chrono.ChVectorD(150, -1.5, 0.5)) # Right lane change
path.AddPoint(chrono.ChVectorD(200, 0, 0.5))   # End

driver = veh.ChPathFollowerDriver(vehicle, path)
driver.SetTargetSpeed(10.0)
driver.SetSteeringLookAhead(5.0)

# Steering controller gains (PID)
steering_controller = chrono.ChPIDController()
steering_controller.SetGains(3.0, 0.1, 0.0)  # KP, KI, KD
driver.SetSteeringController(steering_controller)

# Speed controller gains (PID)
speed_controller = chrono.ChPIDController()
speed_controller.SetGains(0.5, 0.05, 0.0)  # KP, KI, KD
driver.SetSpeedController(speed_controller)

driver.Initialize()

print("VEHICLE MASS: ", vehicle.GetMass())

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

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)