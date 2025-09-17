import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

from pychrono.vehicle import ChPathFollowerDriver, CreateStandardDoubleLaneChangePath

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (changed to (-50,0,0.5))
initLoc = chrono.ChVectorD(-50, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

terrainHeight = 0
terrainLength = 200.0  # Increased to 200.0
terrainWidth = 100.0

trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

contact_method = chrono.ChContactMethod_NSC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50

# Create the F157 vehicle (fixed typo from FEDA)
vehicle = veh.F157()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

# Set visualization types before initialization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.Initialize()  # Correct initialization order

# Removed conflicting collision system setting (Bullet vs NSC)
# vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization system
vis = irr.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Path follower driver setup
path = CreateStandardDoubleLaneChangePath()

steering_controller = veh.ChSteeringControllerPID(vehicle.GetVehicle(), path)
steering_controller.SetLookAheadDistance(5.0)
steering_controller.SetGainProportional(0.5)
steering_controller.SetGainIntegral(0.0)
steering_controller.SetGainDerivative(0.0)

speed_controller = veh.ChSpeedControllerPID(vehicle.GetVehicle())
speed_controller.SetGainProportional(0.5)

driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetSteeringController(steering_controller)
driver.SetSpeedController(speed_controller)
driver.SetTargetSpeed(10.0)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)