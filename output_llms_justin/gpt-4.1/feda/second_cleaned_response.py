import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-50, 0, 0.5)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 200.0  
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.FEDA()
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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())







path_points = chrono.vector_ChVector3d()


path_points.push_back(chrono.ChVector3d(-50, 0, 0.5))
path_points.push_back(chrono.ChVector3d(-40, 0, 0.5))
path_points.push_back(chrono.ChVector3d(-30, 0, 0.5))


path_points.push_back(chrono.ChVector3d(-20, 3.5, 0.5))
path_points.push_back(chrono.ChVector3d(-10, 3.5, 0.5))


path_points.push_back(chrono.ChVector3d(0, 0, 0.5))
path_points.push_back(chrono.ChVector3d(10, 0, 0.5))


path_points.push_back(chrono.ChVector3d(20, -3.5, 0.5))
path_points.push_back(chrono.ChVector3d(30, -3.5, 0.5))


path_points.push_back(chrono.ChVector3d(40, 0, 0.5))
path_points.push_back(chrono.ChVector3d(50, 0, 0.5))
path_points.push_back(chrono.ChVector3d(60, 0, 0.5))


path = chrono.ChBezierCurve(path_points)
path_name = "ISO_DoubleLaneChange"
path_obj = veh.ChPath(path, path_name, True)  


path_color = chrono.ChColor(0.0, 0.8, 0.0)
vis.GetSceneManager().addPath(path_obj, path_color, 1.0, False)


target_speed = 10.0  


look_ahead_distance = 5.0
steering_gain = 0.8    
steering_integral = 0  
steering_derivative = 0 


speed_gain = 0.4       
speed_integral = 0     
speed_derivative = 0   

driver = veh.ChPathFollowerDriver(
    vehicle.GetVehicle(),
    path_obj,
    "ISO_DoubleLaneChange",
    target_speed,
    look_ahead_distance,
    steering_gain,
    steering_integral,
    steering_derivative,
    speed_gain,
    speed_integral,
    speed_derivative
)

driver.Initialize()


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
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