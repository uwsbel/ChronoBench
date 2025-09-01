import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-15, 0, 0.5)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(3, 0, 2.1)  


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


if tire_model == veh.TireModelType_TMEASY:
    for axle in vehicle.GetAxles():
        tire_left = veh.TMeasyTire()
        tire_right = veh.TMeasyTire()
        tire_left.LoadParameters(veh.GetDataFile("truck/tire/MAN_5t_TMeasyTire.json"))  
        tire_right.LoadParameters(veh.GetDataFile("truck/tire/MAN_5t_TMeasyTire.json"))
        vehicle.InitializeTire(tire_left, axle.GetWheel(veh.LEFT), vis_type)
        vehicle.InitializeTire(tire_right, axle.GetWheel(veh.RIGHT), vis_type)
elif tire_model == veh.TireModelType_RIGID:
    for axle in vehicle.GetAxles():
        tire_left = veh.RigidTire()
        tire_right = veh.RigidTire()
        tire_left.Initialize(axle.GetWheel(veh.LEFT))
        tire_right.Initialize(axle.GetWheel(veh.RIGHT))
        vehicle.GetTireList().append(tire_left)
        vehicle.GetTireList().append(tire_right)


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
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 10.5)  
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())


driver = veh.ChDriver()
driver.Initialize()


print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())


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

    
    if time < 1.0:
        
        steering = 0.0
        throttle = 0.5
        braking = 0.0
    elif time < 2.0:
        
        steering = 0.5 * (time - 1.0)
        throttle = 0.5
        braking = 0.0
    elif time < 3.0:
        
        steering = 0.5 - 0.5 * (time - 2.0)
        throttle = 0.5
        braking = 0.0
    elif time < 4.0:
        
        steering = -0.5 * (time - 3.0)
        throttle = 0.5
        braking = 0.0
    elif time < 5.0:
        
        steering = -0.5 + 0.5 * (time - 4.0)
        throttle = 0.5
        braking = 0.0
    else:
        
        steering = 0.0
        throttle = 0.5
        braking = 0.0

    driver.SetSteering(steering)
    driver.SetThrottle(throttle)
    driver.SetBraking(braking)

    
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