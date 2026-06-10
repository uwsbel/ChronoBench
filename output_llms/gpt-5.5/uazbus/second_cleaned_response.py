import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")





initLoc = chrono.ChVector3d(-40, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY




terrainHeight = 0.0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50.0  




vehicle = veh.UAZBUS()
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

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, terrainHeight),
        chrono.QUNIT
    ),
    terrainLength,
    terrainWidth
)


patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Double Lane Change Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.AttachVehicle(vehicle.GetVehicle())

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()




def get_double_lane_change_inputs(time):
    

    driver_inputs = veh.DriverInputs()

    steering = 0.0
    throttle = 0.0
    braking = 0.0

    
    if time < 4.0:
        steering = 0.0
        throttle = 0.65
        braking = 0.0

    
    elif time < 5.0:
        steering = 0.35
        throttle = 0.60
        braking = 0.0

    
    elif time < 6.0:
        steering = -0.35
        throttle = 0.55
        braking = 0.0

    
    elif time < 7.0:
        steering = -0.35
        throttle = 0.55
        braking = 0.0

    
    elif time < 8.0:
        steering = 0.35
        throttle = 0.50
        braking = 0.0

    
    elif time < 11.0:
        steering = 0.0
        throttle = 0.45
        braking = 0.0

    
    elif time < 13.0:
        steering = 0.0
        throttle = 0.0
        braking = 0.35

    
    else:
        steering = 0.0
        throttle = 0.0
        braking = 0.80

    
    steering = max(-1.0, min(1.0, steering))
    throttle = max(0.0, min(1.0, throttle))
    braking = max(0.0, min(1.0, braking))

    driver_inputs.m_steering = steering
    driver_inputs.m_throttle = throttle
    driver_inputs.m_braking = braking

    return driver_inputs




print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())


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

    
    driver_inputs = get_double_lane_change_inputs(time)

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)