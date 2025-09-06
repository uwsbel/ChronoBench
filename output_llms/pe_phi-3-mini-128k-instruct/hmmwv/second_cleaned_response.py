import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath('/path/to/data')


vehicle = veh.HMMWV_Full()  
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(0, 0, 0.5))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(0.01)  

vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain_length = 200.0  
terrain_height = 0
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


sentinel_point = chrono.ChVector3d(0, 0, 0.5)
target_point = chrono.ChVector3d(0, 0, -0.5)


Kp = 1.0
Ki = 0.1
Kd = 0.05


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / steering_time)
driver.SetThrottleDelta(1.0 / throttle_time)
driver.SetBrakingDelta(1.0 / braking_time)


pid_controller = veh.ChPIDControllerIRR(Kp, Ki, Kd)


sentinel_visual = veh.ChVisualShapeSphere(0.1)
target_visual = veh.ChVisualShapeSphere(0.1)


circle_path = chrono.ChVector3d(0, 0, 0)
radius = 5.0
path_points = []
for i in range(0, 360, 10):
    angle = math.radians(i)
    path_points.append(sentinel_point + chrono.ChVector3d(radius * math.cos(angle), radius * math.sin(angle), 0.5))


path_follower = veh.ChPathFollowerIRR(vehicle, circle_path, sentinel_point, target_point, pid_controller)


driver.SetPathFollower(path_follower)


step_size = 1e-3
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    

    
    path_follower.Synchronize(time)
    vehicle.Synchronize(time, None, terrain)  
    vis.Synchronize(time, None)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

    
    pid_controller.Update()

    
    sentinel_visual.SetPosition(path_points[step_number % len(path_points)])
    target_visual.SetPosition(path_points[(step_number + 1) % len(path_points)])

    
    path_follower.Update()


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())