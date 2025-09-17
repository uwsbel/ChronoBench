import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle = veh.BMW_E90()
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))


vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_PRIMITIVES
tire_model = veh.TireModelType_RIGID
tire_step_size = 1e-3
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0
trackPoint = chrono.ChVectorD(-5.0, 0.0, 1.8)
contact_method = chrono.ChContactMethod_NSC
contact_vis = False
step_size = 1e-4
render_step_size = 1.0 / 120  


vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 5.0  
driver.SetSteeringDelta(render_step_size / steering_time)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


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