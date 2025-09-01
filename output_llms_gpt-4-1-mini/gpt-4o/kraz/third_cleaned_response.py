import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc_truck = chrono.ChVectorD(0, 0, 0.5)
initRot_truck = chrono.ChQuaternionD(1, 0, 0, 0)

initLoc_sedan = chrono.ChVectorD(5, 0, 0.5)
initRot_sedan = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVectorD(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysD(initLoc_truck, initRot_truck))
truck.Initialize()

truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType_BULLET)


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysD(initLoc_sedan, initRot_sedan))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)


terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())


driver_truck = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()


driver_sedan = veh.ChPathFollowerDriver(sedan.GetVehicle(), veh.StraightLinePath(chrono.ChVectorD(5, 0, 0.5), chrono.ChVectorD(100, 0, 0.5), 1), "my_path", 0.0)
driver_sedan.GetSteeringController().SetLookAheadDistance(5)
driver_sedan.GetSteeringController().SetGains(0.8, 0.0, 0.0)
driver_sedan.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver_sedan.SetDesiredSpeed(10.0)


print("TRUCK MASS: ", truck.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


truck_states = []

while vis.Run():
    time = truck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs_truck = driver_truck.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    truck_states.append((truck.GetTractor().GetPos(), truck.GetTrailer().GetPos()))

    
    step_number += 1

    
    realtime_timer.Spin(step_size)