import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')




truck_initLoc = chrono.ChVector3d(10, -5, 0.5)
truck_initRot = chrono.Q_from_AngY(0.2)


sedan_initLoc = chrono.ChVector3d(-10, 5, 0.5)
sedan_initRot = chrono.Q_from_AngY(3.14/4)  


vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_initLoc, truck_initRot))
truck.Initialize()
truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)
truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())

terrain.Initialize()  



sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)
sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)


vis.AttachVehicle(truck.GetTractor())
vis.AddVehicle(sedan)  

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()


driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_sedan = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

driver_sedan.SetSteeringDelta(render_step_size / steering_time)
driver_sedan.SetThrottleDelta(render_step_size / throttle_time)
driver_sedan.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()
driver_sedan.Initialize()


print("VEHICLE MASS (truck):", truck.GetTractor().GetMass())
print("VEHICLE MASS (sedan):", sedan.GetChassisBody().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()


step_number = 0
render_frame = 0


initial_truck_state = {
    'tractor_pos': truck.GetTractor().GetPos(),
    'tractor_rot': truck.GetTractor().GetRot(),
    'trailer_pos': None,  
    'trailer_rot': None
}


while vis.Run():
    time = truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    inputs_truck = driver_truck.GetInputs()
    inputs_sedan = driver_sedan.GetInputs()

    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, inputs_truck, terrain)
    sedan.Synchronize(time, inputs_sedan, terrain)
    vis.Synchronize(time, inputs_truck)  

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    initial_truck_state['tractor_pos'] = truck.GetTractor().GetPos()
    initial_truck_state['tractor_rot'] = truck.GetTractor().GetRot()

    
    step_number += 1

    
    realtime_timer.Spin(step_size)