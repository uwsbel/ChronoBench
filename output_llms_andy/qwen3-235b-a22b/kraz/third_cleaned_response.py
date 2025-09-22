import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import csv


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc_truck = chrono.ChVector3d(0, 0.5, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)

initLoc_sedan = chrono.ChVector3d(5, -5, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


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
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.SetTireType(tire_model)
truck.Initialize()

truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.SetTireType(tire_model)
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)


terrain = veh.RigidTerrain(truck.GetSystem())
highway_mesh = veh.GetDataFile("terrain/meshes/highway.obj")
patch = terrain.AddPatch(patch_mat, highway_mesh, veh.CollisionType_MESH,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                          chrono.ChVector3d(1, 1, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/tarmac.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowSize(1280, 1024)
vis.SetWindowTitle('Multi-Vehicle Simulation')
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.Initialize()


truck_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)

sedan_driver.SetSteeringDelta(render_step_size / steering_time)
sedan_driver.SetThrottleDelta(render_step_size / throttle_time)
sedan_driver.SetBrakingDelta(render_step_size / braking_time)

truck_driver.Initialize()
sedan_driver.Initialize()


state_file = open('truck_states.csv', 'w', newline='')
state_writer = csv.writer(state_file)
state_writer.writerow(['time', 'tractor_pos_x', 'tractor_pos_y', 'tractor_pos_z',
                       'tractor_vel_x', 'tractor_vel_y', 'tractor_vel_z',
                       'trailer_pos_x', 'trailer_pos_y', 'trailer_pos_z',
                       'trailer_vel_x', 'trailer_vel_y', 'trailer_vel_z'])


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_steps = math.ceil(render_step_size / step_size)

while vis.Run():
    time = truck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    truck_inputs = truck_driver.GetInputs()
    sedan_inputs = sedan_driver.GetInputs()
    
    
    sedan_inputs.m_throttle = 0.7
    sedan_inputs.m_steering = 0.0
    sedan_inputs.m_braking = 0.0

    
    truck_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, truck_inputs)

    
    truck_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    tractor = truck.GetTractor()
    trailer = truck.GetTrailer()
    
    tractor_pos = tractor.GetPos().GetPos()
    tractor_vel = tractor.GetVel()
    trailer_pos = trailer.GetPos().GetPos()
    trailer_vel = trailer.GetVel()
    
    state_writer.writerow([time,
                           tractor_pos.x, tractor_pos.y, tractor_pos.z,
                           tractor_vel.x, tractor_vel.y, tractor_vel.z,
                           trailer_pos.x, trailer_pos.y, trailer_pos.z,
                           trailer_vel.x, trailer_vel.y, trailer_vel.z])

    step_number += 1
    realtime_timer.Spin(step_size)


state_file.close()