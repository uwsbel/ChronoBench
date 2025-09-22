import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc_truck = chrono.ChVector3d(0, -2, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc_sedan = chrono.ChVector3d(0, 2, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3
render_step_size = 1.0 / 50


trackPoint = chrono.ChVector3d(0, 0, 2.1)


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.SetTireType(tire_model_truck)  
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
sedan.SetTireType(tire_model_sedan)
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
highway_mesh = veh.GetDataFile("terrain/meshes/highway.obj")
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), highway_mesh)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Two Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())


driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


sedan_driver = veh.ChDriver(sedan.GetVehicle())
sedan_driver.Initialize()


print("TRUCK MASS:", truck.GetTractor().GetMass())
print("SEDAN MASS:", sedan.GetVehicle().GetMass())


state_file = open("truck_states.txt", "w")
state_file.write("Time Tractor_PosX Tractor_PosY Tractor_PosZ Tractor_QuatW Tractor_QuatX Tractor_QuatY Tractor_QuatZ Trailer_PosX Trailer_PosY Trailer_PosZ Trailer_QuatW Trailer_QuatX Trailer_QuatY Trailer_QuatZ\n")


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = truck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        tractor = truck.GetTractor()
        trailer = truck.GetTrailer()
        t_pos = tractor.GetChassisBody().GetPos()
        t_rot = tractor.GetChassisBody().GetRot()
        tr_pos = trailer.GetChassisBody().GetPos()
        tr_rot = trailer.GetChassisBody().GetRot()
        state_file.write(f"{time} ")
        state_file.write(f"{t_pos.x} {t_pos.y} {t_pos.z} ")
        state_file.write(f"{t_rot.e0} {t_rot.e1} {t_rot.e2} {t_rot.e3} ")
        state_file.write(f"{tr_pos.x} {tr_pos.y} {tr_pos.z} ")
        state_file.write(f"{tr_rot.e0} {tr_rot.e1} {tr_rot.e2} {tr_rot.e3}\n")
        state_file.flush()

    
    driver_inputs = driver.GetInputs()
    sedan_driver.Synchronize(time)
    sedan_inputs = sedan_driver.GetInputs()
    sedan_inputs.m_throttle = 0.7
    sedan_inputs.m_steering = 0.0
    sedan_inputs.m_braking = 0.0

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)


state_file.close()